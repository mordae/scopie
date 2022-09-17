/*
 * Copyright (C) 2022 Jan Hamal Dvořák <mordae@anilinux.org>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "scope.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_continuous.h>
#include <esp_log.h>
#include <hal/adc_ll.h>

#include <string.h>
#include <math.h>


/* For logging. */
static const char *tag = "scope";


/* Current configuration. */
struct scope_config config;


/* Continuous ADC handle. */
static adc_continuous_handle_t cadc;

/* ADC calibration handle. */
static adc_cali_handle_t cali;

/* To send configuration changes through. */
QueueHandle_t config_queue;
static SemaphoreHandle_t config_counter;

/* To wait for a new window from scope_read(). */
static QueueHandle_t window_queue;

/* To notify window sender that the configuration has changed and
 * it should cease enqueing new windows and reconfigure immediately. */
static SemaphoreHandle_t window_interrupt;


/* Maximum window size. */
#define MAX_WINDOW_SIZE 510

/* How many samples to read at once. */
#define FRAME_LEN 1020
#define FRAME_BUFFER_LEN (FRAME_LEN * SOC_ADC_DIGI_DATA_BYTES_PER_CONV)

/* Size of the trigger buffer. */
#define TRIGGER_LEN 1020

/* Size of the trigger zone. */
#define TRIGGER_ZONE 510


/* Long-term average voltage. */
static float average_voltage = 1000;

/* How long have we waited for ADC on average. */
float scope_adc_ticks = 0;

/* How long have we spent doing the trigger math. */
float scope_math_ticks = 0;

/* How long have we spent waiting on client to pick up the window. */
float scope_send_ticks = 0;

/* Number of times a frame was dropped. */
unsigned scope_dropped_frames = 0;

/* Static working buffers. */
static uint8_t frame_buffer[FRAME_BUFFER_LEN];
static uint16_t trigger_buffer[TRIGGER_LEN];
static uint16_t sample_buffer[TRIGGER_LEN];


/* Try to find a rising or falling edge. */
static int trigger_edge(uint16_t *samples)
{
	if (samples[0] <= average_voltage && samples[1] > average_voltage)
		return 1;

	if (samples[0] >= average_voltage && samples[1] < average_voltage)
		return -1;

	return 0;
}


/* Try to find a match using the current configuration. */
static int trigger(uint16_t *samples)
{
	if (SCOPE_TRIGGER_NONE == config.trigger) {
		/* Instant success! */
		return 0;
	}
	else if (SCOPE_TRIGGER_RISING == config.trigger) {
		for (int i = 10; i < TRIGGER_ZONE; i++)
			if (trigger_edge(samples + i) > 0)
				return i;
	}
	else if (SCOPE_TRIGGER_FALLING == config.trigger) {
		for (int i = 10; i < TRIGGER_ZONE; i++)
			if (trigger_edge(samples + i) < 0)
				return i;
	}

	return -1;
}


static bool apply_config(void);


/* Background task collecting the data. */
static void scope_loop(void *arg)
{
	/* Fast changing average for low-pass filtering samples to the trigger buffer. */
	unsigned fast_average = 0;

	memset(frame_buffer, 0, sizeof(frame_buffer));
	memset(sample_buffer, 0, sizeof(sample_buffer));
	memset(trigger_buffer, 0, sizeof(trigger_buffer));

	/* Wait until we are configured. */
	while (!config.freq_hz) {
		apply_config();
		vTaskDelay(pdMS_TO_TICKS(100));
	}

	while (1) {
		TickType_t t0 = xTaskGetTickCount();

		/* Update configuration. */
		apply_config();

		uint32_t size;
		ESP_ERROR_CHECK(adc_continuous_start(cadc));
		ESP_ERROR_CHECK(adc_continuous_read(cadc, frame_buffer, FRAME_BUFFER_LEN, &size, 1000));
		ESP_ERROR_CHECK(adc_continuous_stop(cadc));
		assert (FRAME_BUFFER_LEN == size);

		TickType_t t1 = xTaskGetTickCount();
		scope_adc_ticks = (scope_adc_ticks * 15 + t1 - t0) / 16;

		/* For the average voltage. */
		unsigned total = 0;

		/* Clear the new samples, we are going to add to them. */
		memset(sample_buffer, 0, sizeof(uint16_t) * TRIGGER_LEN);

		/* Add in the new voltages. */
		for (int i = 0; i < FRAME_LEN; i++) {
			int offset = i * SOC_ADC_DIGI_DATA_BYTES_PER_CONV;
			adc_digi_output_data_t *p = (void *)(frame_buffer + offset);

			int voltage;
			ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali, p->type1.data, &voltage));
			voltage *= config.multiplier;
			sample_buffer[i] += voltage;
			total += voltage;
		}

		/* Start filtering samples for the trigger procedure. */
		for (int i = 0; i < TRIGGER_LEN; i++) {
			trigger_buffer[i] = (fast_average + sample_buffer[i]) >> 1;
			fast_average = (fast_average * 3 + trigger_buffer[i]) >> 2;
		}

		/* Finish filtering in the reverse direction. */
		for (int i = TRIGGER_LEN - 1; i >= 0; i--) {
			trigger_buffer[i] = (fast_average + trigger_buffer[i]) >> 1;
			fast_average = (fast_average * 3 + trigger_buffer[i]) >> 2;
		}

		/* Update the average voltage. */
		float voltage = (float)total / FRAME_LEN;
		average_voltage = ((average_voltage * 255) + voltage) / 256;

		TickType_t t2 = xTaskGetTickCount();
		scope_math_ticks = (scope_math_ticks * 15 + t2 - t1) / 16;

		/* Make ourselves interruptible. */
		if (xSemaphoreTake(window_interrupt, 0))
			continue;

		int match = trigger(trigger_buffer);
		if (match >= 0) {
			/* Create a new window and send it to the client. */
			uint16_t *window = malloc(sizeof(uint16_t) * config.window_size);
			memcpy(window, sample_buffer + match,
			       sizeof(uint16_t) * config.window_size);

			TickType_t t0 = xTaskGetTickCount();
			while (!xQueueSend(window_queue, &window, pdMS_TO_TICKS(5))) {
				if (xSemaphoreTake(window_interrupt, 0)) {
					xSemaphoreGive(window_interrupt);
					free(window);
					break;
				}
			}
			TickType_t t1 = xTaskGetTickCount();
			scope_send_ticks = (scope_send_ticks * 15 + t1 - t0) / 16;
		}
	}
}


void scope_init(int pri, int core)
{
	config_counter = xSemaphoreCreateCounting(1000, 0);
	config_queue = xQueueCreate(1, sizeof(struct scope_config));
	window_queue = xQueueCreate(1, sizeof(uint16_t *));
	window_interrupt = xSemaphoreCreateBinary();

	BaseType_t res;

	if (core < 0) {
		res = xTaskCreate(scope_loop, "scope", 4096, NULL, pri, NULL);
	} else {
		res = xTaskCreatePinnedToCore(scope_loop, "scope", 4096, NULL, pri, NULL, core);
	}

	assert (pdPASS == res);
}


static bool on_pool_ovf(adc_continuous_handle_t handle,
                        const adc_continuous_evt_data_t *data,
                        void *arg)
{
	scope_dropped_frames++;
	return false;
}


static void reconfigure_adc(struct scope_config *cfg)
{
	if (!cadc) {
		adc_continuous_handle_cfg_t adc_config = {
			.max_store_buf_size = 8 * FRAME_BUFFER_LEN,
			.conv_frame_size = FRAME_BUFFER_LEN,
		};
		ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &cadc));

		adc_continuous_evt_cbs_t cbs = {
			.on_conv_done = NULL,
			.on_pool_ovf = on_pool_ovf,
		};
		ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(cadc, &cbs, NULL));
	}

	adc_continuous_config_t dig_cfg = {
		.sample_freq_hz = cfg->freq_hz,
		.conv_mode = ADC_UNIT_1 == cfg->unit
		           ? ADC_CONV_SINGLE_UNIT_1
		           : ADC_CONV_SINGLE_UNIT_2,
		.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
	};

	adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
	dig_cfg.pattern_num = 1;
	dig_cfg.adc_pattern = adc_pattern;

	adc_pattern[0].unit = cfg->unit;
	adc_pattern[0].channel = cfg->channel;
	adc_pattern[0].bit_width = cfg->bit_width;
	adc_pattern[0].atten = cfg->atten;

	ESP_ERROR_CHECK(adc_continuous_config(cadc, &dig_cfg));
}


void reconfigure_cali(struct scope_config *cfg)
{
	if (config.freq_hz)
		ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(cali));

	adc_cali_line_fitting_config_t cali_config = {
		.unit_id = cfg->unit,
		.atten = cfg->atten,
		.bitwidth = cfg->bit_width,
	};
	ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali));
}


static bool apply_config(void)
{
	struct scope_config new = {0};
	bool applied = false;

	xSemaphoreTake(window_interrupt, 0);

	while (xQueueReceive(config_queue, &new, 0)) {
		if ((config.freq_hz != new.freq_hz) ||
		    (config.atten != new.atten) ||
		    (config.channel != new.channel) ||
		    (config.unit != new.unit) ||
		    (config.bit_width != new.bit_width))
		{
			ESP_LOGI(tag, "adc: freq_hz=%i atten=%i chan=%i unit=%i bits=%i",
				 new.freq_hz, new.atten, new.channel, new.unit, new.bit_width);
			reconfigure_adc(&new);
			reconfigure_cali(&new);
		}

		if ((config.trigger != new.trigger) ||
		    (config.window_size != new.window_size) ||
		    (config.multiplier != new.multiplier))
		{
			ESP_LOGI(tag, "capture: trigger=%u, window=%u mul=%u",
				 new.trigger, new.window_size, new.multiplier);

			/* Will be picked up by the task automatically. */
		}

		/* Let the task know. */
		memcpy(&config, &new, sizeof(config));
		applied = true;

		/* Notify scope_config(). */
		xSemaphoreGive(config_counter);
	}

	return applied;
}


void scope_config(struct scope_config *cfg)
{
	assert (cfg->trigger >= 0 && cfg->trigger < SCOPE_TRIGGER_MAX);
	assert (cfg->window_size > 0 && cfg->window_size <= MAX_WINDOW_SIZE);

	/* Take any previously sent configuration to save time. */
	struct scope_config null;
	xQueueReceive(config_queue, &null, 0);

	/* Send new configuration. */
	xQueueSend(config_queue, cfg, portMAX_DELAY);
	xSemaphoreGive(window_interrupt);

	/* Wait for it to get processed. */
	xSemaphoreTake(config_counter, portMAX_DELAY);
}


uint16_t *scope_read(float *average, TickType_t wait)
{
	/* Get a new window. */
	uint16_t *window;
	if (!xQueueReceive(window_queue, &window, wait))
		return NULL;

	/* Report the long-term average voltage. */
	if (average)
		*average = average_voltage;

	/* Return it, it's up to the caller to free it. */
	return window;
}
