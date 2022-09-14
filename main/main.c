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

#include "ili9225.h"
#include "rotary.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/dac_common.h"
#include "soc/dac_channel.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_vfs.h"

#include <string.h>
#include <math.h>


static const char *tag = "main";


/*
 * Display
 */
#define WIDTH	220
#define HEIGHT	176
#define PLOT	160
#define CHROME	16

enum {
	BLACK = 0,
	DGRAY, LGRAY, WHITE,
	RED, GREEN, BLUE,
	LRED, LGREEN, LBLUE,
	YELLOW, CYAN, PURPLE,
	DYELLOW, DCYAN, DPURPLE,
};


/*
 * ADC
 */
#define ADC_FRAME 1000
#define ADC_BUFFER (ADC_FRAME * SOC_ADC_DIGI_DATA_BYTES_PER_CONV)

static int freq_hz = 100000;
static int cw_freq = 50320;

static adc_continuous_handle_t cadc;
static adc_cali_handle_t cali;

static uint8_t frame[ADC_BUFFER];

static uint16_t volts_array[3][ADC_FRAME];
static uint16_t *volts;


/*
 * UI
 */
static int zoom = 3300;
static int offset = 0;

static float average = 3300 / 2;
static int averages[WIDTH] = {0};

static SemaphoreHandle_t ui_semaphore = NULL;
static SemaphoreHandle_t paint_semaphore = NULL;


/*
 * Timing measurements
 */
static float time_to_math = 0;
static float time_math = 0;
static float time_plot = 0;
static float time_adc = 0;
static float time_to_paint = 0;
static float time_paint = 0;


/*
 * Rotary encoder.
 */
enum {
	M_ZOOM = 0,
	M_OFFSET,
	M_MAX,
};

static volatile int mode = M_ZOOM;


static void cadc_before_read(void)
{
	static int prev_hz = 0;

	if (prev_hz == freq_hz)
		return;

	if (prev_hz) {
		ESP_ERROR_CHECK(adc_continuous_stop(cadc));
		ESP_ERROR_CHECK(adc_continuous_deinit(cadc));
	}

	adc_continuous_handle_cfg_t adc_config = {
		.max_store_buf_size = ADC_BUFFER,
		.conv_frame_size = ADC_BUFFER,
	};
	ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &cadc));

	adc_continuous_config_t dig_cfg = {
		.sample_freq_hz = freq_hz,
		.conv_mode = ADC_CONV_SINGLE_UNIT_1,
		.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
	};

	adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
	dig_cfg.pattern_num = 1;
	dig_cfg.adc_pattern = adc_pattern;

	adc_pattern[0].unit = ADC_UNIT_1;
	adc_pattern[0].channel = CONFIG_ADC_3V3_CH;
	adc_pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
	adc_pattern[0].atten = ADC_ATTEN_DB_11;

	ESP_ERROR_CHECK(adc_continuous_config(cadc, &dig_cfg));
	ESP_ERROR_CHECK(adc_continuous_start(cadc));

	prev_hz = freq_hz;
}


static void oscilloscope_loop(void *arg)
{
	while (1) {
		cadc_before_read();

		uint32_t size = 0;

		TickType_t t0 = xTaskGetTickCount();

		ESP_ERROR_CHECK(adc_continuous_read(cadc, frame, ADC_BUFFER, &size, 1000));

		if (size < ADC_BUFFER) {
			ESP_LOGE(tag, "ADC did not return a full frame, just %lu bytes.", size);
			continue;
		}

		uint16_t *voltsptr = volts;
		int total = 0;

		for (int i = 0; i < ADC_BUFFER; i += SOC_ADC_DIGI_DATA_BYTES_PER_CONV) {
			adc_digi_output_data_t *p = (void *)(frame + i);
			int vs = 0;
			ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali, p->type1.data, &vs));
			*voltsptr++ = vs;
			total += vs;
		}

		average = ((average * 31) + ((float)total / ADC_FRAME)) / 32;

		TickType_t t1 = xTaskGetTickCount();
		time_adc = (time_adc * 15 + t1 - t0) / 16;

		xSemaphoreGive(ui_semaphore);
	}
}


static void ui_loop(void *arg)
{
	unsigned bufno = 0;

	while (1) {
		TickType_t t0 = xTaskGetTickCount();
		xSemaphoreTake(ui_semaphore, portMAX_DELAY);

		TickType_t t1 = xTaskGetTickCount();

		/* Use the next buffer, please. */
		bufno = (bufno + 1) % 3;
		volts = volts_array[bufno];

		/* -2th is the current one. */
		uint16_t *vs = volts_array[(bufno - 2) % 3];

		uint32_t total = 0;

		for (uint32_t i = 0; i < ADC_FRAME; i++)
			total += vs[i];

		int start = 0;
		int deviation = INT_MAX;

		for (int i = 0; i < ADC_FRAME - WIDTH; i++) {
			float dev = 0;

			for (int j = 0; j < WIDTH; j++) {
				int d = abs((averages[j] >> 8) - vs[i + j]);
				dev += d * d;
			}

			if (dev < deviation) {
				deviation = dev;
				start = i;
			}
		}

		for (int i = 0; i < WIDTH; i++)
			averages[i] = ((averages[i] << 3) - averages[i] + (vs[start + i] << 8)) >> 3;

		TickType_t t2 = xTaskGetTickCount();

		lcd_draw_rect(0, CHROME, WIDTH - 1, HEIGHT - 1, BLACK);

		for (int i = start; i < start + WIDTH; i++) {
			int avp = PLOT * ((averages[i - start] >> 8) + offset) / zoom;
			if (avp >= 0 && avp < PLOT)
				lcd_draw_pixel(i - start, CHROME + avp, DPURPLE);

			int vp = PLOT * (vs[i] + offset) / zoom;
			if (vp >= 0 && vp < PLOT)
				lcd_draw_pixel(i - start, CHROME + vp, WHITE);
		}

		int ap = PLOT * (average + offset) / zoom;

		if (ap >= 0 && ap < PLOT) {
			lcd_draw_rect(0, CHROME + ap, 99, CHROME + ap, LRED);
			lcd_draw_rect(100, CHROME + ap, WIDTH - 1, CHROME + ap, LGREEN);

			char buf[16];
			sprintf(buf, "%i mV", (int)average);

			if (ap > 16)
				lcd_draw_string(WIDTH - strlen(buf) * 8 - 1, CHROME + ap - 16, LGREEN, buf);
			else
				lcd_draw_string(WIDTH - strlen(buf) * 8 - 1, CHROME + ap + 16, LGREEN, buf);
		}

		lcd_draw_rect(0, 0, WIDTH - 1, CHROME - 1, DGRAY);

		char buf[32];

		sprintf(buf, "%i.%i Hz", freq_hz / 100, (freq_hz % 100) / 10);
		lcd_draw_string(0, 0, LRED, buf);

		if (mode == M_ZOOM)
			strcpy(buf, "zoom");
		else if (mode == M_OFFSET)
			strcpy(buf, "offset");

		lcd_draw_string(WIDTH - (8 * strlen(buf)), 0, WHITE, buf);

		TickType_t t3 = xTaskGetTickCount();

		time_to_math = (time_to_math * 15 + t1 - t0) / 16;
		time_math = (time_math * 15 + t2 - t1) / 16;
		time_plot = (time_plot * 15 + t3 - t2) / 16;

		xSemaphoreGive(paint_semaphore);
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}


static void paint_loop(void *arg)
{
	while (1) {
		TickType_t t0 = xTaskGetTickCount();
		xSemaphoreTake(paint_semaphore, portMAX_DELAY);

		TickType_t t1 = xTaskGetTickCount();
		lcd_sync();

		TickType_t t2 = xTaskGetTickCount();

		time_to_paint = (time_to_paint * 15 + t1 - t0) / 16;
		time_paint = (time_paint * 15 + t2 - t1) / 16;
	}
}


static int clamp(int x, int min, int max)
{
	if (x < min)
		return min;

	if (x > max)
		return max;

	return x;
}


static void input_loop(void *arg)
{
	ESP_LOGI(tag, "Register input handlers...");
	ESP_ERROR_CHECK(gpio_install_isr_service(0));

	int red = rotary_add(CONFIG_RE1_SW_PIN,
	                     CONFIG_RE1_LEFT_PIN,
	                     CONFIG_RE1_RIGHT_PIN,
	                     200);

	int green = rotary_add(CONFIG_RE2_SW_PIN,
	                       CONFIG_RE2_LEFT_PIN,
	                       CONFIG_RE2_RIGHT_PIN,
	                       100);

	int white = rotary_add(CONFIG_RE3_SW_PIN,
	                       CONFIG_RE3_LEFT_PIN,
	                       CONFIG_RE3_RIGHT_PIN,
	                       1);

	int blue = rotary_add(CONFIG_RE4_SW_PIN,
	                      CONFIG_RE4_LEFT_PIN,
	                      CONFIG_RE4_RIGHT_PIN,
	                      100);

	assert (red >= 0);
	assert (green >= 0);
	assert (white >= 0);
	assert (blue >= 0);

	while (1) {
		int red_steps = rotary_read_steps(red);
		int green_steps = rotary_read_steps(green);
		int white_steps = rotary_read_steps(white);
		int blue_steps = rotary_read_steps(blue);

		freq_hz = clamp(freq_hz + red_steps * 10,
				SOC_ADC_SAMPLE_FREQ_THRES_LOW,
				SOC_ADC_SAMPLE_FREQ_THRES_HIGH);

		while (white_steps > 0) {
			if (mode + 1 >= M_MAX)
				mode = 0;
			else
				mode += 1;

			white_steps--;
		}

		while (white_steps < 0) {
			if (mode - 1 < 0)
				mode = M_MAX - 1;
			else
				mode--;

			white_steps++;
		}

		if (M_ZOOM == mode) {
			zoom = clamp(zoom - green_steps, 500, 3300);
		} else if (M_OFFSET == mode) {
			offset = clamp(offset - green_steps, -3200, 0);
		}

		if (blue_steps) {
			cw_freq = clamp(cw_freq + blue_steps * 130, 130, 55000);
			ESP_LOGI(tag, "cw: raw_freq=%i", cw_freq);

			ESP_ERROR_CHECK(dac_cw_generator_disable());
			dac_cw_config_t cw_config = {
				.en_ch = DAC_GPIO26_CHANNEL,
				.scale = DAC_CW_SCALE_1,
				.phase = DAC_CW_PHASE_0,
				.freq = cw_freq,
				.offset = 0,
			};
			ESP_ERROR_CHECK(dac_cw_generator_config(&cw_config));
			ESP_ERROR_CHECK(dac_cw_generator_enable());
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}


void app_main(void)
{
	ESP_LOGI(tag, "Initialize SPIFFS...");

	esp_vfs_spiffs_conf_t conf = {
		.base_path = "/spiffs",
		.max_files = 16,
	};
	ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

	ESP_LOGI(tag, "Initialize SPI3 host...");
	spi_bus_config_t spi_config = {
		.mosi_io_num = 23,
		.sclk_io_num = 18,
	};
	ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &spi_config, SPI_DMA_CH_AUTO));

	ESP_LOGI(tag, "Initialize ILI9225 screen...");
	lcd_init(CONFIG_LCD_RS_PIN, CONFIG_LCD_RST_PIN);
	lcd_load_font("/spiffs/HaxorMedium-13.bin");

	ESP_LOGI(tag, "Prepare ADC calibration...");
	adc_cali_line_fitting_config_t cali_config = {
		.unit_id = ADC_UNIT_1,
		.atten = ADC_ATTEN_DB_11,
		.bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
	};
	ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali));

	ESP_LOGI(tag, "Start cosine wave generator...");
	dac_output_enable(DAC_GPIO26_CHANNEL);

	dac_cw_config_t cw_config = {
		.en_ch = DAC_GPIO26_CHANNEL,
		.scale = DAC_CW_SCALE_1,
		.phase = DAC_CW_PHASE_0,
		.freq = cw_freq, /* ~100kHz */
		.offset = 0,
	};

	ESP_ERROR_CHECK(dac_cw_generator_config(&cw_config));
	ESP_ERROR_CHECK(dac_cw_generator_enable());

	ui_semaphore = xSemaphoreCreateBinary();
	assert (NULL != ui_semaphore);

	paint_semaphore = xSemaphoreCreateBinary();
	assert (NULL != paint_semaphore);

	ESP_LOGI(tag, "Start input processing task...");
	xTaskCreate(input_loop, "input", 4096, NULL, 1, NULL);

	ESP_LOGI(tag, "Start the UI loop...");
	xTaskCreate(ui_loop, "ui", 4096, NULL, 1, NULL);

	ESP_LOGI(tag, "Start the paint loop...");
	xTaskCreate(paint_loop, "paint", 4096, NULL, 1, NULL);

	ESP_LOGI(tag, "Start the oscilloscope...");
	volts = volts_array[0];
	xTaskCreate(oscilloscope_loop, "oscilloscope", 4096, NULL, 1, NULL);

	while (1) {
		ESP_LOGI(tag, "timing: adc=%-2.1f to_math=%-2.1f math=%-2.1f plot=%-2.1f to_paint=%-2.1f paint=%-2.1f",
			 time_adc, time_to_math, time_math, time_plot, time_to_paint, time_paint);

		multi_heap_info_t heap;
		heap_caps_get_info(&heap, MALLOC_CAP_INTERNAL);

		ESP_LOGI(tag, "memory: free=%zu used=%zu watermark=%zu largest=%zu",
		         heap.total_free_bytes, heap.total_allocated_bytes,
			 heap.minimum_free_bytes, heap.largest_free_block);

		vTaskDelay(pdMS_TO_TICKS(2000));
	}
}
