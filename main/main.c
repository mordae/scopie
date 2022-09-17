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
#include "scope.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/dac_common.h>
#include <driver/ledc.h>
#include <soc/dac_channel.h>
#include <esp_adc/adc_continuous.h>

#include <esp_freertos_hooks.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_spiffs.h>
#include <esp_vfs.h>

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
 * Oscilloscopee
 */
static struct scope_config scope = {
	/* We keep these static: */
	.atten = ADC_ATTEN_DB_6,
	.channel = CONFIG_ADC_CH,
	.unit = ADC_UNIT_1,
	.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
	.multiplier = 2,

	/* And let user change these: */
	.freq_hz = 250000,
	.window_size = 510,
	.trigger = SCOPE_TRIGGER_RISING,
};


/*
 * Signal generators
 */
static int signal_freq = 1000;


/*
 * GUI
 */
#define MAX_MV 3400
static int v_scale = MAX_MV;
static int v_offset = 0;
static int h_offset = 0;
static bool hold = false;
static bool held = false;

static float average = MAX_MV / 2;

static SemaphoreHandle_t ui_signal = NULL;
static SemaphoreHandle_t paint_signal = NULL;

static TickType_t show_signal_freq_until = 0;

/* Historic windows. */
#define HISTORY_MAX 128
static uint16_t *history[HISTORY_MAX] = {0};
static unsigned history_idx = 0;
static unsigned history_offset = 0;


/*
 * Timing measurements
 */
static float time_to_plot = 0;
static float time_plot = 0;
static float time_to_paint = 0;
static float time_paint = 0;


/*
 * Rotary encoder.
 */
enum {
	MODE_SIGNAL = 0,
	MODE_V_OFFSET,
	MODE_H_OFFSET,
	MODE_V_SCALE,
	MODE_HOLD,
	MODE_TRIGGER,
	MODE_MAX,
};

static int mode = MODE_HOLD;

enum {
	SIGNAL_CW = 0,
	SIGNAL_PWM,
	SIGNAL_MAX,
};

static int signal_type = 0;


static void gui_loop(void *arg)
{
	uint16_t *vs = NULL;

	while (1) {
		TickType_t t0 = xTaskGetTickCount();

		if (!held) {
			uint16_t *newvs = scope_read(&average, pdMS_TO_TICKS(33));

			if (NULL != newvs) {
				history_idx = (history_idx + 1) % HISTORY_MAX;

				if (history[history_idx])
					free(history[history_idx]);

				history[history_idx] = newvs;
			}

			if (hold)
				held = true;
		}

		vs = history[(history_idx + HISTORY_MAX + history_offset) % HISTORY_MAX];

		TickType_t t1 = xTaskGetTickCount();
		lcd_draw_rect(0, CHROME, WIDTH - 1, HEIGHT - 1, BLACK);

		if (vs) {
			for (int i = 0; i < WIDTH; i++) {
				int vp = PLOT * (vs[h_offset + i] + v_offset) / v_scale;
				if (vp >= 0 && vp < PLOT)
					lcd_draw_pixel(i, CHROME + vp, WHITE);
			}
		}

		int ap = PLOT * (average + v_offset) / v_scale;

		if (ap >= 0 && ap < PLOT) {
			lcd_draw_rect(0, CHROME + ap, 99, CHROME + ap, LRED);

			for (int i = 1; i < 3; i++) {
				int x = 50 * i - 1;
				lcd_draw_rect(x, CHROME + ap - 4, x, CHROME + ap + 4, LRED);
			}

			for (int i = 1; i < 10; i++) {
				int x = i * 10 - 1;
				lcd_draw_rect(x, CHROME + ap - 2, x, CHROME + ap + 2, LRED);
			}

			lcd_draw_rect(100, CHROME + ap, WIDTH - 1, CHROME + ap, LGREEN);

			char buf[16];
			sprintf(buf, "%i mV", (int)average);

			if (ap > 16)
				lcd_draw_string(WIDTH - strlen(buf) * 8 - 1, CHROME + ap - 16, LGREEN, buf);
			else
				lcd_draw_string(WIDTH - strlen(buf) * 8 - 1, CHROME + ap + 2, LGREEN, buf);
		}

		lcd_draw_rect(0, 0, WIDTH - 1, CHROME - 1, DGRAY);

		char buf[32];

		if (show_signal_freq_until > xTaskGetTickCount()) {
			sprintf(buf, "%i.%.3i kHz", signal_freq / 1000, signal_freq % 1000);
			lcd_draw_string(0, 0, LBLUE, buf);
		} else {
			int freq_hz = 2 * scope.freq_hz / 5 / 100;
			sprintf(buf, "%i.%.3i kHz", freq_hz / 1000, freq_hz % 1000);
			lcd_draw_string(0, 0, LRED, buf);
		}

		if (mode == MODE_V_SCALE)
			strcpy(buf, "v-scale");
		else if (mode == MODE_V_OFFSET)
			strcpy(buf, "v-offset");
		else if (mode == MODE_H_OFFSET)
			strcpy(buf, "h-offset");
		else if (mode == MODE_SIGNAL)
			strcpy(buf, "signal");
		else if (mode == MODE_HOLD)
			strcpy(buf, "hold");
		else if (mode == MODE_TRIGGER)
			strcpy(buf, "trigger");

		lcd_draw_string(WIDTH - (8 * strlen(buf)), 0, WHITE, buf);

		char status[16] = "";

		if (hold) {
			/* Draw history offset. */
			char buf[8];
			sprintf(buf, "%i", history_offset);
			strcat(status, buf);

			/* Draw the pause symbol. */
			strcat(status, "\x1c");
		}

		/* Draw trigger method glyph. */
		if (SCOPE_TRIGGER_RISING == scope.trigger) {
			strcat(status, " \x1e");
		} else if (SCOPE_TRIGGER_FALLING == scope.trigger) {
			strcat(status, " \x1f");
		}

		/* Draw the status icons. */
		lcd_draw_string(WIDTH - strlen(status) * 8 - 1, HEIGHT - 17, LRED, status);

		TickType_t t2 = xTaskGetTickCount();
		time_to_plot = (time_to_plot * 15 + t1 - t0) / 16;
		time_plot = (time_plot * 15 + t2 - t1) / 16;

		xSemaphoreGive(paint_signal);

		/* We want ~30 fps. */
		if (t2 - t0 < pdMS_TO_TICKS(33))
			vTaskDelay(pdMS_TO_TICKS(33) - (t2 - t0));
	}
}


static void paint_loop(void *arg)
{
	while (1) {
		TickType_t t0 = xTaskGetTickCount();
		xSemaphoreTake(paint_signal, portMAX_DELAY);

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


static void reset_signal()
{
	static int prev_type = -1;
	static int prev_freq = -1;

	if (SIGNAL_PWM == signal_type && signal_freq < 550)
		signal_freq = 550;

	if (SIGNAL_CW == signal_type && signal_freq < 130)
		signal_freq = 130;

	if (SIGNAL_CW == signal_type && signal_freq > 55000)
		signal_freq = 55000;

	if (prev_type == signal_type && prev_freq == signal_freq)
		return;

	if (SIGNAL_CW == prev_type) {
		ESP_ERROR_CHECK(dac_cw_generator_disable());
		ESP_ERROR_CHECK(dac_output_disable(DAC_GPIO26_CHANNEL));
		gpio_reset_pin(26);
	}

	if (SIGNAL_PWM == prev_type) {
		ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE, 0, 0));
		gpio_reset_pin(25);
	}

	if (SIGNAL_CW == signal_type) {
		dac_cw_config_t cw_config = {
			.en_ch = DAC_GPIO26_CHANNEL,
			.scale = DAC_CW_SCALE_1,
			.phase = DAC_CW_PHASE_0,
			.freq = signal_freq,
			.offset = 0,
		};
		ESP_ERROR_CHECK(dac_cw_generator_config(&cw_config));
		ESP_ERROR_CHECK(dac_cw_generator_enable());
		dac_output_enable(DAC_GPIO26_CHANNEL);
	}

	if (SIGNAL_PWM == signal_type) {
		ledc_timer_config_t ledc_timer = {
			.duty_resolution = LEDC_TIMER_1_BIT,
			.freq_hz = signal_freq,
			.speed_mode = LEDC_LOW_SPEED_MODE,
			.timer_num = LEDC_TIMER_0,
			.clk_cfg = LEDC_AUTO_CLK,
		};
		ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

		ledc_channel_config_t ledc_channel = {
			.gpio_num   = 25,
			.channel    = 0,
			.duty       = 1,
			.speed_mode = LEDC_LOW_SPEED_MODE,
			.timer_sel  = LEDC_TIMER_0,
		};
		ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
	}

	if (prev_type != signal_type || prev_freq != signal_freq)
		show_signal_freq_until = xTaskGetTickCount() + pdMS_TO_TICKS(1000);

	prev_type = signal_type;
	prev_freq = signal_freq;
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
	                       1);

	int white = rotary_add(CONFIG_RE3_SW_PIN,
	                       CONFIG_RE3_LEFT_PIN,
	                       CONFIG_RE3_RIGHT_PIN,
	                       1);

	int blue = rotary_add(CONFIG_RE4_SW_PIN,
	                      CONFIG_RE4_LEFT_PIN,
	                      CONFIG_RE4_RIGHT_PIN,
	                      200);

	assert (red >= 0);
	assert (green >= 0);
	assert (white >= 0);
	assert (blue >= 0);

	while (1) {
		rotary_wait(portMAX_DELAY);

		int red_steps = rotary_read_steps(red);
		int green_steps = rotary_read_steps(green);
		int white_steps = rotary_read_steps(white);
		int blue_steps = rotary_read_steps(blue);

		if (red_steps) {
			scope.freq_hz = clamp(scope.freq_hz + red_steps * 250,
					      SOC_ADC_SAMPLE_FREQ_THRES_LOW,
					      SOC_ADC_SAMPLE_FREQ_THRES_HIGH);

			scope.window_size = scope.freq_hz < 100000 ? 220 : 510;
			h_offset = clamp(h_offset, 0, scope.window_size - WIDTH);

			scope_config(&scope);
			held = false;
		}

		while (white_steps > 0) {
			mode = (mode + 1 >= MODE_MAX) ? 0 : mode + 1;
			white_steps--;
		}

		while (white_steps < 0) {
			mode = (mode - 1) < 0 ? MODE_MAX - 1 : mode - 1;
			white_steps++;
		}

		if (MODE_V_SCALE == mode && green_steps) {
			v_scale = clamp(v_scale - green_steps * 100, 500, MAX_MV);
			ESP_LOGI(tag, "config: v_scale=%i", v_scale);
		}
		else if (MODE_V_OFFSET == mode && green_steps) {
			v_offset = clamp(v_offset + green_steps * 50, -MAX_MV, MAX_MV);
			ESP_LOGI(tag, "config: v_offset=%i", v_offset);
		}
		else if (MODE_H_OFFSET == mode && green_steps) {
			h_offset = clamp(h_offset + green_steps * 10, 0, scope.window_size - WIDTH);
			ESP_LOGI(tag, "config: h_offset=%i", h_offset);
		}
		else if (MODE_SIGNAL == mode && green_steps) {
			while (green_steps > 0) {
				signal_type = signal_type + 1 >= SIGNAL_MAX ? 0 : signal_type + 1;
				green_steps--;
			}

			while (green_steps < 0) {
				signal_type = signal_type - 1 < 0 ? SIGNAL_MAX - 1 : signal_type - 1;
				green_steps++;
			}

			reset_signal();
			held = false;
		}
		else if (MODE_HOLD == mode && green_steps) {
			int new_offset = history_offset + green_steps;
			history_offset = clamp(new_offset, 1 - HISTORY_MAX, 0);

			if (0 == history_offset) {
				hold = false;
				held = false;
			} else {
				hold = true;
				held = true;
			}
		}
		else if (MODE_TRIGGER == mode && green_steps) {
			while (green_steps > 0) {
				scope.trigger = scope.trigger + 1 >= SCOPE_TRIGGER_MAX ? 0 : scope.trigger + 1;
				green_steps--;
			}

			while (green_steps < 0) {
				scope.trigger = scope.trigger - 1 < 0 ? SCOPE_TRIGGER_MAX - 1 : scope.trigger - 1;
				green_steps++;
			}

			scope_config(&scope);
			held = false;
		}

		if (blue_steps) {
			signal_freq = clamp(signal_freq + blue_steps, 0, 200000);
			reset_signal();
			ESP_LOGI(tag, "signal: type=%i freq=%i", signal_type, signal_freq);
		}
	}
}


static unsigned idle_cpu0 = 0;
static unsigned idle_cpu1 = 0;

static bool add_idle_cpu0(void)
{
	idle_cpu0++;
	return true;
}

static bool add_idle_cpu1(void)
{
	idle_cpu1++;
	return true;
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

	ESP_LOGI(tag, "Start cosine wave generator...");
	reset_signal();

	ui_signal = xSemaphoreCreateBinary();
	assert (NULL != ui_signal);

	paint_signal = xSemaphoreCreateBinary();
	assert (NULL != paint_signal);

	ESP_LOGI(tag, "Start the oscilloscope...");
	scope_init(10, 0);
	scope_config(&scope);

	ESP_LOGI(tag, "Start input processing task...");
	xTaskCreatePinnedToCore(input_loop, "input", 4096, NULL, 4, NULL, 0);

	ESP_LOGI(tag, "Start the GUI loop...");
	xTaskCreatePinnedToCore(gui_loop, "gui", 4096, NULL, 2, NULL, 1);

	ESP_LOGI(tag, "Start the paint loop...");
	xTaskCreatePinnedToCore(paint_loop, "paint", 4096, NULL, 3, NULL, 0);

	ESP_ERROR_CHECK(esp_register_freertos_idle_hook_for_cpu(add_idle_cpu0, 0));
	ESP_ERROR_CHECK(esp_register_freertos_idle_hook_for_cpu(add_idle_cpu1, 1));

	while (1) {
		ESP_LOGI(tag, "timing: adc=%-2.1f math=%-2.1f send=%-2.1f to_plot=%-2.1f plot=%-2.1f to_paint=%-2.1f paint=%-2.1f",
			 scope_adc_ticks, scope_math_ticks, scope_send_ticks, time_to_plot, time_plot, time_to_paint, time_paint);

		multi_heap_info_t heap;
		heap_caps_get_info(&heap, MALLOC_CAP_INTERNAL);

		ESP_LOGI(tag, "memory: free=%zu used=%zu watermark=%zu largest=%zu",
		         heap.total_free_bytes, heap.total_allocated_bytes,
			 heap.minimum_free_bytes, heap.largest_free_block);

		ESP_LOGI(tag, " scope: dropped_frames=%u", scope_dropped_frames);

		ESP_LOGI(tag, "  idle: cpu0=%u cpu1=%u", idle_cpu0 / 2, idle_cpu1 / 2);
		idle_cpu0 = idle_cpu1 = 0;

		vTaskDelay(pdMS_TO_TICKS(2000));
	}
}
