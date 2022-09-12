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

#include "ili9340.h"
#include "fontx.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/dac_common.h"
#include "soc/dac_channel.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
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
#define PLOT	150

static fontx_file_t font[1];
static tft_t tft[1];

/*
 * ADC
 */
#define ADC_FRAME (WIDTH * 4)
#define ADC_BUFFER (ADC_FRAME * SOC_ADC_DIGI_DATA_BYTES_PER_CONV)

static int freq_hz = 100000;

static adc_continuous_handle_t cadc;
static adc_cali_handle_t cali;

static uint8_t frame[ADC_BUFFER];

static uint16_t volts_array[3][ADC_FRAME];
static uint16_t *volts;

static int zoom = 3300;
static int offset = 0;

float average = 3300 / 2;
int averages[WIDTH] = {0};


/*
 * Rotary encoder.
 */
enum {
	M_FREQ = 0,
	M_ZOOM,
	M_OFFSET,
	M_MAX,
};

enum {
	DIR_CW	= 0x10,
	DIR_CCW	= 0x20,
};

enum {
	R_START = 0,
	R_CCW_BEGIN,
	R_CW_BEGIN,
	R_START_M,
	R_CW_BEGIN_M,
	R_CCW_BEGIN_M,
};

static volatile IRAM_ATTR int r_state = R_START;
static volatile IRAM_ATTR int r_direction = 0;
static volatile IRAM_ATTR int r_pressed = 0;
static volatile IRAM_ATTR int r_mode = 0;


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
	adc_pattern[0].channel = ADC_CHANNEL_6;
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
	}
}


static void display_loop(void *arg)
{
	unsigned bufno = 0;
	int prev_hz = 0;
	int prev_mode = -1;

	while (1) {
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

		for (int i = start; i < start + WIDTH; i++) {
			uint16_t colors[PLOT] = {BLACK};

			int avp = PLOT * ((averages[i - start] >> 8) + offset) / zoom;
			if (avp >= 0 && avp < PLOT)
				colors[avp] = PURPLE & GRAY;

			int vp = PLOT * (vs[i] + offset) / zoom;
			if (vp >= 0 && vp < PLOT)
				colors[vp] = WHITE;

			int ap = PLOT * (average + offset) / zoom;
			if (ap >= 0 && ap < PLOT)
				colors[ap] = i - start < 100 ? BLUE : GREEN;

			lcd_draw_multi_pixels(tft, HEIGHT - PLOT, i - start, PLOT, colors);
		}

		if ((prev_hz != freq_hz) || (prev_mode != r_mode)) {
			prev_hz = freq_hz;
			prev_mode = r_mode;

			lcd_draw_fill_rect(tft, 0, 0, HEIGHT - PLOT - 1, WIDTH, DARK);

			char buf[32];

			sprintf(buf, "%i.%i Hz", freq_hz / 100, (freq_hz % 100) / 10);
			lcd_draw_string(tft, font, 2, 2, buf, BLUE);

			if (r_mode == M_FREQ)
				strcpy(buf, "freq.");
			else if (r_mode == M_ZOOM)
				strcpy(buf, "zoom");
			else if (r_mode == M_OFFSET)
				strcpy(buf, "offset");

			lcd_draw_string(tft, font, 2, WIDTH - (8 * strlen(buf)), buf, RED);
		}
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


static void rotary_change(void *arg)
{
	r_pressed = !gpio_get_level(13);

	const uint8_t r_table[6][4] = {
		{R_START_M,           R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
		{R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START},
		{R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START},
		{R_START_M,           R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
		{R_START_M,           R_START_M,      R_CW_BEGIN_M, R_START},
		{R_START_M,           R_CCW_BEGIN_M,  R_START_M,    R_START},
	};

	uint8_t pinstate = (!gpio_get_level(4) << 1) | (!gpio_get_level(2));
	r_state = r_table[r_state & 0xf][pinstate];

	if (r_state & (DIR_CW | DIR_CCW)) {
		static TickType_t prev = 0;
		TickType_t now = xTaskGetTickCountFromISR() / portTICK_PERIOD_MS;

		if (now <= prev) {
			prev = now - 1;
		}
		else if (now - prev > 200) {
			prev = now - 200;
		}

		int speed = 200 / (now - prev);
		speed = speed * speed;
		prev = now;

		if (r_state & DIR_CW) {
			r_state &= 0xf;
			r_direction += speed;
		}
		else if (r_state & DIR_CCW) {
			r_state &= 0xf;
			r_direction -= speed;
		}
	}
}


static void input_loop(void *arg)
{
	ESP_LOGI(tag, "Register input handlers...");
	ESP_ERROR_CHECK(gpio_install_isr_service(0));

	ESP_ERROR_CHECK(gpio_isr_handler_add(2, rotary_change, NULL));
	ESP_ERROR_CHECK(gpio_isr_handler_add(4, rotary_change, NULL));
	ESP_ERROR_CHECK(gpio_isr_handler_add(13, rotary_change, NULL));

	gpio_config_t config = {
		.pin_bit_mask = BIT64(4) | BIT64(2) | BIT64(13),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = 1,
		.intr_type = GPIO_INTR_ANYEDGE,
	};
	ESP_ERROR_CHECK(gpio_config(&config));

	while (1) {
		rotary_change(NULL);

		if (r_pressed) {
			if (r_direction > 0) {
				if (r_mode + 1 >= M_MAX)
					r_mode = 0;
				else
					r_mode++;
			}

			if (r_direction < 0) {
				if (r_mode - 1 < 0)
					r_mode = M_MAX - 1;
				else
					r_mode--;
			}

			r_direction = 0;
		}
		else if (r_direction) {
			if (M_FREQ == r_mode) {
				freq_hz = clamp(freq_hz + r_direction * 10,
				                SOC_ADC_SAMPLE_FREQ_THRES_LOW,
				                SOC_ADC_SAMPLE_FREQ_THRES_HIGH);
			}
			else if (M_ZOOM == r_mode) {
				zoom = clamp(zoom - r_direction * 3, 500, 3300);
			}
			else if (M_OFFSET == r_mode) {
				offset = clamp(offset - r_direction * 10, -3200, 0);
			}

			r_direction = 0;
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

	ESP_LOGI(tag, "Load font...");
	fontx_init(font, "/spiffs/ILGH16XB.FNT", "");

	ESP_LOGI(tag, "Initialize screen...");
	spi_master_init(tft, 19, 18, 5, 17, 16, -1, -1, -1);
	lcd_init(tft, 0x9225, HEIGHT, WIDTH, 0, 0);
	lcd_set_font_direction(tft, DIRECTION90);
	lcd_fill_screen(tft, BLACK);

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
		.freq = 50250, /* ~1kHz */
		.offset = 0,
	};

	ESP_ERROR_CHECK(dac_cw_generator_config(&cw_config));
	ESP_ERROR_CHECK(dac_cw_generator_enable());

	ESP_LOGI(tag, "Start input processing task...");
	xTaskCreatePinnedToCore(input_loop, "input", 4096, NULL, 2, NULL, 0);

	ESP_LOGI(tag, "Start the display loop...");
	xTaskCreatePinnedToCore(display_loop, "display", 4096, NULL, 1, NULL, 0);

	ESP_LOGI(tag, "Start the oscilloscope...");
	volts = volts_array[0];
	xTaskCreatePinnedToCore(oscilloscope_loop, "oscilloscope", 4096, NULL, 1, NULL, 1);

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(10000));
	}
}
