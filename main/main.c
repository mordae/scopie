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


static const char *tag = "main";

static fontx_file_t font[1];
static tft_t tft[1];


#define WIDTH	220
#define HEIGHT	176

#define ADC_FRAME WIDTH
#define ADC_BUFFER (ADC_FRAME * SOC_ADC_DIGI_DATA_BYTES_PER_CONV)

static adc_continuous_handle_t cadc;
static adc_cali_handle_t cali;
static uint8_t frame[ADC_BUFFER];
static int volts[ADC_FRAME];

static void cadc_start(int freq_hz)
{
	static int prev_hz = 0;

	if (prev_hz == freq_hz)
		goto end;

	if (prev_hz)
		ESP_ERROR_CHECK(adc_continuous_deinit(cadc));

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

	prev_hz = freq_hz;

end:
	ESP_ERROR_CHECK(adc_continuous_start(cadc));
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

	ESP_LOGI(tag, "Preparing ADC calibration...");
	adc_cali_line_fitting_config_t cali_config = {
		.unit_id = ADC_UNIT_1,
		.atten = ADC_ATTEN_DB_11,
		.bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
	};
	ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali));

	ESP_LOGI(tag, "Starting cosine wave generator...");
	dac_output_enable(DAC_GPIO26_CHANNEL);

	dac_cw_config_t cw_config = {
		.en_ch = DAC_GPIO26_CHANNEL,
		.scale = DAC_CW_SCALE_2,
		.phase = DAC_CW_PHASE_180,
		.freq = 10 * 1000,
		.offset = 0,
	};

	ESP_ERROR_CHECK(dac_cw_generator_config(&cw_config));
	ESP_ERROR_CHECK(dac_cw_generator_enable());

	while (1) {
		uint32_t size = 0;

		cadc_start(100000);
		ESP_ERROR_CHECK(adc_continuous_read(cadc, frame, ADC_BUFFER, &size, 30));
		ESP_ERROR_CHECK(adc_continuous_stop(cadc));

		if (size < ADC_BUFFER) {
			ESP_LOGE(tag, "ADC did not return a full frame, just %lu bytes.", size);
			continue;
		}

		uint32_t total = 0;
		int *voltsptr = volts;

		for (int i = 0; i < ADC_BUFFER; i += SOC_ADC_DIGI_DATA_BYTES_PER_CONV) {
			adc_digi_output_data_t *p = (void *)(frame + i);
			ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali, p->type1.data, voltsptr));
			total += *voltsptr++;
		}

		int average = total / ADC_FRAME;

		lcd_fill_screen(tft, BLACK);
		lcd_draw_line(tft, HEIGHT * average / 3300, 0, HEIGHT * average / 3300, WIDTH - 1, CYAN);

		for (int i = 0; i < WIDTH; i++)
			lcd_draw_pixel(tft, HEIGHT * volts[i] / 3300, i, WHITE);

		ESP_LOGI(tag, "average: %-4d", average);
	}

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(10000));
	}
}
