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

#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_vfs.h"

#include <string.h>


static const char *tag = "main";

static fontx_file_t font[1];
static tft_t tft[1];


#define WIDTH	220
#define HEIGHT	176


void test(void)
{
	lcd_draw_string(tft, font, 100, 100, "Test", WHITE);
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

	test();

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(10000));
	}
}
