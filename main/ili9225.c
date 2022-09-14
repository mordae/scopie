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

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>
#include <stdio.h>


static const char *tag = "ili9225";


enum lcd_orientation lcd_orientation = LCD_ROTATE_90;

uint16_t lcd_palette[16] = {
	0b0000000000000000, /* black */
	0b0001100001100011, /* dark gray */
	0b1000110001010001, /* light gray */
	0b1111111111111111, /* white */

	0b1111100000000000, /* red */
	0b0000011111100000, /* green */
	0b0000000000011111, /* blue */

	0b1111101001001001, /* light red */
	0b0100111111101001, /* light green */
	0b0100101001011111, /* light blue */

	0b1111111111100000, /* yellow */
	0b0000011111111111, /* cyan */
	0b1111100000011111, /* purple */

	0b1000110001000000, /* dark yellow */
	0b0000010001010001, /* dark cyan */
	0b1000100000010001, /* dark purple */
};

/*
 * We are using double buffering.
 *
 * One buffer is being written to by the client.
 * The other buffer is being transmitted.
 *
 * After every cycle the buffers are rotated.
 */
static uint8_t buffer[2][LCD_HEIGHT][LCD_WIDTH / 2];
static uint8_t (*committed)[LCD_HEIGHT][LCD_WIDTH / 2];

uint8_t (*lcd_input)[LCD_HEIGHT][LCD_WIDTH / 2];


/* Font is initially empty. */
uint8_t lcd_font[256][16] = {0};


/*
 * Static buffers for SPI transactions. Must be at least 3,
 * since the queue is TXBUF_COUNT - 2 to prevent clobbering.
 * Using >4 does not translate to further speedup.
 */
#define TXBUF_COUNT 4

static spi_transaction_t *next_txbuf()
{
	static uint8_t txbufs[TXBUF_COUNT][sizeof(spi_transaction_t) + LCD_WIDTH * 2];
	static unsigned txbufno = 0;

	spi_transaction_t *tx = (void *)txbufs[txbufno];
	memset(tx, 0, sizeof(*tx));

	txbufno = (txbufno + 1) % TXBUF_COUNT;

	return tx;
}


/* Our SPI device handle. */
static spi_device_handle_t dev;


/* GPIO pin to select either register or data. */
static int register_select = -1;

static void select_register(void)
{
	gpio_set_level(register_select, 0);
}

static void select_data(void)
{
	gpio_set_level(register_select, 1);
}


/*
 * Change register ID.
 */
static void write_register(uint8_t reg)
{
	spi_transaction_t *tx = next_txbuf();

	tx->flags = SPI_TRANS_USE_TXDATA;
	tx->tx_data[0] = reg;
	tx->length = 8;
	tx->user = select_register;

	ESP_ERROR_CHECK(spi_device_queue_trans(dev, tx, portMAX_DELAY));
	//ESP_ERROR_CHECK(spi_device_transmit(dev, tx));
}


/*
 * Write a word to the current register.
 */
static void write_word(uint16_t word)
{
	spi_transaction_t *tx = next_txbuf();

	tx->flags = SPI_TRANS_USE_TXDATA;
	tx->tx_data[0] = word >> 8;
	tx->tx_data[1] = word;
	tx->length = 16;
	tx->user = select_data;

	ESP_ERROR_CHECK(spi_device_queue_trans(dev, tx, portMAX_DELAY));
	//ESP_ERROR_CHECK(spi_device_transmit(dev, tx));
}


/*
 * Write a whole long buffer.
 */
static void write_buffer(uint8_t *bstr, size_t len)
{
	spi_transaction_t *tx = next_txbuf();

	tx->tx_buffer = tx + 1;
	tx->length = 8 * len;
	tx->user = select_data;

	memcpy(tx + 1, bstr, len);

	ESP_ERROR_CHECK(spi_device_queue_trans(dev, tx, portMAX_DELAY));
	//ESP_ERROR_CHECK(spi_device_transmit(dev, tx));
}


/*
 * Shortcut to change the register and then write a word to it.
 * Useful for preflight().
 */
static void set_register(uint8_t reg, uint16_t word)
{
	write_register(reg);
	write_word(word);
}


/*
 * Prepare the display for operation.
 */
static void preflight()
{
	/* Reset power options. */
	set_register(0x10, 0x0000);	// Set SAP, DSTB, STB
	set_register(0x11, 0x0000);	// Set APON, PON, AON, VCI1EN, VC
	set_register(0x12, 0x0000);	// Set BT, DC1, DC2, DC3
	set_register(0x13, 0x0000);	// Set GVDD
	set_register(0x14, 0x0000);	// Set VCOMH/VCOML voltage
	vTaskDelay(pdMS_TO_TICKS(40));

	/* Set power options. */
	set_register(0x11, 0x0018);	// Set APON, PON, AON, VCI1EN, VC
	set_register(0x12, 0x6121);	// Set BT, DC1, DC2, DC3
	set_register(0x13, 0x006f);	// Set GVDD
	set_register(0x14, 0x495f);	// Set VCOMH/VCOML voltage
	set_register(0x10, 0x0800);	// Set SAP, DSTB, STB
	vTaskDelay(pdMS_TO_TICKS(10));

	set_register(0x11, 0x103b);	// Set APON, PON, AON, VCI1EN, VC
	vTaskDelay(pdMS_TO_TICKS(50));

	/* Configure most of the display. */
	set_register(0x01, 0x011c);     // 220 lines, SS=1
	set_register(0x02, 0x0100);	// INV0=1 (3 field interlace)
	set_register(0x03, 0x1030);	// BGR=1, AC=11
	set_register(0x07, 0x0000);	// Display off
	set_register(0x08, 0x0808);	// Back and front porch
	set_register(0x0b, 0x1100);	// NO=1, STD=1, RTN=0 (16 clocks per line)
	set_register(0x0c, 0x0000);	// RIM=0 (18b), DM=0, RM=0
	set_register(0x0f, 0x8001);	// FOSC=1000 (285.7KHz), OSC_EN=1
	set_register(0x15, 0x0020);	// VCIR=010 (2 clocks)
	set_register(0x20, 0x0000);	// AD_LO=0 (position GRAM AC=0)
	set_register(0x21, 0x0000);	// AD_HI=0

	/* Scrolling and update area setup. */
	set_register(0x30, 0x0000);
	set_register(0x31, 0x00db);
	set_register(0x32, 0x0000);
	set_register(0x33, 0x0000);
	set_register(0x34, 0x00db);
	set_register(0x35, 0x0000);
	set_register(0x36, 0x00af);
	set_register(0x37, 0x0000);
	set_register(0x38, 0x00db);
	set_register(0x39, 0x0000);

	/* Adjust gamma curve. */
	set_register(0x50, 0x0000);
	set_register(0x51, 0x0808);
	set_register(0x52, 0x080a);
	set_register(0x53, 0x000a);
	set_register(0x54, 0x0a08);
	set_register(0x55, 0x0808);
	set_register(0x56, 0x0000);
	set_register(0x57, 0x0a00);
	set_register(0x58, 0x0710);
	set_register(0x59, 0x0710);

	set_register(0x07, 0x0012);
	vTaskDelay(pdMS_TO_TICKS(50));

	set_register(0x07, 0x1017);     // TEMON=1, GON=1, CL=1, REV=1, D=11
}


static void pre_cb(spi_transaction_t *tx)
{
	void (*user_fn)(void) = tx->user;
	if (user_fn)
		user_fn();
}


void lcd_init(int rs, int rst)
{
	ESP_LOGI(tag, "Configure SPI device (SPI3, cs=5)...");
	spi_device_interface_config_t config = {
		.clock_speed_hz = SPI_MASTER_FREQ_40M,
		.spics_io_num = 5,
		.queue_size = TXBUF_COUNT - 2,
		.pre_cb = pre_cb,
		.flags = SPI_DEVICE_NO_DUMMY,
	};
	ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &config, &dev));

	if (rst >= 0) {
		gpio_reset_pin(rst);
		gpio_set_direction(rst, GPIO_MODE_OUTPUT);

		gpio_set_level(rst, 0);
		vTaskDelay(pdMS_TO_TICKS(100));

		gpio_set_level(rst, 1);
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	register_select = rs;
	gpio_reset_pin(register_select);
	gpio_set_direction(register_select, GPIO_MODE_OUTPUT);

	ESP_LOGI(tag, "Arrange buffers...");
	lcd_input = &buffer[0];
	committed = &buffer[1];

	ESP_LOGI(tag, "Begin preflight...");
	preflight();

	lcd_fill(1);
	lcd_sync();

	lcd_fill(0);
	lcd_sync();
}


int lcd_load_font(const char *path)
{
	FILE *fp = fopen(path, "rb");

	if (NULL == fp)
		goto fail;

	if (256 != fread(lcd_font, 16, 256, fp))
		goto fail;

	fclose(fp);
	return 0;

fail:
	return -1;
}


inline static uint8_t high(uint8_t x)
{
	return (x >> 4) & 0b1111;
}

inline static uint8_t low(uint8_t x)
{
	return x & 0b1111;
}


void lcd_sync(void)
{
	static uint8_t (*tmp)[LCD_HEIGHT][LCD_WIDTH / 2];

	tmp       = committed;
	committed = lcd_input;
	lcd_input = tmp;

	memcpy(*lcd_input, *committed, sizeof(*lcd_input));

	/* Home the GRAM Address Counter. */
	set_register(0x20, 0);
	set_register(0x21, 0);

	for (int y = 0; y < LCD_HEIGHT; y++) {
		uint8_t txbuf[LCD_WIDTH * 2];

		for (int x = 0; x < LCD_WIDTH / 2; x++) {
			uint8_t twopix = (*committed)[y][x];

			uint16_t left  = lcd_palette[(twopix >> 4) & 0b1111];
			uint16_t right = lcd_palette[twopix & 0b1111];

			txbuf[4 * x + 0] = left >> 8;
			txbuf[4 * x + 1] = left;
			txbuf[4 * x + 2] = right >> 8;
			txbuf[4 * x + 3] = right;
		}

		/* Activate GRAM write register. */
		write_register(0x22);

		/* Send the line in buffer. */
		write_buffer(txbuf, LCD_WIDTH * 2);
	}
}


struct lcd_point lcd_point_to_phys(uint8_t x, uint8_t y)
{
	struct lcd_point phys = {x, y};

	switch (lcd_orientation) {
		case LCD_ROTATE_0:
			break;

		case LCD_ROTATE_90:
			phys.x = y;
			phys.y = x;
			break;

		case LCD_ROTATE_180:
			phys.x = LCD_WIDTH - x;
			phys.y = LCD_HEIGHT - y;
			break;

		case LCD_ROTATE_270:
			phys.x = LCD_WIDTH - y;
			phys.y = LCD_HEIGHT - x;
			break;

		case LCD_MIRROR_X:
			phys.x = LCD_WIDTH - x;
			phys.y = y;
			break;

		case LCD_MIRROR_Y:
			phys.x = x;
			phys.y = LCD_HEIGHT - y;
			break;
	}

	assert (phys.x >= 0);
	assert (phys.x < LCD_WIDTH);
	assert (phys.y >= 0);
	assert (phys.y < LCD_HEIGHT);

	return phys;
}


struct lcd_rect lcd_rect_to_phys(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
	struct lcd_point phys0 = lcd_point_to_phys(x0, y0);
	struct lcd_point phys1 = lcd_point_to_phys(x1, y1);
	struct lcd_rect phys = {
		.x0 = phys0.x,
		.y0 = phys0.y,
		.x1 = phys1.x,
		.y1 = phys1.y,
	};
	return phys;
}


void lcd_draw_pixel(uint8_t x, uint8_t y, uint8_t color)
{
	struct lcd_point phys = lcd_point_to_phys(x, y);
	uint8_t twopix = (*lcd_input)[phys.y][phys.x / 2];

	if (phys.x & 1)
		twopix = (twopix & 0b11110000) | ((color & 0b1111) << 0);
	else
		twopix = (twopix & 0b00001111) | ((color & 0b1111) << 4);

	(*lcd_input)[phys.y][phys.x / 2] = twopix;
}


void lcd_draw_rect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color)
{
	struct lcd_rect phys = lcd_rect_to_phys(x0, y0, x1, y1);

	if (phys.x0 > phys.x1) {
		phys.x0 ^= phys.x1;
		phys.x1 ^= phys.x0;
		phys.x0 ^= phys.x1;
	}

	if (phys.y0 > phys.y1) {
		phys.y0 ^= phys.y1;
		phys.y1 ^= phys.y0;
		phys.y0 ^= phys.y1;
	}

	for (int y = phys.y0; y <= phys.y1; y++) {
		for (int x = phys.x0; x <= phys.x1; x++) {
			uint8_t twopix = (*lcd_input)[y][x / 2];

			if (x & 1)
				twopix = (twopix & 0b11110000) | ((color & 0b1111) << 0);
			else
				twopix = (twopix & 0b00001111) | ((color & 0b1111) << 4);

			(*lcd_input)[y][x / 2] = twopix;
		}
	}
}


void lcd_fill(uint8_t color)
{
	uint8_t twopix = ((color & 0b1111) << 4) | (color & 0b1111);
	memset(*lcd_input, twopix, sizeof(*lcd_input));
}


void lcd_draw_glyph(uint8_t x, uint8_t y, uint8_t color, char c)
{
	uint8_t *glyph = lcd_font[(uint8_t)c];

	for (int gx = 0; gx < 8; gx++) {
		for (int gy = 0; gy < 16; gy++) {
			if ((glyph[15 - gy] >> (7 - gx)) & 1) {
				lcd_draw_pixel(x + gx, y + gy, color);
			}
		}
	}
}


void lcd_draw_string(uint8_t x, uint8_t y, uint8_t color, const char *str)
{
	int len = strlen(str);

	for (int i = 0; i < len; i++)
		lcd_draw_glyph(x + i * 8, y, color, str[i]);
}
