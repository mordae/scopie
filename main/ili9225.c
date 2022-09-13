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
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>


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

	0b1111110001010001, /* light red */
	0b1000111111110001, /* light green */
	0b1000110001011111, /* light blue */

	0b1111111111100000, /* yellow */
	0b0000011111111111, /* cyan */
	0b1111100000011111, /* purple */

	0b1000110001000000, /* dark yellow */
	0b0000010001010001, /* dark cyan */
	0b1000100000010001, /* dark purple */
};

/*
 * We are using triple buffering.
 *
 * One buffer is being written to by the client.
 * Previous buffer is being transmitted.
 * Last buffer is a reference for change detection.
 *
 * After every cycle the buffers are rotated.
 */
static uint8_t buffer[3][LCD_Y_MAX][LCD_X_MAX / 2];
static uint8_t (*reference)[LCD_Y_MAX][LCD_X_MAX / 2];
static uint8_t (*committed)[LCD_Y_MAX][LCD_X_MAX / 2];
static unsigned lcd_input_no = 0;

uint8_t (*lcd_input)[LCD_Y_MAX][LCD_X_MAX / 2];


/*
 * Static buffers for SPI transactions.
 */
#define TXBUF_COUNT 4

static spi_transaction_t *next_txbuf()
{
	static uint8_t txbufs[TXBUF_COUNT][sizeof(spi_transaction_t) + LCD_X_MAX * 2];
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


void lcd_init(spi_host_device_t host, int rs, int cs, int rst)
{
	ESP_LOGI(tag, "Configure SPI device (host=%i, cs=%i)...", host, cs);
	spi_device_interface_config_t config = {
		.clock_speed_hz = SPI_MASTER_FREQ_40M,
		.spics_io_num = cs,
		.queue_size = TXBUF_COUNT - 1,
		.pre_cb = pre_cb,
		.flags = SPI_DEVICE_NO_DUMMY,
	};
	ESP_ERROR_CHECK(spi_bus_add_device(host, &config, &dev));

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
	lcd_input_no = 0;
	lcd_input = &buffer[lcd_input_no];
	committed = &buffer[(lcd_input_no - 1) % 3];
	reference = &buffer[(lcd_input_no - 2) % 3];

	ESP_LOGI(tag, "Begin preflight...");
	preflight();

	lcd_fill(1);
	lcd_sync();

	lcd_fill(0);
	lcd_sync();
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
	lcd_input_no = (lcd_input_no + 1) % 3;

	lcd_input = &buffer[lcd_input_no];
	committed = &buffer[(lcd_input_no + 2) % 3];
	reference = &buffer[(lcd_input_no + 1) % 3];

	memcpy(*lcd_input, *committed, sizeof(*lcd_input));

	/* Home the GRAM Address Counter. */
	set_register(0x20, 0);
	set_register(0x21, 0);

	for (int y = 0; y < LCD_Y_MAX; y++) {
		uint8_t txbuf[LCD_X_MAX * 2];

		for (int x = 0; x < LCD_X_MAX / 2; x++) {
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
		write_buffer(txbuf, LCD_X_MAX * 2);
	}

#if 0
	for (int y = 0; y < LCD_Y_MAX; y++) {
		for (int x = 0; x < LCD_X_MAX / 2; x++) {
			uint8_t twopix0 = (*reference)[y][x];
			uint8_t twopix1 = (*committed)[y][x];

			if (twopix0 != twopix1) {
				set_register(0x20, 2 * x);
				set_register(0x21, y);
				write_register(0x22);

				uint8_t buf[4] = {
					lcd_palette[high(twopix1)] >> 8,
					lcd_palette[high(twopix1)],
					lcd_palette[low(twopix1)] >> 8,
					lcd_palette[low(twopix1)],
				};
				write_buffer(buf, 4);
			}

#if 0
			if (high(twopix0) != high(twopix1)) {
				set_register(0x20, 2 * x + 0);
				set_register(0x21, y);
				set_register(0x22, lcd_palette[high(twopix1)]);
			}

			if (low(twopix0) != low(twopix1)) {
				set_register(0x20, 2 * x + 1);
				set_register(0x21, y);
				set_register(0x22, lcd_palette[low(twopix1)]);
			}
#endif
		}
	}
#endif
}
