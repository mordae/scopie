/*
 * Copyright (C) 2020 nopnop2002
 * Copyright (C) 2022 Jan Hamal Dvořák <mordae@anilinux.org>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include "driver/spi_master.h"
#include "fontx.h"

#define RED			0xf800
#define GREEN			0x07e0
#define BLUE			0x001f
#define BLACK			0x0000
#define WHITE			0xffff
#define GRAY			0x8c51
#define YELLOW			0xFFE0
#define CYAN			0x07FF
#define PURPLE			0xF81F

#define DIRECTION0		0
#define DIRECTION90		1
#define DIRECTION180		2
#define DIRECTION270		3

typedef struct {
	uint16_t _model;
	uint16_t _width;
	uint16_t _height;
	uint16_t _offsetx;
	uint16_t _offsety;
	uint16_t _font_direction;
	uint16_t _font_fill;
	uint16_t _font_fill_color;
	uint16_t _font_underline;
	uint16_t _font_underline_color;
	int16_t _dc;
	int16_t _bl;
	int16_t _irq;
	spi_device_handle_t _TFT_Handle;
	spi_device_handle_t _XPT_Handle;
	bool _calibration;
	int16_t _min_xp;	// Minimum xp calibration
	int16_t _min_yp;	// Minimum yp calibration
	int16_t _max_xp;	// Maximum xp calibration
	int16_t _max_yp;	// Maximum yp calibration
	int16_t _min_xc;	// Minimum x coordinate
	int16_t _min_yc;	// Minimum y coordinate
	int16_t _max_xc;	// Maximum x coordinate
	int16_t _max_yc;	// Maximum y coordinate
} tft_t;

void spi_master_init(tft_t *dev, int16_t GPIO_MOSI, int16_t GPIO_SCLK,
		     int16_t TFT_CS, int16_t GPIO_DC, int16_t GPIO_RESET,
		     int16_t GPIO_MISO, int16_t XPT_CS, int16_t XPT_IRQ);

bool spi_master_write_byte(spi_device_handle_t SPIHandle, const uint8_t *Data,
			   size_t DataLength);

bool spi_master_write_comm_byte(tft_t *dev, uint8_t cmd);

bool spi_master_write_comm_word(tft_t *dev, uint16_t cmd);

bool spi_master_write_data_byte(tft_t *dev, uint8_t data);

bool spi_master_write_data_word(tft_t *dev, uint16_t data);

bool spi_master_write_addr(tft_t *dev, uint16_t addr1, uint16_t addr2);

bool spi_master_write_color(tft_t *dev, uint16_t color, uint16_t size);

bool spi_master_write_colors(tft_t *dev, uint16_t *colors, uint16_t size);

void delayMS(int ms);

void lcd_write_register_word(tft_t *dev, uint16_t addr, uint16_t data);

void lcd_write_register_byte(tft_t *dev, uint8_t addr, uint16_t data);

void lcd_init(tft_t *dev, uint16_t model, int width, int height, int offsetx,
	     int offsety);

void lcd_draw_pixel(tft_t *dev, uint16_t x, uint16_t y, uint16_t color);

void lcd_draw_multi_pixels(tft_t *dev, uint16_t x, uint16_t y, uint16_t size,
			uint16_t *colors);

void lcd_draw_fill_rect(tft_t *dev, uint16_t x1, uint16_t y1, uint16_t x2,
		     uint16_t y2, uint16_t color);

void lcd_display_off(tft_t *dev);

void lcd_display_on(tft_t *dev);

void lcd_inversion_off(tft_t *dev);

void lcd_inversion_on(tft_t *dev);

void lcd_bgr_filter(tft_t *dev);

void lcd_fill_screen(tft_t *dev, uint16_t color);

void lcd_draw_line(tft_t *dev, uint16_t x1, uint16_t y1, uint16_t x2,
		 uint16_t y2, uint16_t color);

void lcd_draw_rect(tft_t *dev, uint16_t x1, uint16_t y1, uint16_t x2,
		 uint16_t y2, uint16_t color);

void lcd_draw_rect_angle(tft_t *dev, uint16_t xc, uint16_t yc, uint16_t w,
		      uint16_t h, uint16_t angle, uint16_t color);

void lcd_draw_triangle(tft_t *dev, uint16_t xc, uint16_t yc, uint16_t w,
		     uint16_t h, uint16_t angle, uint16_t color);

void lcd_draw_circle(tft_t *dev, uint16_t x0, uint16_t y0, uint16_t r,
		   uint16_t color);

void lcd_draw_fill_circle(tft_t *dev, uint16_t x0, uint16_t y0, uint16_t r,
		       uint16_t color);

void lcd_draw_round_rect(tft_t *dev, uint16_t x1, uint16_t y1, uint16_t x2,
		      uint16_t y2, uint16_t r, uint16_t color);

void lcd_draw_arrow(tft_t *dev, uint16_t x0, uint16_t y0, uint16_t x1,
		  uint16_t y1, uint16_t w, uint16_t color);

void lcd_draw_fill_arrow(tft_t *dev, uint16_t x0, uint16_t y0, uint16_t x1,
		      uint16_t y1, uint16_t w, uint16_t color);

uint16_t rgb565_conv(uint16_t r, uint16_t g, uint16_t b);

int lcd_draw_char(tft_t *dev, fontx_file_t *fx, uint16_t x, uint16_t y,
		  char ascii, uint16_t color);

int lcd_draw_string(tft_t *dev, fontx_file_t *fx, uint16_t x, uint16_t y,
		    const char *ascii, uint16_t color);

int lcd_draw_code(tft_t *dev, fontx_file_t *fx, uint16_t x, uint16_t y,
		  char code, uint16_t color);

void lcd_set_font_direction(tft_t *dev, uint16_t);

void lcd_set_font_fill(tft_t *dev, uint16_t color);

void lcd_unset_font_fill(tft_t *dev);

void lcd_set_font_underline(tft_t *dev, uint16_t color);

void lcd_unset_font_underline(tft_t *dev);

void lcd_set_scroll_area(tft_t *dev, uint16_t tfa, uint16_t vsa, uint16_t bfa);

void lcd_reset_scroll_area(tft_t *dev, uint16_t vsa);

void lcd_scroll(tft_t *dev, uint16_t vsp);

int xptGetit(tft_t *dev, int cmd);

void xptGetxy(tft_t *dev, int *xp, int *yp);
