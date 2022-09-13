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

#pragma once
#include <driver/spi_master.h>
#include <stdint.h>
#include <string.h>


/*
 * Dimensions of this particular screen in hardware coordinates.
 */
#define LCD_WIDTH 176
#define LCD_HEIGHT 220


/*
 * Ways the screen can be oriented.
 *
 * We can compensate when drawing and properly transform the virtual xy
 * coordinates to the real coordinates so that users don't have to.
 */
enum lcd_orientation {
	LCD_ROTATE_0 = 0,
	LCD_ROTATE_90,
	LCD_ROTATE_180,
	LCD_ROTATE_270,
	LCD_MIRROR_X,
	LCD_MIRROR_Y,
};

/*
 * Defaults to LCD_ROTATE_90, because everyone likes it wider.
 */
extern enum lcd_orientation lcd_orientation;


/*
 * Color palette. We save only 4 bits per pixel instead of full 16.
 *
 * Default colors are:
 *
 *   0 - black
 *   1 - dark gray
 *   2 - light gray
 *   3 - white,
 *   4 - red
 *   5 - green
 *   6 - blue
 *   7 - light red
 *   8 - light green
 *   9 - light blue
 *  10 - yellow
 *  11 - cyan
 *  12 - purple
 *  13 - dark yellow
 *  14 - dark cyan
 *  15 - dark purple
 *
 * You should define some aliases and perhaps change the colors to
 * your liking. You can use lcd_rgb() to generate the colors.
 */
extern uint16_t lcd_palette[16];

/*
 * Generate 16bit color from the customary 3x 8bit RGB.
 */
inline static uint16_t lcd_rgb(uint8_t r, uint8_t g, uint8_t b)
{
	r = (r >> 3) & 0b00011111;
	g = (g >> 2) & 0b00111111;
	b = (b >> 3) & 0b00011111;
	return (r << 11) | (g << 5) | b;
}


/*
 * Modified latin 16x8 bitmap font.
 * Populate it with lcd_load_font().
 */
extern uint8_t lcd_font[256][16];


/*
 * Initialize the screen.
 *
 * The SPI master parameter is mandatory. You must use spi_bus_initialize()
 * to configure the SPI master before you call this function.
 *
 * You must also provide the Register Select GPIO pin number.
 *
 * You can give a CS pin number or leave it at -1.
 * You can give a RST pin number or leave it at -1.
 */
void lcd_init(spi_host_device_t host, int rs, int cs, int rst);


/*
 * Load the font into memory.
 * Expects path to a file with 256x16 bytes.
 * Every glyph has 16 rows and 8 columns.
 */
int lcd_load_font(const char *path);


/*
 * Start display synchronization cycle.
 *
 * Blocks until the last cycle finishes so the you do not clobber
 * the input buffer it reads from.
 *
 * Changes lcd_input to a new buffer and copies the last one to it.
 */
void lcd_sync(void);


/*
 * Here goes whatever should be on the screen.
 * First come the Y rows, then the X columns.
 */
extern uint8_t (*lcd_input)[LCD_HEIGHT][LCD_WIDTH / 2];


/* Coordinates for a point. */
struct lcd_point {
	uint8_t x, y;
};

/* Coordinates for a rectangle. */
struct lcd_rect {
	uint8_t x0, y0;
	uint8_t x1, y1;
};


/*
 * Transfer coordinates from virtual space to the physical space.
 */
struct lcd_point lcd_point_to_phys(uint8_t x, uint8_t y);


/*
 * Transfer coordinates from virtual space to the physical space,
 * but for a whole rectangle.
 */
struct lcd_rect lcd_rect_to_phys(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);


/*
 * Color a single pixel.
 */
void lcd_draw_pixel(uint8_t x, uint8_t y, uint8_t color);


/*
 * Color a whole rect of pixels.
 */
void lcd_draw_rect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);


/*
 * Paint the whole screen with a single color.
 */
void lcd_fill(uint8_t color);


/*
 * Draw given glyph at specified coordinates.
 * The coordinates indicate bottom left of the glyph.
 */
void lcd_draw_glyph(uint8_t x, uint8_t y, uint8_t color, char c);


/*
 * Draw given string at specified coordinates.
 * The coordinates indicate bottom left of the string.
 */
void lcd_draw_string(uint8_t x, uint8_t y, uint8_t color, const char *str);
