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

#define FONTX_GLYPH_BUF_SIZE (32 * 32 / 8)

typedef struct {
	const char *path;
	char fxname[10];
	bool opened;
	bool valid;
	bool is_ank;
	uint8_t w;
	uint8_t h;
	uint16_t fsz;
	uint8_t bc;
	FILE *file;
} fontx_file_t;

void fontx_add(fontx_file_t *fx, const char *path);
void fontx_init(fontx_file_t *fxs, const char *f0, const char *f1);
bool fontx_open(fontx_file_t *fx);
void fontx_close(fontx_file_t *fx);
void fontx_dump(fontx_file_t *fxs);
uint8_t fontx_get_width(fontx_file_t *fx);
uint8_t fontx_get_height(fontx_file_t *fx);
bool fontx_get(fontx_file_t *fxs, char ascii, uint8_t *pGlyph, uint8_t *pw,
	       uint8_t *ph);
void fontx_to_bitmap(uint8_t *fonts, uint8_t *line, uint8_t w, uint8_t h,
		 uint8_t inverse);
void fontx_underline(uint8_t *line, uint8_t w, uint8_t h);
void fontx_reverse(uint8_t *line, uint8_t w, uint8_t h);
void fontx_show_font(uint8_t *fonts, uint8_t pw, uint8_t ph);
void fontx_show_bitmap(uint8_t *bitmap, uint8_t pw, uint8_t ph);
uint8_t fontx_rotate_byte(uint8_t ch);
