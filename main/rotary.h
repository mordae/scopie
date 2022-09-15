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
#include <freertos/FreeRTOS.h>
#include <stdbool.h>
#include <stdint.h>


/* Maximum encoder count. */
#define ROTARY_MAX 4


/*
 * Add a rotary encoder using given GPIO pins.
 * Returns a handle or -1 on error.
 *
 * The sensitivity is used to multiply the steps taken in fast
 * succession. Set to 1 to count the steps exactly. Use 100 to
 * produce about 1000 steps per a rapid turn.
 *
 * Please note that the encoder should connect to ground,
 * since we are pulling the input pins up.
 *
 * You must call gpio_install_isr_service() before,
 * so that we can register per-pin handlers.
 */
int rotary_add(int sw, int left, int right, uint8_t sens);


/* Wait for any of the rotary encoders to change. */
void rotary_wait(TickType_t ticks);


/*
 * Read from the rotary encoder.
 * Returns number of steps (sensitivity adjusted) taken since the last read.
 * Positive number indicates clockwise direction, negative count-clockwise.
 */
int rotary_read_steps(int handle);


/*
 * Determine whether is the rotary encoder currently pressed.
 */
bool rotary_read_pressed(int handle);
