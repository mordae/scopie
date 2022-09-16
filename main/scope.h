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
#include <stdint.h>
#include <stdlib.h>


/*
 * Ways the oscilloscope can stabilize the image.
 */
enum scope_trigger {
	/*
	 * No stabilization, let the curves flow freely.
	 */
	SCOPE_TRIGGER_NONE = 0,

	/*
	 * Look for a rising edge within the trigger zone.
	 * Requires at least 3 samples in the trigger_zone.
	 */
	SCOPE_TRIGGER_RISING,

	/*
	 * Look for a falling edge within the trigger zone.
	 *
	 * Same as SCOPE_TRIGGER_RISING, but for falling edges.
	 */
	SCOPE_TRIGGER_FALLING,

	/* Marks the last enum value, don't use. */
	SCOPE_TRIGGER_MAX,
};


/*
 * Oscilloscope configuration.
 */
struct scope_config {
	/* ADC options. */
	unsigned freq_hz;
	uint8_t atten;
	uint8_t channel;
	uint8_t unit;
	uint8_t bit_width;

	/* Way to stabilize the image. See scope_trigger above. */
	int trigger;

	/* Number of samples to return between 64 and 2048. */
	unsigned window_size;

	/* Voltage multiplier to compensate for an external voltage
	 * divider or something. */
	unsigned multiplier;
};


/*
 * How long have been spent on waiting for ADC on average.
 * It is rather important for this to be slightly above 0,
 * so please monitor it.
 */
extern float scope_adc_ticks;

/*
 * How long have we spent doing the trigger math on average.
 * This utilizes the CPU, so you want to make sure it stays low.
 * Namely compared to scope_adc_ticks.
 */
extern float scope_math_ticks;


/*
 * Start the background data acquisition task.
 */
void scope_init(int pri, int core);


/*
 * Change data acquisition settings.
 */
void scope_config(struct scope_config *cfg);


/*
 * Read a window starting with a trigger.
 * Trigger pattern is part of the result window.
 *
 * You must free() the window.
 *
 * If average is non-NULL, long-term average voltage will be written to it.
 */
uint16_t *scope_read(float *average, TickType_t wait);
