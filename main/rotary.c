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

#include "rotary.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <driver/gpio.h>
#include <esp_log.h>
#include <stdint.h>


static const char *tag = "rotary";


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

static const uint8_t state_table[6][4] = {
	{R_START_M,           R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
	{R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START},
	{R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START},
	{R_START_M,           R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
	{R_START_M,           R_START_M,      R_CW_BEGIN_M, R_START},
	{R_START_M,           R_CCW_BEGIN_M,  R_START_M,    R_START},
};

struct encoder {
	int8_t sw, left, right;
	uint8_t state;
	bool pressed;
	int steps;
	uint8_t sens;
};


static unsigned next = 0;
static struct encoder encoders[ROTARY_MAX];


static StaticSemaphore_t input_semaphore;
static SemaphoreHandle_t input_signal;


static void intr_handler(void *arg);


inline static int clamp(int x, int min, int max)
{
	if (x < min)
		return min;

	if (x > max)
		return max;

	return x;
}


void rotary_wait(TickType_t ticks)
{
	xSemaphoreTake(input_signal, ticks);
}


int rotary_add(int sw, int left, int right, uint8_t sens)
{
	assert ((left >= 0) && (right >= 0));
	assert (sens > 0);

	if (NULL == input_signal)
		input_signal = xSemaphoreCreateBinaryStatic(&input_semaphore);

	if (next >= ROTARY_MAX)
		return -1;

	ESP_LOGI(tag, "Add RE%i: sw=%i, left=%i, right=%i, sens=%hhu",
	         next, sw, left, right, sens);

	encoders[next].sw = sw;
	encoders[next].left = left;
	encoders[next].right = right;
	encoders[next].steps = 0;
	encoders[next].sens = sens;
	encoders[next].state = R_START_M;
	encoders[next].pressed = false;

	gpio_config_t config = {
		.pin_bit_mask = (sw >= 0 ? BIT64(sw) : 0)
		              | BIT64(left)
			      | BIT64(right),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = 1,
		.intr_type = GPIO_INTR_ANYEDGE,
	};
	ESP_ERROR_CHECK(gpio_config(&config));

	if (sw >= 0)
		ESP_ERROR_CHECK(gpio_isr_handler_add(sw, intr_handler, encoders + next));

	ESP_ERROR_CHECK(gpio_isr_handler_add(left, intr_handler, encoders + next));
	ESP_ERROR_CHECK(gpio_isr_handler_add(right, intr_handler, encoders + next));

	return next++;
}


int rotary_read_steps(int handle)
{
	assert (handle >= 0 && handle < ROTARY_MAX);

	int steps = encoders[handle].steps;
	encoders[handle].steps = 0;
	return steps;
}


bool rotary_read_pressed(int handle)
{
	assert (handle >= 0 && handle < ROTARY_MAX);
	return encoders[handle].pressed;
}


static void intr_handler(void *arg)
{
	struct encoder *en = arg;

	if (en->sw >= 0)
		en->pressed = !gpio_get_level(en->sw);

	uint8_t left = !gpio_get_level(en->left);
	uint8_t right = !gpio_get_level(en->right);

	uint8_t pin_state = (left << 1) | right;
	en->state = state_table[en->state & 0xf][pin_state];

	if (en->state & (DIR_CW | DIR_CCW)) {
		static TickType_t prev = 0;
		TickType_t now = xTaskGetTickCountFromISR() / portTICK_PERIOD_MS;

		if (now <= prev) {
			prev = now - 1;
		}
		else if (now - prev > en->sens) {
			prev = now - en->sens;
		}

		int speed = en->sens / (now - prev);
		speed = speed * speed;
		prev = now;

		if (en->state & DIR_CW) {
			en->state &= 0xf;
			en->steps += speed;
		}
		else if (en->state & DIR_CCW) {
			en->state &= 0xf;
			en->steps -= speed;
		}
	}

	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(input_signal, &xHigherPriorityTaskWoken);
}
