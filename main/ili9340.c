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

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "esp_log.h"

#include "ili9340.h"

#define TAG "ILI9340"

#ifdef CONFIG_IDF_TARGET_ESP32
#define LCD_HOST HSPI_HOST
#elif defined CONFIG_IDF_TARGET_ESP32S2
#define LCD_HOST SPI2_HOST
#elif defined CONFIG_IDF_TARGET_ESP32S3
#define LCD_HOST SPI2_HOST
#elif defined CONFIG_IDF_TARGET_ESP32C3
#define LCD_HOST SPI2_HOST
#endif

static const int SPI_Command_Mode = 0;
static const int SPI_Data_Mode = 1;
static const int TFT_Frequency = SPI_MASTER_FREQ_40M;

void spi_master_init(tft_t *dev, int16_t GPIO_MOSI, int16_t GPIO_SCLK,
		     int16_t TFT_CS, int16_t GPIO_DC, int16_t GPIO_RESET,
		     int16_t GPIO_MISO, int16_t XPT_CS, int16_t XPT_IRQ)
{
	esp_err_t ret;

	gpio_reset_pin(TFT_CS);
	gpio_set_direction(TFT_CS, GPIO_MODE_OUTPUT);
	gpio_set_level(TFT_CS, 1);

	gpio_reset_pin(GPIO_DC);
	gpio_set_direction(GPIO_DC, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_DC, 0);

	if (GPIO_RESET >= 0) {
		gpio_reset_pin(GPIO_RESET);
		gpio_set_direction(GPIO_RESET, GPIO_MODE_OUTPUT);
		gpio_set_level(GPIO_RESET, 0);
		vTaskDelay(pdMS_TO_TICKS(100));
		gpio_set_level(GPIO_RESET, 1);
	}

	spi_bus_config_t buscfg = {
		.sclk_io_num = GPIO_SCLK,
		.mosi_io_num = GPIO_MOSI,
		.miso_io_num = -1,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};

	ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
	ESP_LOGD(TAG, "spi_bus_initialize=%d", ret);
	assert(ret == ESP_OK);

	spi_device_interface_config_t tft_devcfg = {
		.clock_speed_hz = TFT_Frequency,
		.spics_io_num = TFT_CS,
		.queue_size = 7,
		.flags = SPI_DEVICE_NO_DUMMY,
	};

	spi_device_handle_t tft_handle;
	ret = spi_bus_add_device(LCD_HOST, &tft_devcfg, &tft_handle);
	ESP_LOGD(TAG, "spi_bus_add_device=%d", ret);
	assert(ret == ESP_OK);
	dev->_dc = GPIO_DC;
	dev->_TFT_Handle = tft_handle;
}

bool spi_master_write_byte(spi_device_handle_t SPIHandle, const uint8_t *Data,
			   size_t DataLength)
{
	spi_transaction_t SPITransaction;
	esp_err_t ret;

	if (DataLength > 0) {
		memset(&SPITransaction, 0, sizeof(spi_transaction_t));
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = Data;
		ret = spi_device_transmit(SPIHandle, &SPITransaction);
		assert(ret == ESP_OK);
	}

	return true;
}

bool spi_master_write_comm_byte(tft_t *dev, uint8_t cmd)
{
	static uint8_t Byte = 0;
	Byte = cmd;
	gpio_set_level(dev->_dc, SPI_Command_Mode);
	return spi_master_write_byte(dev->_TFT_Handle, &Byte, 1);
}

bool spi_master_write_comm_word(tft_t *dev, uint16_t cmd)
{
	static uint8_t Byte[2];
	Byte[0] = (cmd >> 8) & 0xFF;
	Byte[1] = cmd & 0xFF;
	gpio_set_level(dev->_dc, SPI_Command_Mode);
	return spi_master_write_byte(dev->_TFT_Handle, Byte, 2);
}

bool spi_master_write_data_byte(tft_t *dev, uint8_t data)
{
	static uint8_t Byte = 0;
	Byte = data;
	gpio_set_level(dev->_dc, SPI_Data_Mode);
	return spi_master_write_byte(dev->_TFT_Handle, &Byte, 1);
}

bool spi_master_write_data_word(tft_t *dev, uint16_t data)
{
	static uint8_t Byte[2];
	Byte[0] = (data >> 8) & 0xFF;
	Byte[1] = data & 0xFF;
	gpio_set_level(dev->_dc, SPI_Data_Mode);
	return spi_master_write_byte(dev->_TFT_Handle, Byte, 2);
}

bool spi_master_write_addr(tft_t *dev, uint16_t addr1, uint16_t addr2)
{
	static uint8_t Byte[4];
	Byte[0] = (addr1 >> 8) & 0xFF;
	Byte[1] = addr1 & 0xFF;
	Byte[2] = (addr2 >> 8) & 0xFF;
	Byte[3] = addr2 & 0xFF;
	gpio_set_level(dev->_dc, SPI_Data_Mode);
	return spi_master_write_byte(dev->_TFT_Handle, Byte, 4);
}

bool spi_master_write_color(tft_t *dev, uint16_t color, uint16_t size)
{
	static uint8_t Byte[1024];
	int index = 0;
	for (int i = 0; i < size; i++) {
		Byte[index++] = (color >> 8) & 0xFF;
		Byte[index++] = color & 0xFF;
	}
	gpio_set_level(dev->_dc, SPI_Data_Mode);
	return spi_master_write_byte(dev->_TFT_Handle, Byte, size * 2);
}

bool spi_master_write_colors(tft_t *dev, uint16_t *colors, uint16_t size)
{
	static uint8_t Byte[1024];
	int index = 0;
	for (int i = 0; i < size; i++) {
		Byte[index++] = (colors[i] >> 8) & 0xFF;
		Byte[index++] = colors[i] & 0xFF;
	}
	gpio_set_level(dev->_dc, SPI_Data_Mode);
	return spi_master_write_byte(dev->_TFT_Handle, Byte, size * 2);
}

void delayMS(int ms)
{
	int _ms = ms + (portTICK_PERIOD_MS - 1);
	TickType_t xTicksToDelay = _ms / portTICK_PERIOD_MS;
	ESP_LOGD(TAG, "ms=%d _ms=%d portTICK_PERIOD_MS=%lu xTicksToDelay=%lu",
		 ms, _ms, portTICK_PERIOD_MS, xTicksToDelay);
	vTaskDelay(xTicksToDelay);
}

void lcd_write_register_word(tft_t *dev, uint16_t addr, uint16_t data)
{
	spi_master_write_comm_word(dev, addr);
	spi_master_write_data_word(dev, data);
}

void lcd_write_register_byte(tft_t *dev, uint8_t addr, uint16_t data)
{
	spi_master_write_comm_byte(dev, addr);
	spi_master_write_data_word(dev, data);
}

void lcd_init(tft_t *dev, uint16_t model, int width, int height, int offsetx,
	      int offsety)
{
	dev->_model = model;
	dev->_width = width;
	dev->_height = height;
	dev->_offsetx = offsetx;
	dev->_offsety = offsety;
	dev->_font_direction = DIRECTION0;
	dev->_font_fill = false;
	dev->_font_underline = false;

	if (dev->_model == 0x7796) {
		ESP_LOGI(TAG, "Your TFT is ST7796");
		ESP_LOGI(TAG, "Screen width: %d", width);
		ESP_LOGI(TAG, "Screen height: %d", height);
		spi_master_write_comm_byte(dev, 0xC0);	//Power Control 1
		spi_master_write_data_byte(dev, 0x10);
		spi_master_write_data_byte(dev, 0x10);

		spi_master_write_comm_byte(dev, 0xC1);	//Power Control 2
		spi_master_write_data_byte(dev, 0x41);

		spi_master_write_comm_byte(dev, 0xC5);	//VCOM Control 1
		spi_master_write_data_byte(dev, 0x00);
		spi_master_write_data_byte(dev, 0x22);
		spi_master_write_data_byte(dev, 0x80);
		spi_master_write_data_byte(dev, 0x40);

		spi_master_write_comm_byte(dev, 0x36);	//Memory Access Control
		spi_master_write_data_byte(dev, 0x48);	//Right top start, BGR color filter panel
		//spi_master_write_data_byte(dev, 0x68);        //Right top start, BGR color filter panel

		spi_master_write_comm_byte(dev, 0xB0);	//Interface Mode Control
		spi_master_write_data_byte(dev, 0x00);

		spi_master_write_comm_byte(dev, 0xB1);	//Frame Rate Control
		spi_master_write_data_byte(dev, 0xB0);
		spi_master_write_data_byte(dev, 0x11);

		spi_master_write_comm_byte(dev, 0xB4);	//Display Inversion Control
		spi_master_write_data_byte(dev, 0x02);

		spi_master_write_comm_byte(dev, 0xB6);	//Display Function Control
		spi_master_write_data_byte(dev, 0x02);
		spi_master_write_data_byte(dev, 0x02);
		spi_master_write_data_byte(dev, 0x3B);

		spi_master_write_comm_byte(dev, 0xB7);	//Entry Mode Set
		spi_master_write_data_byte(dev, 0xC6);

		spi_master_write_comm_byte(dev, 0x3A);	//Interface Pixel Format
		spi_master_write_data_byte(dev, 0x55);

		spi_master_write_comm_byte(dev, 0xF7);	//Adjust Control 3
		spi_master_write_data_byte(dev, 0xA9);
		spi_master_write_data_byte(dev, 0x51);
		spi_master_write_data_byte(dev, 0x2C);
		spi_master_write_data_byte(dev, 0x82);

		spi_master_write_comm_byte(dev, 0x11);	//Sleep Out
		delayMS(120);

		spi_master_write_comm_byte(dev, 0x29);	//Display ON
	}			// endif 0x7796

	if (dev->_model == 0x9340 || dev->_model == 0x9341
	    || dev->_model == 0x7735) {
		if (dev->_model == 0x9340)
			ESP_LOGI(TAG, "Your TFT is ILI9340");
		if (dev->_model == 0x9341)
			ESP_LOGI(TAG, "Your TFT is ILI9341");
		if (dev->_model == 0x7735)
			ESP_LOGI(TAG, "Your TFT is ST7735");
		ESP_LOGI(TAG, "Screen width: %d", width);
		ESP_LOGI(TAG, "Screen height: %d", height);
		spi_master_write_comm_byte(dev, 0xC0);	//Power Control 1
		spi_master_write_data_byte(dev, 0x23);

		spi_master_write_comm_byte(dev, 0xC1);	//Power Control 2
		spi_master_write_data_byte(dev, 0x10);

		spi_master_write_comm_byte(dev, 0xC5);	//VCOM Control 1
		spi_master_write_data_byte(dev, 0x3E);
		spi_master_write_data_byte(dev, 0x28);

		spi_master_write_comm_byte(dev, 0xC7);	//VCOM Control 2
		spi_master_write_data_byte(dev, 0x86);

		spi_master_write_comm_byte(dev, 0x36);	//Memory Access Control
		spi_master_write_data_byte(dev, 0x08);	//Right top start, BGR color filter panel
		//spi_master_write_data_byte(dev, 0x00);//Right top start, RGB color filter panel

		spi_master_write_comm_byte(dev, 0x3A);	//Pixel Format Set
		spi_master_write_data_byte(dev, 0x55);	//65K color: 16-bit/pixel

		spi_master_write_comm_byte(dev, 0x20);	//Display Inversion OFF

		spi_master_write_comm_byte(dev, 0xB1);	//Frame Rate Control
		spi_master_write_data_byte(dev, 0x00);
		spi_master_write_data_byte(dev, 0x18);

		spi_master_write_comm_byte(dev, 0xB6);	//Display Function Control
		spi_master_write_data_byte(dev, 0x08);
		spi_master_write_data_byte(dev, 0xA2);	// REV:1 GS:0 SS:0 SM:0
		spi_master_write_data_byte(dev, 0x27);
		spi_master_write_data_byte(dev, 0x00);

		spi_master_write_comm_byte(dev, 0x26);	//Gamma Set
		spi_master_write_data_byte(dev, 0x01);

		spi_master_write_comm_byte(dev, 0xE0);	//Positive Gamma Correction
		spi_master_write_data_byte(dev, 0x0F);
		spi_master_write_data_byte(dev, 0x31);
		spi_master_write_data_byte(dev, 0x2B);
		spi_master_write_data_byte(dev, 0x0C);
		spi_master_write_data_byte(dev, 0x0E);
		spi_master_write_data_byte(dev, 0x08);
		spi_master_write_data_byte(dev, 0x4E);
		spi_master_write_data_byte(dev, 0xF1);
		spi_master_write_data_byte(dev, 0x37);
		spi_master_write_data_byte(dev, 0x07);
		spi_master_write_data_byte(dev, 0x10);
		spi_master_write_data_byte(dev, 0x03);
		spi_master_write_data_byte(dev, 0x0E);
		spi_master_write_data_byte(dev, 0x09);
		spi_master_write_data_byte(dev, 0x00);

		spi_master_write_comm_byte(dev, 0xE1);	//Negative Gamma Correction
		spi_master_write_data_byte(dev, 0x00);
		spi_master_write_data_byte(dev, 0x0E);
		spi_master_write_data_byte(dev, 0x14);
		spi_master_write_data_byte(dev, 0x03);
		spi_master_write_data_byte(dev, 0x11);
		spi_master_write_data_byte(dev, 0x07);
		spi_master_write_data_byte(dev, 0x31);
		spi_master_write_data_byte(dev, 0xC1);
		spi_master_write_data_byte(dev, 0x48);
		spi_master_write_data_byte(dev, 0x08);
		spi_master_write_data_byte(dev, 0x0F);
		spi_master_write_data_byte(dev, 0x0C);
		spi_master_write_data_byte(dev, 0x31);
		spi_master_write_data_byte(dev, 0x36);
		spi_master_write_data_byte(dev, 0x0F);

		spi_master_write_comm_byte(dev, 0x11);	//Sleep Out
		delayMS(120);

		spi_master_write_comm_byte(dev, 0x29);	//Display ON
	}			// endif 0x9340/0x9341/0x7735

	if (dev->_model == 0x9225) {
		ESP_LOGI(TAG, "Your TFT is ILI9225");
		ESP_LOGI(TAG, "Screen width: %d", width);
		ESP_LOGI(TAG, "Screen height: %d", height);
		lcd_write_register_byte(dev, 0x10, 0x0000);	// Set SAP,DSTB,STB
		lcd_write_register_byte(dev, 0x11, 0x0000);	// Set APON,PON,AON,VCI1EN,VC
		lcd_write_register_byte(dev, 0x12, 0x0000);	// Set BT,DC1,DC2,DC3
		lcd_write_register_byte(dev, 0x13, 0x0000);	// Set GVDD
		lcd_write_register_byte(dev, 0x14, 0x0000);	// Set VCOMH/VCOML voltage
		delayMS(40);

		// Power-on sequence
		lcd_write_register_byte(dev, 0x11, 0x0018);	// Set APON,PON,AON,VCI1EN,VC
		lcd_write_register_byte(dev, 0x12, 0x6121);	// Set BT,DC1,DC2,DC3
		lcd_write_register_byte(dev, 0x13, 0x006F);	// Set GVDD
		lcd_write_register_byte(dev, 0x14, 0x495F);	// Set VCOMH/VCOML voltage
		lcd_write_register_byte(dev, 0x10, 0x0800);	// Set SAP,DSTB,STB
		delayMS(10);
		lcd_write_register_byte(dev, 0x11, 0x103B);	// Set APON,PON,AON,VCI1EN,VC
		delayMS(50);

		lcd_write_register_byte(dev, 0x01, 0x011C);	// set the display line number and display direction
		lcd_write_register_byte(dev, 0x02, 0x0100);	// set 1 line inversion
		lcd_write_register_byte(dev, 0x03, 0x1030);	// set GRAM write direction and BGR=1.
		lcd_write_register_byte(dev, 0x07, 0x0000);	// Display off
		lcd_write_register_byte(dev, 0x08, 0x0808);	// set the back porch and front porch
		lcd_write_register_byte(dev, 0x0B, 0x1100);	// set the clocks number per line
		lcd_write_register_byte(dev, 0x0C, 0x0000);	// CPU interface
		//lcd_write_register_byte(dev, 0x0F, 0x0D01); // Set Osc
		lcd_write_register_byte(dev, 0x0F, 0x0801);	// Set Osc
		lcd_write_register_byte(dev, 0x15, 0x0020);	// Set VCI recycling
		lcd_write_register_byte(dev, 0x20, 0x0000);	// RAM Address
		lcd_write_register_byte(dev, 0x21, 0x0000);	// RAM Address

		// Set GRAM area
		lcd_write_register_byte(dev, 0x30, 0x0000);
		lcd_write_register_byte(dev, 0x31, 0x00DB);
		lcd_write_register_byte(dev, 0x32, 0x0000);
		lcd_write_register_byte(dev, 0x33, 0x0000);
		lcd_write_register_byte(dev, 0x34, 0x00DB);
		lcd_write_register_byte(dev, 0x35, 0x0000);
		lcd_write_register_byte(dev, 0x36, 0x00AF);
		lcd_write_register_byte(dev, 0x37, 0x0000);
		lcd_write_register_byte(dev, 0x38, 0x00DB);
		lcd_write_register_byte(dev, 0x39, 0x0000);

		// Adjust GAMMA Curve
		lcd_write_register_byte(dev, 0x50, 0x0000);
		lcd_write_register_byte(dev, 0x51, 0x0808);
		lcd_write_register_byte(dev, 0x52, 0x080A);
		lcd_write_register_byte(dev, 0x53, 0x000A);
		lcd_write_register_byte(dev, 0x54, 0x0A08);
		lcd_write_register_byte(dev, 0x55, 0x0808);
		lcd_write_register_byte(dev, 0x56, 0x0000);
		lcd_write_register_byte(dev, 0x57, 0x0A00);
		lcd_write_register_byte(dev, 0x58, 0x0710);
		lcd_write_register_byte(dev, 0x59, 0x0710);

		lcd_write_register_byte(dev, 0x07, 0x0012);
		delayMS(50);	// Delay 50ms
		lcd_write_register_byte(dev, 0x07, 0x1017);
	}			// endif 0x9225

	if (dev->_model == 0x9226) {
		ESP_LOGI(TAG, "Your TFT is ILI9225G");
		ESP_LOGI(TAG, "Screen width: %d", width);
		ESP_LOGI(TAG, "Screen height: %d", height);
		//lcd_write_register_byte(dev, 0x01, 0x011c);
		lcd_write_register_byte(dev, 0x01, 0x021c);
		lcd_write_register_byte(dev, 0x02, 0x0100);
		lcd_write_register_byte(dev, 0x03, 0x1030);
		lcd_write_register_byte(dev, 0x08, 0x0808);	// set BP and FP
		lcd_write_register_byte(dev, 0x0B, 0x1100);	// frame cycle
		lcd_write_register_byte(dev, 0x0C, 0x0000);	// RGB interface setting R0Ch=0x0110 for RGB 18Bit and R0Ch=0111for RGB16Bit
		lcd_write_register_byte(dev, 0x0F, 0x1401);	// Set frame rate----0801
		lcd_write_register_byte(dev, 0x15, 0x0000);	// set system interface
		lcd_write_register_byte(dev, 0x20, 0x0000);	// Set GRAM Address
		lcd_write_register_byte(dev, 0x21, 0x0000);	// Set GRAM Address
		//------------- Power On sequence --------------//
		delayMS(50);
		lcd_write_register_byte(dev, 0x10, 0x0800);	// Set SAP,DSTB,STB----0A00
		lcd_write_register_byte(dev, 0x11, 0x1F3F);	// Set APON,PON,AON,VCI1EN,VC----1038
		delayMS(50);
		lcd_write_register_byte(dev, 0x12, 0x0121);	// Internal reference voltage= Vci;----1121
		lcd_write_register_byte(dev, 0x13, 0x006F);	// Set GVDD----0066
		lcd_write_register_byte(dev, 0x14, 0x4349);	// Set VCOMH/VCOML voltage----5F60
		//-------------- Set GRAM area -----------------//
		lcd_write_register_byte(dev, 0x30, 0x0000);
		lcd_write_register_byte(dev, 0x31, 0x00DB);
		lcd_write_register_byte(dev, 0x32, 0x0000);
		lcd_write_register_byte(dev, 0x33, 0x0000);
		lcd_write_register_byte(dev, 0x34, 0x00DB);
		lcd_write_register_byte(dev, 0x35, 0x0000);
		lcd_write_register_byte(dev, 0x36, 0x00AF);
		lcd_write_register_byte(dev, 0x37, 0x0000);
		lcd_write_register_byte(dev, 0x38, 0x00DB);
		lcd_write_register_byte(dev, 0x39, 0x0000);
		// ----------- Adjust the Gamma Curve ----------//
		lcd_write_register_byte(dev, 0x50, 0x0001);
		lcd_write_register_byte(dev, 0x51, 0x200B);
		lcd_write_register_byte(dev, 0x52, 0x0000);
		lcd_write_register_byte(dev, 0x53, 0x0404);
		lcd_write_register_byte(dev, 0x54, 0x0C0C);
		lcd_write_register_byte(dev, 0x55, 0x000C);
		lcd_write_register_byte(dev, 0x56, 0x0101);
		lcd_write_register_byte(dev, 0x57, 0x0400);
		lcd_write_register_byte(dev, 0x58, 0x1108);
		lcd_write_register_byte(dev, 0x59, 0x050C);
		delayMS(50);
		lcd_write_register_byte(dev, 0x07, 0x1017);
	}			// endif 0x9226
}

void lcd_draw_pixel(tft_t *dev, uint16_t x, uint16_t y, uint16_t color)
{
	if (x >= dev->_width)
		return;
	if (y >= dev->_height)
		return;

	uint16_t _x = x + dev->_offsetx;
	uint16_t _y = y + dev->_offsety;

	if (dev->_model == 0x9340 || dev->_model == 0x9341
	    || dev->_model == 0x7796) {
		spi_master_write_comm_byte(dev, 0x2A);	// set column(x) address
		spi_master_write_addr(dev, _x, _x);
		spi_master_write_comm_byte(dev, 0x2B);	// set Page(y) address
		spi_master_write_addr(dev, _y, _y);
		spi_master_write_comm_byte(dev, 0x2C);	// Memory Write
		spi_master_write_data_word(dev, color);
	}			// endif 0x9340/0x9341/0x7796

	if (dev->_model == 0x7735) {
		spi_master_write_comm_byte(dev, 0x2A);	// set column(x) address
		spi_master_write_data_word(dev, _x);
		spi_master_write_data_word(dev, _x);
		spi_master_write_comm_byte(dev, 0x2B);	// set Page(y) address
		spi_master_write_data_word(dev, _y);
		spi_master_write_data_word(dev, _y);
		spi_master_write_comm_byte(dev, 0x2C);	// Memory Write
		spi_master_write_data_word(dev, color);
	}			// endif 0x7735

	if (dev->_model == 0x9225) {
		lcd_write_register_byte(dev, 0x20, _x);
		lcd_write_register_byte(dev, 0x21, _y);
		spi_master_write_comm_byte(dev, 0x22);	// Memory Write
		spi_master_write_data_word(dev, color);
	}			// endif 0x9225

	if (dev->_model == 0x9226) {
		lcd_write_register_byte(dev, 0x36, _x);
		lcd_write_register_byte(dev, 0x37, _x);
		lcd_write_register_byte(dev, 0x38, _y);
		lcd_write_register_byte(dev, 0x39, _y);
		lcd_write_register_byte(dev, 0x20, _x);
		lcd_write_register_byte(dev, 0x21, _y);
		spi_master_write_comm_byte(dev, 0x22);
		spi_master_write_data_word(dev, color);
	}			// endif 0x9226
}

void lcd_draw_multi_pixels(tft_t *dev, uint16_t x, uint16_t y, uint16_t size,
			   uint16_t *colors)
{
	if (x + size > dev->_width)
		return;
	if (y >= dev->_height)
		return;

	ESP_LOGD(TAG, "offset(x)=%d offset(y)=%d", dev->_offsetx,
		 dev->_offsety);
	uint16_t _x1 = x + dev->_offsetx;
	uint16_t _x2 = _x1 + size;
	uint16_t _y1 = y + dev->_offsety;
	uint16_t _y2 = _y1;
	ESP_LOGD(TAG, "_x1=%d _x2=%d _y1=%d _y2=%d", _x1, _x2, _y1, _y2);

	if (dev->_model == 0x9340 || dev->_model == 0x9341
	    || dev->_model == 0x7796) {
		spi_master_write_comm_byte(dev, 0x2A);	// set column(x) address
		spi_master_write_addr(dev, _x1, _x2);
		spi_master_write_comm_byte(dev, 0x2B);	// set Page(y) address
		spi_master_write_addr(dev, _y1, _y2);
		spi_master_write_comm_byte(dev, 0x2C);	// Memory Write
		spi_master_write_colors(dev, colors, size);
	}			// endif 0x9340/0x9341/0x7796

	if (dev->_model == 0x7735) {
		spi_master_write_comm_byte(dev, 0x2A);	// set column(x) address
		spi_master_write_data_word(dev, _x1);
		spi_master_write_data_word(dev, _x2);
		spi_master_write_comm_byte(dev, 0x2B);	// set Page(y) address
		spi_master_write_data_word(dev, _y1);
		spi_master_write_data_word(dev, _y2);
		spi_master_write_comm_byte(dev, 0x2C);	// Memory Write
		spi_master_write_colors(dev, colors, size);
	}			// 0x7735

	if (dev->_model == 0x9225) {
		for (int j = _y1; j <= _y2; j++) {
			lcd_write_register_byte(dev, 0x20, _x1);
			lcd_write_register_byte(dev, 0x21, j);
			spi_master_write_comm_byte(dev, 0x22);	// Memory Write
			spi_master_write_colors(dev, colors, size);
		}
	}			// endif 0x9225

	if (dev->_model == 0x9226) {
		for (int j = _x1; j <= _x2; j++) {
			lcd_write_register_byte(dev, 0x36, j);
			lcd_write_register_byte(dev, 0x37, j);
			lcd_write_register_byte(dev, 0x38, _y2);
			lcd_write_register_byte(dev, 0x39, _y1);
			lcd_write_register_byte(dev, 0x20, j);
			lcd_write_register_byte(dev, 0x21, _y1);
			spi_master_write_comm_byte(dev, 0x22);
			spi_master_write_colors(dev, colors, size);
		}
	}			// endif 0x9226

}

void lcd_draw_fill_rect(tft_t *dev, uint16_t x1, uint16_t y1, uint16_t x2,
			uint16_t y2, uint16_t color)
{
	if (x1 >= dev->_width)
		return;
	if (x2 >= dev->_width)
		x2 = dev->_width - 1;
	if (y1 >= dev->_height)
		return;
	if (y2 >= dev->_height)
		y2 = dev->_height - 1;

	ESP_LOGD(TAG, "offset(x)=%d offset(y)=%d", dev->_offsetx,
		 dev->_offsety);
	uint16_t _x1 = x1 + dev->_offsetx;
	uint16_t _x2 = x2 + dev->_offsetx;
	uint16_t _y1 = y1 + dev->_offsety;
	uint16_t _y2 = y2 + dev->_offsety;

	if (dev->_model == 0x9340 || dev->_model == 0x9341
	    || dev->_model == 0x7796) {
		spi_master_write_comm_byte(dev, 0x2A);	// set column(x) address
		spi_master_write_addr(dev, _x1, _x2);
		spi_master_write_comm_byte(dev, 0x2B);	// set Page(y) address
		spi_master_write_addr(dev, _y1, _y2);
		spi_master_write_comm_byte(dev, 0x2C);	// Memory Write
		for (int i = _x1; i <= _x2; i++) {
			uint16_t size = _y2 - _y1 + 1;
			spi_master_write_color(dev, color, size);
		}
	}			// endif 0x9340/0x9341/0x7796

	if (dev->_model == 0x7735) {
		spi_master_write_comm_byte(dev, 0x2A);	// set column(x) address
		spi_master_write_data_word(dev, _x1);
		spi_master_write_data_word(dev, _x2);
		spi_master_write_comm_byte(dev, 0x2B);	// set Page(y) address
		spi_master_write_data_word(dev, _y1);
		spi_master_write_data_word(dev, _y2);
		spi_master_write_comm_byte(dev, 0x2C);	// Memory Write
		for (int i = _x1; i <= _x2; i++) {
			uint16_t size = _y2 - _y1 + 1;
			spi_master_write_color(dev, color, size);
		}
	}			// 0x7735

	if (dev->_model == 0x9225) {
		for (int j = _y1; j <= _y2; j++) {
			lcd_write_register_byte(dev, 0x20, _x1);
			lcd_write_register_byte(dev, 0x21, j);
			spi_master_write_comm_byte(dev, 0x22);	// Memory Write
			uint16_t size = _x2 - _x1 + 1;
			spi_master_write_color(dev, color, size);
		}
	}			// endif 0x9225

	if (dev->_model == 0x9226) {
		for (int j = _x1; j <= _x2; j++) {
			lcd_write_register_byte(dev, 0x36, j);
			lcd_write_register_byte(dev, 0x37, j);
			lcd_write_register_byte(dev, 0x38, _y2);
			lcd_write_register_byte(dev, 0x39, _y1);
			lcd_write_register_byte(dev, 0x20, j);
			lcd_write_register_byte(dev, 0x21, _y1);
			spi_master_write_comm_byte(dev, 0x22);
			uint16_t size = _y2 - _y1 + 1;
			spi_master_write_color(dev, color, size);
		}
	}			// endif 0x9226

}

void lcd_display_off(tft_t *dev)
{
	if (dev->_model == 0x9340 || dev->_model == 0x9341
	    || dev->_model == 0x7735 || dev->_model == 0x7796) {
		spi_master_write_comm_byte(dev, 0x28);
	}			// endif 0x9340/0x9341/0x7735/0x7796

	if (dev->_model == 0x9225 || dev->_model == 0x9226) {
		lcd_write_register_byte(dev, 0x07, 0x1014);
	}			// endif 0x9225/0x9226

}

void lcd_display_on(tft_t *dev)
{
	if (dev->_model == 0x9340 || dev->_model == 0x9341
	    || dev->_model == 0x7735 || dev->_model == 0x7796) {
		spi_master_write_comm_byte(dev, 0x29);
	}			// endif 0x9340/0x9341/0x7735/0x7796

	if (dev->_model == 0x9225 || dev->_model == 0x9226) {
		lcd_write_register_byte(dev, 0x07, 0x1017);
	}			// endif 0x9225/0x9226

}

void lcd_inversion_off(tft_t *dev)
{
	if (dev->_model == 0x9340 || dev->_model == 0x9341
	    || dev->_model == 0x7735 || dev->_model == 0x7796) {
		spi_master_write_comm_byte(dev, 0x20);
	}			// endif 0x9340/0x9341/0x7735/0x7796

	if (dev->_model == 0x9225 || dev->_model == 0x9226) {
		lcd_write_register_byte(dev, 0x07, 0x1017);
	}			// endif 0x9225/0x9226
}

void lcd_inversion_on(tft_t *dev)
{
	if (dev->_model == 0x9340 || dev->_model == 0x9341
	    || dev->_model == 0x7735 || dev->_model == 0x7796) {
		spi_master_write_comm_byte(dev, 0x21);
	}			// endif 0x9340/0x9341/0x7735/0x7796

	if (dev->_model == 0x9225 || dev->_model == 0x9226) {
		lcd_write_register_byte(dev, 0x07, 0x1013);
	}			// endif 0x9225/0x9226
}

void lcd_bgr_filter(tft_t *dev)
{
	if (dev->_model == 0x9340 || dev->_model == 0x9341
	    || dev->_model == 0x7735 || dev->_model == 0x7796) {
		spi_master_write_comm_byte(dev, 0x36);	//Memory Access Control
		spi_master_write_data_byte(dev, 0x00);	//Right top start, RGB color filter panel
	}			// endif 0x9340/0x9341/0x7735/0x7796

	if (dev->_model == 0x9225 || dev->_model == 0x9226) {
		lcd_write_register_byte(dev, 0x03, 0x0030);	// set GRAM write direction and BGR=0.
	}			// endif 0x9225/0x9226
}

void lcd_fill_screen(tft_t *dev, uint16_t color)
{
	lcd_draw_fill_rect(dev, 0, 0, dev->_width - 1, dev->_height - 1, color);
}

void lcd_draw_line(tft_t *dev, uint16_t x1, uint16_t y1, uint16_t x2,
		   uint16_t y2, uint16_t color)
{
	int i;
	int dx, dy;
	int sx, sy;
	int E;

	/* distance between two points */
	dx = (x2 > x1) ? x2 - x1 : x1 - x2;
	dy = (y2 > y1) ? y2 - y1 : y1 - y2;

	/* direction of two point */
	sx = (x2 > x1) ? 1 : -1;
	sy = (y2 > y1) ? 1 : -1;

	/* inclination < 1 */
	if (dx > dy) {
		E = -dx;
		for (i = 0; i <= dx; i++) {
			lcd_draw_pixel(dev, x1, y1, color);
			x1 += sx;
			E += 2 * dy;
			if (E >= 0) {
				y1 += sy;
				E -= 2 * dx;
			}
		}

		/* inclination >= 1 */
	} else {
		E = -dy;
		for (i = 0; i <= dy; i++) {
			lcd_draw_pixel(dev, x1, y1, color);
			y1 += sy;
			E += 2 * dx;
			if (E >= 0) {
				x1 += sx;
				E -= 2 * dy;
			}
		}
	}
}

void lcd_draw_rect(tft_t *dev, uint16_t x1, uint16_t y1, uint16_t x2,
		   uint16_t y2, uint16_t color)
{
	lcd_draw_line(dev, x1, y1, x2, y1, color);
	lcd_draw_line(dev, x2, y1, x2, y2, color);
	lcd_draw_line(dev, x2, y2, x1, y2, color);
	lcd_draw_line(dev, x1, y2, x1, y1, color);
}

/*
 * Draw rectangle with angle
 * xc:Center X coordinate
 * yc:Center Y coordinate
 * w:Width of rectangle
 * h:Height of rectangle
 * angle :Angle of rectangle
 * color :color
 *
 * When the origin is (0, 0), the point (x1, y1) after rotating the point (x, y) by the angle is obtained by the following calculation.
 * x1 = x * cos(angle) - y * sin(angle)
 * y1 = x * sin(angle) + y * cos(angle)
 */
void lcd_draw_rect_angle(tft_t *dev, uint16_t xc, uint16_t yc, uint16_t w,
			uint16_t h, uint16_t angle, uint16_t color)
{
	double xd, yd, rd;
	int x1, y1;
	int x2, y2;
	int x3, y3;
	int x4, y4;
	rd = -angle * M_PI / 180.0;
	xd = 0.0 - w / 2;
	yd = h / 2;
	x1 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y1 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	yd = 0.0 - yd;
	x2 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y2 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	xd = w / 2;
	yd = h / 2;
	x3 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y3 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	yd = 0.0 - yd;
	x4 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y4 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	lcd_draw_line(dev, x1, y1, x2, y2, color);
	lcd_draw_line(dev, x1, y1, x3, y3, color);
	lcd_draw_line(dev, x2, y2, x4, y4, color);
	lcd_draw_line(dev, x3, y3, x4, y4, color);
}

/*
 * Draw triangle
 * xc:Center X coordinate
 * yc:Center Y coordinate
 * w:Width of triangle
 * h:Height of triangle
 * angle :Angle of triangle
 * color :color

 * When the origin is (0, 0), the point (x1, y1) after rotating the point (x, y) by the angle is obtained by the following calculation.
 * x1 = x * cos(angle) - y * sin(angle)
 * y1 = x * sin(angle) + y * cos(angle)
 */
void lcd_draw_triangle(tft_t *dev, uint16_t xc, uint16_t yc, uint16_t w,
		       uint16_t h, uint16_t angle, uint16_t color)
{
	double xd, yd, rd;
	int x1, y1;
	int x2, y2;
	int x3, y3;
	rd = -angle * M_PI / 180.0;
	xd = 0.0;
	yd = h / 2;
	x1 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y1 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	xd = w / 2;
	yd = 0.0 - yd;
	x2 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y2 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	xd = 0.0 - w / 2;
	x3 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y3 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	lcd_draw_line(dev, x1, y1, x2, y2, color);
	lcd_draw_line(dev, x1, y1, x3, y3, color);
	lcd_draw_line(dev, x2, y2, x3, y3, color);
}

void lcd_draw_circle(tft_t *dev, uint16_t x0, uint16_t y0, uint16_t r,
		     uint16_t color)
{
	int x;
	int y;
	int err;
	int old_err;

	x = 0;
	y = -r;
	err = 2 - 2 * r;
	do {
		lcd_draw_pixel(dev, x0 - x, y0 + y, color);
		lcd_draw_pixel(dev, x0 - y, y0 - x, color);
		lcd_draw_pixel(dev, x0 + x, y0 - y, color);
		lcd_draw_pixel(dev, x0 + y, y0 + x, color);
		if ((old_err = err) <= x)
			err += ++x * 2 + 1;
		if (old_err > y || err > x)
			err += ++y * 2 + 1;
	} while (y < 0);
}

void lcd_draw_fill_circle(tft_t *dev, uint16_t x0, uint16_t y0, uint16_t r,
			  uint16_t color)
{
	int x;
	int y;
	int err;
	int old_err;
	int ChangeX;

	x = 0;
	y = -r;
	err = 2 - 2 * r;
	ChangeX = 1;
	do {
		if (ChangeX) {
			lcd_draw_line(dev, x0 - x, y0 - y, x0 - x, y0 + y,
				      color);
			lcd_draw_line(dev, x0 + x, y0 - y, x0 + x, y0 + y,
				      color);
		}		// endif
		ChangeX = (old_err = err) <= x;
		if (ChangeX)
			err += ++x * 2 + 1;
		if (old_err > y || err > x)
			err += ++y * 2 + 1;
	} while (y <= 0);
}

void lcd_draw_round_rect(tft_t *dev, uint16_t x1, uint16_t y1, uint16_t x2,
			 uint16_t y2, uint16_t r, uint16_t color)
{
	int x;
	int y;
	int err;
	int old_err;
	unsigned char temp;

	if (x1 > x2) {
		temp = x1;
		x1 = x2;
		x2 = temp;
	}			// endif

	if (y1 > y2) {
		temp = y1;
		y1 = y2;
		y2 = temp;
	}			// endif

	ESP_LOGD(TAG, "x1=%d x2=%d delta=%d r=%d", x1, x2, x2 - x1, r);
	ESP_LOGD(TAG, "y1=%d y2=%d delta=%d r=%d", y1, y2, y2 - y1, r);
	if (x2 - x1 < r)
		return;		// Add 20190517
	if (y2 - y1 < r)
		return;		// Add 20190517

	x = 0;
	y = -r;
	err = 2 - 2 * r;

	do {
		if (x) {
			lcd_draw_pixel(dev, x1 + r - x, y1 + r + y, color);
			lcd_draw_pixel(dev, x2 - r + x, y1 + r + y, color);
			lcd_draw_pixel(dev, x1 + r - x, y2 - r - y, color);
			lcd_draw_pixel(dev, x2 - r + x, y2 - r - y, color);
		}		// endif
		if ((old_err = err) <= x)
			err += ++x * 2 + 1;
		if (old_err > y || err > x)
			err += ++y * 2 + 1;
	} while (y < 0);

	ESP_LOGD(TAG, "x1+r=%d x2-r=%d", x1 + r, x2 - r);
	lcd_draw_line(dev, x1 + r, y1, x2 - r, y1, color);
	lcd_draw_line(dev, x1 + r, y2, x2 - r, y2, color);
	ESP_LOGD(TAG, "y1+r=%d y2-r=%d", y1 + r, y2 - r);
	lcd_draw_line(dev, x1, y1 + r, x1, y2 - r, color);
	lcd_draw_line(dev, x2, y1 + r, x2, y2 - r, color);
}

void lcd_draw_arrow(tft_t *dev, uint16_t x0, uint16_t y0, uint16_t x1,
		    uint16_t y1, uint16_t w, uint16_t color)
{
	double Vx = x1 - x0;
	double Vy = y1 - y0;
	double v = sqrt(Vx * Vx + Vy * Vy);
	double Ux = Vx / v;
	double Uy = Vy / v;

	uint16_t L[2], R[2];
	L[0] = x1 - Uy * w - Ux * v;
	L[1] = y1 + Ux * w - Uy * v;
	R[0] = x1 + Uy * w - Ux * v;
	R[1] = y1 - Ux * w - Uy * v;

	lcd_draw_line(dev, x1, y1, L[0], L[1], color);
	lcd_draw_line(dev, x1, y1, R[0], R[1], color);
	lcd_draw_line(dev, L[0], L[1], R[0], R[1], color);
}

void lcd_draw_fill_arrow(tft_t *dev, uint16_t x0, uint16_t y0, uint16_t x1,
			 uint16_t y1, uint16_t w, uint16_t color)
{
	double Vx = x1 - x0;
	double Vy = y1 - y0;
	double v = sqrt(Vx * Vx + Vy * Vy);
	double Ux = Vx / v;
	double Uy = Vy / v;

	uint16_t L[2], R[2];
	L[0] = x1 - Uy * w - Ux * v;
	L[1] = y1 + Ux * w - Uy * v;
	R[0] = x1 + Uy * w - Ux * v;
	R[1] = y1 - Ux * w - Uy * v;

	lcd_draw_line(dev, x0, y0, x1, y1, color);
	lcd_draw_line(dev, x1, y1, L[0], L[1], color);
	lcd_draw_line(dev, x1, y1, R[0], R[1], color);
	lcd_draw_line(dev, L[0], L[1], R[0], R[1], color);

	int ww;
	for (ww = w - 1; ww > 0; ww--) {
		L[0] = x1 - Uy * ww - Ux * v;
		L[1] = y1 + Ux * ww - Uy * v;
		R[0] = x1 + Uy * ww - Ux * v;
		R[1] = y1 - Ux * ww - Uy * v;
		lcd_draw_line(dev, x1, y1, L[0], L[1], color);
		lcd_draw_line(dev, x1, y1, R[0], R[1], color);
	}
}

/*
 * RGB565 conversion
 * RGB565 is R(5)+G(6)+B(5)=16bit color format.
 * Bit image "RRRRRGGGGGGBBBBB"
 */
uint16_t rgb565_conv(uint16_t r, uint16_t g, uint16_t b)
{
	return (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

int lcd_draw_char(tft_t *dev, fontx_file_t *fxs, uint16_t x, uint16_t y,
		  char ascii, uint16_t color)
{
	uint16_t xx, yy, bit, ofs;
	unsigned char fonts[128];	// font pattern
	unsigned char pw, ph;
	int h, w;
	uint16_t mask;
	bool rc;

	rc = fontx_get(fxs, ascii, fonts, &pw, &ph);
	if (!rc)
		return 0;

	int16_t xd1 = 0;
	int16_t yd1 = 0;
	int16_t xd2 = 0;
	int16_t yd2 = 0;
	int16_t xss = 0;
	int16_t yss = 0;
	int16_t xsd = 0;
	int16_t ysd = 0;
	int16_t next = 0;
	int16_t x0 = 0;
	int16_t x1 = 0;
	int16_t y0 = 0;
	int16_t y1 = 0;
	if (dev->_font_direction == 0) {
		xd1 = +1;
		yd1 = +1;	//-1;
		xd2 = 0;
		yd2 = 0;
		xss = x;
		yss = y - (ph - 1);
		xsd = 1;
		ysd = 0;
		next = x + pw;

		x0 = x;
		y0 = y - (ph - 1);
		x1 = x + (pw - 1);
		y1 = y;
	} else if (dev->_font_direction == 2) {
		xd1 = -1;
		yd1 = -1;	//+1;
		xd2 = 0;
		yd2 = 0;
		xss = x;
		yss = y + ph + 1;
		xsd = 1;
		ysd = 0;
		next = x - pw;

		x0 = x - (pw - 1);
		y0 = y;
		x1 = x;
		y1 = y + (ph - 1);
	} else if (dev->_font_direction == 1) {
		xd1 = 0;
		yd1 = 0;
		xd2 = -1;
		yd2 = +1;	//-1;
		xss = x + ph;
		yss = y;
		xsd = 0;
		ysd = 1;
		next = y + pw;	//y - pw;

		x0 = x;
		y0 = y;
		x1 = x + (ph - 1);
		y1 = y + (pw - 1);
	} else if (dev->_font_direction == 3) {
		xd1 = 0;
		yd1 = 0;
		xd2 = +1;
		yd2 = -1;	//+1;
		xss = x - (ph - 1);
		yss = y;
		xsd = 0;
		ysd = 1;
		next = y - pw;	//y + pw;

		x0 = x - (ph - 1);
		y0 = y - (pw - 1);
		x1 = x;
		y1 = y;
	}

	if (dev->_font_fill)
		lcd_draw_fill_rect(dev, x0, y0, x1, y1, dev->_font_fill_color);

	int bits;
	ofs = 0;
	yy = yss;
	xx = xss;
	for (h = 0; h < ph; h++) {
		if (xsd)
			xx = xss;
		if (ysd)
			yy = yss;

		bits = pw;
		for (w = 0; w < ((pw + 4) / 8); w++) {
			mask = 0x80;
			for (bit = 0; bit < 8; bit++) {
				bits--;
				if (bits < 0)
					continue;
				if (fonts[ofs] & mask) {
					lcd_draw_pixel(dev, xx, yy, color);
				}
				if (h == (ph - 2) && dev->_font_underline)
					lcd_draw_pixel(dev, xx, yy,
						       dev->
						       _font_underline_color);
				if (h == (ph - 1) && dev->_font_underline)
					lcd_draw_pixel(dev, xx, yy,
						       dev->
						       _font_underline_color);
				xx = xx + xd1;
				yy = yy + yd2;
				mask = mask >> 1;
			}
			ofs++;
		}
		yy = yy + yd1;
		xx = xx + xd2;
	}

	if (next < 0)
		next = 0;
	return next;
}

int lcd_draw_string(tft_t *dev, fontx_file_t *fx, uint16_t x, uint16_t y,
		    const char *ascii, uint16_t color)
{
	int length = strlen(ascii);
	for (int i = 0; i < length; i++) {
		if (dev->_font_direction == 0)
			x = lcd_draw_char(dev, fx, x, y, ascii[i], color);
		if (dev->_font_direction == 1)
			y = lcd_draw_char(dev, fx, x, y, ascii[i], color);
		if (dev->_font_direction == 2)
			x = lcd_draw_char(dev, fx, x, y, ascii[i], color);
		if (dev->_font_direction == 3)
			y = lcd_draw_char(dev, fx, x, y, ascii[i], color);
	}
	if (dev->_font_direction == 0)
		return x;
	if (dev->_font_direction == 2)
		return x;
	if (dev->_font_direction == 1)
		return y;
	if (dev->_font_direction == 3)
		return y;
	return 0;
}

int lcd_draw_code(tft_t *dev, fontx_file_t *fx, uint16_t x, uint16_t y,
		  char code, uint16_t color)
{
	if (dev->_font_direction == 0)
		x = lcd_draw_char(dev, fx, x, y, code, color);
	if (dev->_font_direction == 1)
		y = lcd_draw_char(dev, fx, x, y, code, color);
	if (dev->_font_direction == 2)
		x = lcd_draw_char(dev, fx, x, y, code, color);
	if (dev->_font_direction == 3)
		y = lcd_draw_char(dev, fx, x, y, code, color);
	if (dev->_font_direction == 0)
		return x;
	if (dev->_font_direction == 2)
		return x;
	if (dev->_font_direction == 1)
		return y;
	if (dev->_font_direction == 3)
		return y;
	return 0;
}

void lcd_set_font_direction(tft_t *dev, uint16_t dir)
{
	dev->_font_direction = dir;
}

void lcd_set_font_fill(tft_t *dev, uint16_t color)
{
	dev->_font_fill = true;
	dev->_font_fill_color = color;
}

void lcd_unset_font_fill(tft_t *dev)
{
	dev->_font_fill = false;
}

void lcd_set_font_underline(tft_t *dev, uint16_t color)
{
	dev->_font_underline = true;
	dev->_font_underline_color = color;
}

void lcd_unset_font_underline(tft_t *dev)
{
	dev->_font_underline = false;
}

void lcd_set_scroll_area(tft_t *dev, uint16_t tfa, uint16_t vsa, uint16_t bfa)
{
	if (dev->_model == 0x9340 || dev->_model == 0x9341
	    || dev->_model == 0x7796) {
		spi_master_write_comm_byte(dev, 0x33);	// Vertical Scrolling Definition
		spi_master_write_data_word(dev, tfa);
		spi_master_write_data_word(dev, vsa);
		spi_master_write_data_word(dev, bfa);
	}			// endif 0x9340/0x9341/0x7796

	if (dev->_model == 0x9225 || dev->_model == 0x9226) {
		lcd_write_register_byte(dev, 0x31, vsa);	// Specify scroll end and step at the scroll display
		lcd_write_register_byte(dev, 0x32, tfa);	// Specify scroll start and step at the scroll display
	}			// endif 0x9225/0x9226
}

void lcd_reset_scroll_area(tft_t *dev, uint16_t vsa)
{
	if (dev->_model == 0x9340 || dev->_model == 0x9341
	    || dev->_model == 0x7796) {
		spi_master_write_comm_byte(dev, 0x33);	// Vertical Scrolling Definition
		spi_master_write_data_word(dev, 0);
		//spi_master_write_data_word(dev, 0x140);
		spi_master_write_data_word(dev, vsa);
		spi_master_write_data_word(dev, 0);
	}			// endif 0x9340/0x9341/0x7796

	if (dev->_model == 0x9225 || dev->_model == 0x9226) {
		lcd_write_register_byte(dev, 0x31, 0x0);	// Specify scroll end and step at the scroll display
		lcd_write_register_byte(dev, 0x32, 0x0);	// Specify scroll start and step at the scroll display
	}			// endif 0x9225/0x9226
}

// Vertical Scrolling Start Address
// vsp:Vertical Scrolling Start Address
void lcd_scroll(tft_t *dev, uint16_t vsp)
{
	if (dev->_model == 0x9340 || dev->_model == 0x9341
	    || dev->_model == 0x7796) {
		spi_master_write_comm_byte(dev, 0x37);	// Vertical Scrolling Start Address
		spi_master_write_data_word(dev, vsp);
	}			// endif 0x9340/0x9341/0x7796

	if (dev->_model == 0x9225 || dev->_model == 0x9226) {
		lcd_write_register_byte(dev, 0x33, vsp);	// Vertical Scrolling Start Address
	}			// endif 0x9225/0x9226
}

#define MAX_LEN		3
#define	XPT_START	0x80
#define XPT_XPOS	0x50
#define XPT_YPOS	0x10
#define XPT_8BIT	0x80
#define XPT_SER		0x04
#define XPT_DEF		0x03

int xptGetit(tft_t *dev, int cmd)
{
	char rbuf[MAX_LEN];
	char wbuf[MAX_LEN];

	memset(wbuf, 0, sizeof(rbuf));
	memset(rbuf, 0, sizeof(rbuf));
	wbuf[0] = cmd;
	spi_transaction_t SPITransaction;
	esp_err_t ret;

	memset(&SPITransaction, 0, sizeof(spi_transaction_t));
	SPITransaction.length = MAX_LEN * 8;
	SPITransaction.tx_buffer = wbuf;
	SPITransaction.rx_buffer = rbuf;
	ret = spi_device_transmit(dev->_XPT_Handle, &SPITransaction);
	assert(ret == ESP_OK);
	ESP_LOGD(TAG, "rbuf[0]=%02x rbuf[1]=%02x rbuf[2]=%02x", rbuf[0],
		 rbuf[1], rbuf[2]);
	// 12bit Conversion
	int pos = (rbuf[1] << 4) + (rbuf[2] >> 4);
	return (pos);
}

void xptGetxy(tft_t *dev, int *xp, int *yp)
{
	*xp = xptGetit(dev, (XPT_START | XPT_XPOS | XPT_SER));
	*yp = xptGetit(dev, (XPT_START | XPT_YPOS | XPT_SER));
}
