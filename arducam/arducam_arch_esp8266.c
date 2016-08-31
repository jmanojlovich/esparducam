/* 
 * The MIT License (MIT)
 * 
 * Copyright (c) 2015 Johan Kanflo (github.com/kanflo)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <FreeRTOS.h>
#include "task.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <esp/spi.h>
#include <i2c/i2c.h>
#include "arducam.h"
#include "arducam_arch.h"

#define ARDUCAM_CS  (16)

#define ESP8266_REG(addr) *((volatile uint32_t *)(0x60000000+(addr)))
#define GPFFS_GPIO(p) (((p)==0||(p)==2||(p)==4||(p)==5)?0:((p)==16)?1:3)

//GPIO 16 Control Registers
#define GP16O  ESP8266_REG(0x768)
#define GP16E  ESP8266_REG(0x774)
#define GP16I  ESP8266_REG(0x78C)

//GPIO 16 PIN Control Register
#define GP16C  ESP8266_REG(0x790)
#define GPC16  GP16C

//GPIO 16 PIN Function Register
#define GP16F  ESP8266_REG(0x7A0)
#define GPF16  GP16F

#define GP16FFS0 0 //Function Select bit 0
#define GP16FFS1 1 //Function Select bit 1
#define GP16FPD  3 //Pulldown
#define GP16FSPD 5 //Sleep Pulldown
#define GP16FFS2 6 //Function Select bit 2
#define GP16FFS(f) (((f) & 0x03) | (((f) & 0x04) << 4))

static uint8_t _sensor_addr;

/* #define CONFIG_VERIFY */

// D2
#define I2C_SDA_PIN (4)
// D5
#define I2C_SCK_PIN (5) 

static void spi_chip_select(uint8_t cs_pin);
static void spi_chip_unselect(uint8_t cs_pin);

bool arducam_spi_init(void)
{
	printf("arducam_spi_init\n");

	GPF16 = GP16FFS(GPFFS_GPIO(16));//Set mode to GPIO
	GPC16 = 0;
	GP16E |= 1;
	
	GP16O |= 1; // gpio_write(ARDUCAM_CS, 1);

	if(!spi_init(1, SPI_MODE0, SPI_FREQ_DIV_4M, true, SPI_BIG_ENDIAN, true)) {
	  printf("Failed to initialize SPI bus\n");
	  return false;
	}

	return true;
}

bool arducam_i2c_init(uint8_t sensor_addr)
{
	printf("arducam_i2c_init\n");
	i2c_init(I2C_SCK_PIN, I2C_SDA_PIN);
	_sensor_addr = sensor_addr;
	return true;
}

void arducam_delay_ms(uint32_t delay)
{
	vTaskDelay(delay / portTICK_RATE_MS);
}

void arducam_spi_write_data(uint8_t value)
{
	spi_chip_select(ARDUCAM_CS);
	spi_transfer_8(1, value);
	spi_chip_unselect(ARDUCAM_CS);
}

void arducam_spi_write(uint8_t address, uint8_t value)
{
#ifdef CONFIG_VERIFY
	static int counter = 0;
#endif // CONFIG_VERIFY

	uint8_t data[2] = {address, value};
	
	spi_chip_select(ARDUCAM_CS);
	
	spi_transfer_16(1, data[0] << 8 | data[1]);
	
	spi_chip_unselect(ARDUCAM_CS);
#ifdef CONFIG_VERIFY
	data[0] = arducam_spi_read(address & 0x7f);
//	printf("arducam_spi_write: [0x%02x] = 0x%02x\n", address & 0x7f, value);
	if (data[0] != value) {
		printf("arducam_spi_write: verify failed after %d for reg 0x%02x (0x%02x should be 0x%02x)\n", counter, address & 0x7f, data[0], value);
	}
	counter++;
#endif // CONFIG_VERIFY
}

uint8_t arducam_spi_read(uint8_t address)
{
	uint8_t data[2] = {address, 0x00};
	uint16_t out;
	spi_chip_select(ARDUCAM_CS);
	
	out = spi_transfer_16(1, data[0] << 8 | data[1]);
	spi_chip_unselect(ARDUCAM_CS);
	return (uint8_t)(out & 0x00ff);
}

uint8_t arducam_i2c_write(uint8_t regID, uint8_t regDat)
{
	uint8_t data[] = {regID, regDat};
	return i2c_slave_write(_sensor_addr, data, sizeof(data));
}

uint8_t arducam_i2c_read(uint8_t regID, uint8_t* regDat)
{
	return i2c_slave_read(_sensor_addr, regID, regDat, 1);
}

uint8_t arducam_i2c_write_8_16(uint8_t regID, uint16_t regDat)
{
	return i2c_slave_word_write(_sensor_addr, regID, (uint8_t*)&regDat, 1);
}

uint8_t arducam_i2c_read_8_16(uint8_t regID, uint16_t* regDat)
{
	return 0;
}

uint8_t arducam_i2c_write_16_8(uint16_t regID, uint8_t regDat)
{
	return i2c_slave_word_write(_sensor_addr, regID, &regDat, 1);
}

uint8_t arducam_i2c_read_16_8(uint16_t regID, uint8_t* regDat)
{
	return i2c_slave_word_read(_sensor_addr, regID, regDat, 1);
}

uint8_t arducam_i2c_write_8_8_array(const struct sensor_reg reglist[])
{
	uint16_t reg_addr = 0, reg_idx = 0;
	uint16_t reg_val = 0;
	const struct sensor_reg *next = reglist;

	while ((reg_addr != 0xff) | (reg_val != 0xff))
	{
		reg_addr = pgm_read_word(&next->reg);
		reg_val = pgm_read_word(&next->val);
		if (!arducam_i2c_write(reg_addr, reg_val)) {
			printf("arducam_i2c_write_regs failed at register %d\n", reg_idx);
			return 0;
		}
	   	next++;
	   	reg_idx++;
	}

	return 1;
}


int arducam_i2c_write_8_16_array(const struct sensor_reg reglist[])
{
	unsigned int reg_addr = 0, reg_val = 0;
	const struct sensor_reg *next = reglist;
	while ((reg_addr != 0xff) | (reg_val != 0xffff))
	{
		reg_addr = pgm_read_word(&next->reg);
		reg_val = pgm_read_word(&next->val);
		if (!arducam_i2c_write_8_16(reg_addr, reg_val)) {
			return 0;
		}
	   	next++;
	}

	return 1;
}

uint8_t arducam_i2c_write_16_8_array(const struct sensor_reg reglist[])
{
	unsigned int reg_addr = 0, reg_val = 0, reg_idx = 0;
	const struct sensor_reg *next = reglist;
	
	while ((reg_addr != 0xffff) | (reg_val != 0xff))
	{
		reg_addr = pgm_read_word(&next->reg);
		reg_val = pgm_read_word(&next->val);
		if (!arducam_i2c_write_16_8(reg_addr, reg_val)) {
			printf("arducam_i2c_write_16_8_array failed at register %d\n", reg_idx);
			return 0;
		}
		
	   	next++;
	   	reg_idx++;

		arducam_delay_ms(10);
	}

	return 1;
}

static void spi_chip_select(uint8_t cs_pin)
{
    GP16O &= ~1;
    arducam_delay_ms(1);
}

static void spi_chip_unselect(uint8_t cs_pin)
{
    GP16O |= 1;
    arducam_delay_ms(1);
}

void arducam_spi_chip_select()
{
  spi_chip_select(0);
}

void arducam_spi_chip_unselect()
{
  spi_chip_unselect(0);
}

