//
// CH32V Hardware Abstraction layer
// written by Larry Bank
// bitbank@pobox.com
//
// Copyright 2023 BitBank Software, Inc. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//    http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
#ifndef CH32V_HAL_H_
#define CH32V_HAL_H_

#include <stdio.h>
#include <stdint.h>

#ifdef CH32V003
#include "ch32v00x_i2c.h"
#include "ch32v00x_spi.h"
#include "ch32v00x_gpio.h"
#include "ch32v00x_rcc.h"
#define SRAM_SIZE 2048
#define FLASH_SIZE 16384
#else // CH32V203
#include "ch32v20x_i2c.h"
#include "ch32v20x_spi.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_rcc.h"
#define SRAM_SIZE 20480
#define FLASH_SIZE 65536
#endif

#define CTLR1_SPE_Set         ((uint16_t)0x0040)

#define BITBANG
// GPIO pin states
enum {
	OUTPUT = 0,
	INPUT,
	INPUT_PULLUP,
	INPUT_PULLDOWN
};

#define PROGMEM
#define memcpy_P memcpy
#define pgm_read_byte(s) *(uint8_t *)s
#define pgm_read_word(s) *(uint16_t *)s

// Wrapper methods
void delay(int i);
//
// Digital pin functions use a numbering scheme to make it easier to map the
// pin number to a port name and number
// The GPIO ports A-D become the most significant nibble of the pin number
// for example, to use Port C pin 7 (C7), use the pin number 0xC7
//
void pinMode(uint8_t u8Pin, int iMode);
uint8_t digitalRead(uint8_t u8Pin);
void digitalWrite(uint8_t u8Pin, uint8_t u8Value);
void digitalWrite8(uint8_t u8Port, uint8_t u8Value);

// The Wire library is a C++ class; I've created a work-alike to my
// BitBang_I2C API which is a set of C functions to simplify I2C
void I2CInit(uint8_t u8SDA, uint8_t u8SCL, int iSpeed);
void I2CWrite(uint8_t u8Addr, uint8_t *pData, int iLen);
int I2CRead(uint8_t u8Addr, uint8_t *pData, int iLen);
void I2CReadRegister(uint8_t u8Addr, uint8_t u8Reg, uint8_t *pData, int iLen);
int I2CTest(uint8_t u8Addr);
void I2CSetSpeed(int iSpeed);

// SPI1 (polling mode)
void SPI_write(uint8_t *pData, int iLen);
void SPI_begin(int iSpeed, int iMode, uint8_t MOSI, uint8_t CLK);


// Random stuff
void Standby82ms(uint8_t iTicks);
void breatheLED(uint8_t u8Pin, int iPeriod);
uint32_t udivmod32(uint32_t num, uint32_t den, uint32_t *pRemainder);
uint64_t udiv64(uint64_t num, uint64_t den);
void my_memset(void *pDest, uint8_t u8, int iLen);
void my_strcpy(void *pDest, void *pSrc);
void my_memcpy(void *pDest, void *pSrc, int iLen);
int my_strlen(const char *pString);
#endif /* CH32V_HAL_H_ */
