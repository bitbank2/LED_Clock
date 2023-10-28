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
#include "ch32v_hal.h"

//#define SPI_BITBANG

#ifdef BITBANG
uint8_t u8SDA_Pin, u8SCL_Pin;
int iDelay = 1;
#endif
static uint8_t u8MOSI, u8CLK;

#ifdef CH32V003
void SetClock(uint32_t u32Clock)
{
	uint32_t u32Div = 0;

if (u32Clock > 24000000)
	SystemCoreClock = 48000000;
else if (u32Clock > 12000000) {
	SystemCoreClock = 24000000;
	u32Div = RCC_HPRE_DIV1;
}
else if (u32Clock > 8000000) {
	SystemCoreClock = 12000000;
	u32Div = RCC_HPRE_DIV2;
}
else if (u32Clock > 6000000) {
	SystemCoreClock = 8000000;
	u32Div = RCC_HPRE_DIV3;
}
else if (u32Clock > 4800000) {
	SystemCoreClock = 6000000;
	u32Div = RCC_HPRE_DIV4;
}
else if (u32Clock > 4000000) {
	SystemCoreClock = 4800000;
	u32Div = RCC_HPRE_DIV5;
}
else if (u32Clock > 3428571) {
	SystemCoreClock = 4000000;
	u32Div = RCC_HPRE_DIV6;
}
else if (u32Clock >= 3000000) {
	SystemCoreClock = 3428571;
	u32Div = RCC_HPRE_DIV7;
}
else if (u32Clock > 1500000) {
	SystemCoreClock = 3000000;
	u32Div = RCC_HPRE_DIV8;
}
else if (u32Clock > 750000) {
	SystemCoreClock = 1500000;
	u32Div = RCC_HPRE_DIV16;
}
else if (u32Clock > 375000) {
	SystemCoreClock = 750000;
	u32Div = RCC_HPRE_DIV32;
}
else if (u32Clock > 187500) {
	SystemCoreClock = 375000;
	u32Div = RCC_HPRE_DIV64;
}
else {
	SystemCoreClock = 187500; // slowest setting for now
	u32Div = RCC_HPRE_DIV128;
}
switch (SystemCoreClock) {
case 48000000: // special case - needs PLL
    /* Flash 0 wait state */
    FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
    FLASH->ACTLR |= (uint32_t)FLASH_ACTLR_LATENCY_1;

    /* HCLK = SYSCLK = APB1 */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;

    /* PLL configuration: PLLCLK = HSI * 2 = 48 MHz */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_PLLSRC));
    RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSI_Mul2);

    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;
    /* Wait till PLL is ready */
    while((RCC->CTLR & RCC_PLLRDY) == 0)
    {
    }
    /* Select PLL as system clock source */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }
	break;
default: // simpler - just use the RC clock with a divider
    /* Flash 0 wait state */
    FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
    FLASH->ACTLR |= (SystemCoreClock >= 24000000) ? (uint32_t)FLASH_ACTLR_LATENCY_1 : (uint32_t)FLASH_ACTLR_LATENCY_0;

    /* HCLK = SYSCLK = APB1 */
    RCC->CFGR0 |= u32Div;
	break;
} // switch on clock
} /* SetClock() */
#else // CH32V203
//
// HSI = 8Mhz, so system clock is a PLL multiple
//
void SetClock(uint32_t u32Clock)
{
uint32_t u32Mul;

  RCC->CTLR |= (uint32_t)0x00000001;
  RCC->CFGR0 &= (uint32_t)0xF8FF0000;
  RCC->CTLR &= (uint32_t)0xFEF6FFFF;
  RCC->CTLR &= (uint32_t)0xFFFBFFFF;
  RCC->CFGR0 &= (uint32_t)0xFF80FFFF;
  RCC->INTR = 0x009F0000;    

	// Get the PLL multiplier
	u32Clock /= 8000000;
	if (u32Clock <= 1) { // 8Mhz
		RCC->CFGR0 &= ~(RCC_SWS); // set clock source to HSI (no PLL)
		RCC->CTLR &= ~RCC_PLLON; // turn off PLL
		SystemCoreClock = 8000000;
		return;
	}
	if (u32Clock < 2) {
		u32Clock = 2; // 16Mhz is the lowest we can go here
		SystemCoreClock = 16000000;
	} else if (u32Clock >= 17) {
	    u32Clock = 17; // 144Mhz max with a multiplier of 18, but the value is 17
	    SystemCoreClock = 144000000;
	} else {
		SystemCoreClock = u32Clock * 8000000;
	}
	u32Mul = (u32Clock-2) * RCC_PLLMul_3; // non-zero starting value for PLL multiplier
    EXTEN->EXTEN_CTR |= EXTEN_PLL_HSI_PRE;

    /* HCLK = SYSCLK */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV2;

    /*  PLL configuration: PLLCLK = HSI * N = u32Clock MHz */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_PLLSRC | RCC_PLLXTPRE | RCC_PLLMULL));

    RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSI_Div2 | u32Mul/*RCC_PLLMULL2*/);

    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;
    /* Wait till PLL is ready */
    while((RCC->CTLR & RCC_PLLRDY) == 0)
    {
    }
    /* Select PLL as system clock source */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }
} /* SetClock() */
#endif // CH32V203
void delay(int i)
{
	Delay_Ms(i);
}
// Arduino-like API defines and function wrappers for WCH MCUs

void pinMode(uint8_t u8Pin, int iMode)
{
GPIO_TypeDef *pGPIO=NULL;
uint32_t u32;

    if (u8Pin < 0xa0 || u8Pin > 0xdf) return; // invalid pin number

    switch (u8Pin & 0xf0) {
    case 0xa0:
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOA;
	pGPIO = GPIOA;
    	break;
#ifndef CH32V003
    case 0xb0:
        RCC->APB2PCENR |= RCC_APB2Periph_GPIOB;
        pGPIO = GPIOB;
        break;
#endif
    case 0xc0:
        RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;
	pGPIO = GPIOC;
    	break;
    case 0xd0:
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD;
	pGPIO = GPIOD;
    	break;
    }
    u8Pin &= 0xf; // isolate the pin from this port
    if (u8Pin < 8) {
	    u32 = pGPIO->CFGLR & ~(0xf << (4 * u8Pin)); // unset all flags
	    switch (iMode) {
		case OUTPUT:
			u32 |= (GPIO_Speed_10MHz | GPIO_Mode_Out_PP) << (4*u8Pin);
			break;
		case INPUT:
			u32 |= (GPIO_Mode_IN_FLOATING << (4*u8Pin));
			break;
		case INPUT_PULLUP:
                        pGPIO->BSHR = (1 << u8Pin);
			u32 |= (GPIO_Mode_IPU << (4*u8Pin));
			break;
		case INPUT_PULLDOWN:
                        pGPIO->BCR = (1 << u8Pin);
			u32 |= (GPIO_Mode_IPD << (4 * u8Pin));
			break;
	    } // switch on iMode
	    pGPIO->CFGLR = u32;
    } else { // pins 8-15
        u8Pin -= 8;
	    u32 = pGPIO->CFGHR & ~(0xf << (4 * u8Pin)); // unset all flags
	    switch (iMode) {
		case OUTPUT:
			u32 |= (GPIO_Speed_10MHz | GPIO_Mode_Out_PP) << (4*u8Pin);
			break;
		case INPUT:
			u32 |= (GPIO_Mode_IN_FLOATING << (4*u8Pin));
			break;
		case INPUT_PULLUP:
			u32 |= (GPIO_Mode_IPU << (4*u8Pin));
			pGPIO->BSHR = (1 << (u8Pin+8));
			break;
		case INPUT_PULLDOWN:
			u32 |= (GPIO_Mode_IPD << (4 * u8Pin));
			pGPIO->BCR = (1 << (u8Pin+8));
			break;
	    } // switch on iMode
	    pGPIO->CFGHR = u32;
    } // high numbered pins
} /* pinMode() */

uint8_t digitalRead(uint8_t u8Pin)
{
    uint32_t u32GPIO = 1 << (u8Pin & 0xf);
    uint32_t u32Value = 0;
    switch (u8Pin & 0xf0) {
    case 0xa0:
    	u32Value = GPIOA->INDR & u32GPIO;
    	break;
#ifndef CH32V003
    case 0xb0:
    	u32Value = GPIOB->INDR & u32GPIO;
    	break;
#endif
    case 0xc0:
    	u32Value = GPIOC->INDR & u32GPIO;
    	break;
    case 0xd0:
    	u32Value = GPIOD->INDR & u32GPIO;
    	break;
    }
    return (u32Value != 0);
} /* digitalRead() */

void digitalWrite(uint8_t u8Pin, uint8_t u8Value)
{
	uint32_t u32Value = 1 << (u8Pin & 0xf); // turn on bit
	if (!u8Value)
		u32Value <<= 16; // turn off bit 

	switch (u8Pin & 0xf0) {
	case 0xa0:
	    GPIOA->BSHR = u32Value;
	    break;
#ifndef CH32V003
	case 0xb0:
	    GPIOB->BSHR = u32Value;
	    break;
#endif
	case 0xc0:
     	   GPIOC->BSHR = u32Value;
           break;
	case 0xd0:
            GPIOD->BSHR = u32Value;
	    break;
	}
} /* digitalWrite() */

void digitalWrite8(uint8_t u8Port, uint8_t u8Value)
{
        switch (u8Port & 0xf0) {
        case 0xa0:
                GPIO_Write(GPIOA, u8Value);
                break;
#ifdef CH32V203
	case 0xb0:
		GPIO_Write(GPIOB, u8Value);
		break;
#endif
	case 0xc0:
                GPIO_Write(GPIOC, u8Value);
                break;
        case 0xd0:
                GPIO_Write(GPIOD, u8Value);
                break;
        }
} /* digitalWrite8() */

#ifdef BITBANG
uint8_t SDA_READ(void)
{
	return digitalRead(u8SDA_Pin);
}
void SDA_HIGH(void)
{
	pinMode(u8SDA_Pin, INPUT_PULLDOWN);
}
void SDA_LOW(void)
{
	pinMode(u8SDA_Pin, OUTPUT);
	digitalWrite(u8SDA_Pin, 0);
}
void SCL_HIGH(void)
{
	pinMode(u8SCL_Pin, INPUT_PULLDOWN);
}
void SCL_LOW(void)
{
	pinMode(u8SCL_Pin, OUTPUT);
	digitalWrite(u8SCL_Pin, 0);
}
void I2CSetSpeed(int iSpeed)
{
	if (iSpeed >= 400000) iDelay = 1;
	else if (iSpeed >= 100000) iDelay = 10;
	else iDelay = 20;
}
void I2CInit(uint8_t u8SDA, uint8_t u8SCL, int iSpeed)
{
	u8SDA_Pin = u8SDA;
	u8SCL_Pin = u8SCL;
	if (iSpeed >= 400000) iDelay = 1;
	else if (iSpeed >= 100000) iDelay = 10;
	else iDelay = 20;
} /* I2CInit() */

void my_sleep_us(int iDelay)
{
	Delay_Us(iDelay);
}
// Transmit a byte and read the ack bit
// if we get a NACK (negative acknowledge) return 0
// otherwise return 1 for success
//

int i2cByteOut(uint8_t b)
{
uint8_t i, ack;

for (i=0; i<8; i++)
{
//    my_sleep_us(iDelay);
    if (b & 0x80)
      SDA_HIGH(); // set data line to 1
    else
      SDA_LOW(); // set data line to 0
    b <<= 1;
//    my_sleep_us(iDelay);
    SCL_HIGH(); // clock high (slave latches data)
    my_sleep_us(iDelay);
    SCL_LOW(); // clock low
    my_sleep_us(iDelay);
} // for i
//my_sleep_us(iDelay);
// read ack bit
SDA_HIGH(); // set data line for reading
//my_sleep_us(iDelay);
SCL_HIGH(); // clock line high
my_sleep_us(iDelay); // DEBUG - delay/2
ack = SDA_READ();
//my_sleep_us(iDelay);
SCL_LOW(); // clock low
my_sleep_us(iDelay); // DEBUG - delay/2
SDA_LOW(); // data low
return (ack == 0); // a low ACK bit means success
} /* i2cByteOut() */

//
// Receive a byte and read the ack bit
// if we get a NACK (negative acknowledge) return 0
// otherwise return 1 for success
//
uint8_t i2cByteIn(uint8_t bLast)
{
uint8_t i;
uint8_t b = 0;

     SDA_HIGH(); // set data line as input
     for (i=0; i<8; i++)
     {
         my_sleep_us(iDelay); // wait for data to settle
         SCL_HIGH(); // clock high (slave latches data)
         my_sleep_us(iDelay);
         b <<= 1;
         if (SDA_READ() != 0) // read the data bit
           b |= 1; // set data bit
         SCL_LOW(); // clock low
     } // for i
     if (bLast)
        SDA_HIGH(); // last byte sends a NACK
     else
        SDA_LOW();
//     my_sleep_us(iDelay);
     SCL_HIGH(); // clock high
     my_sleep_us(iDelay);
     SCL_LOW(); // clock low to send ack
     my_sleep_us(iDelay);
//     SDA_HIGH();
     SDA_LOW(); // data low
  return b;
} /* i2cByteIn() */
//
// Send I2C STOP condition
//
void i2cEnd(void)
{
   SDA_LOW(); // data line low
   my_sleep_us(iDelay);
   SCL_HIGH(); // clock high
   my_sleep_us(iDelay);
   SDA_HIGH(); // data high
   my_sleep_us(iDelay);
} /* i2cEnd() */

int i2cBegin(uint8_t addr, uint8_t bRead)
{
   int rc;
//   SCL_HIGH();
//   my_sleep_us(iDelay);
   SDA_LOW(); // data line low first
   my_sleep_us(iDelay);
   SCL_LOW(); // then clock line low is a START signal
   addr <<= 1;
   if (bRead)
      addr++; // set read bit
   rc = i2cByteOut(addr); // send the slave address and R/W bit
   return rc;
} /* i2cBegin() */

void I2CWrite(uint8_t addr, uint8_t *pData, int iLen)
{
uint8_t b;
int rc;

   i2cBegin(addr, 0);
   rc = 1;
   while (iLen && rc == 1)
   {
      b = *pData++;
      rc = i2cByteOut(b);
      if (rc == 1) // success
      {
         iLen--;
      }
   } // for each byte
   i2cEnd();
//return (rc == 1) ? (iOldLen - iLen) : 0; // 0 indicates bad ack from sending a byte
} /* I2CWrite() */

int I2CRead(uint8_t addr, uint8_t *pData, int iLen)
{
   i2cBegin(addr, 1);
   while (iLen--)
   {
      *pData++ = i2cByteIn(iLen == 0);
   } // for each byte
   i2cEnd();
   return 1;
} /* I2CRead() */

int I2CTest(uint8_t addr)
{
int response = 0;

   if (i2cBegin(addr, 0)) // try to write to the given address
   {
      response = 1;
   }
   i2cEnd();
return response;
} /* I2CTest() */

#else // hardware I2C

void I2CSetSpeed(int iSpeed)
{
    I2C_InitTypeDef I2C_InitTSturcture={0};

    I2C_InitTSturcture.I2C_ClockSpeed = iSpeed;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
    I2C_InitTSturcture.I2C_OwnAddress1 = 0x02; //address; sender's unimportant address
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init( I2C1, &I2C_InitTSturcture );
} /* I2CSetSpeed() */

void I2CInit(uint8_t iSDA, uint8_t iSCL, int iSpeed)
{
	(void)iSDA; (void)iSCL;

#ifdef CH32V003
    // Fixed to pins C1/C2 for now
        // Enable GPIOC and I2C
        RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;
        RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

        // PC1 is SDA, 10MHz Output, alt func
        GPIOC->CFGLR &= ~(0xf<<(4*1));
        GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*1);

        // PC2 is SCL, 10MHz Output, alt func
        GPIOC->CFGLR &= ~(0xf<<(4*2));
        GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*2);
#else // CH32V203
    GPIO_InitTypeDef GPIO_InitStructure={0};
    // Fixed to pins B6 = SCL, B7 = SDA on TSSOP20 (remapped)
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );
#endif
    I2C_DeInit(I2C1);
    I2CSetSpeed(iSpeed);

    I2C_Cmd( I2C1, ENABLE );

    I2C_AcknowledgeConfig( I2C1, ENABLE );
//    while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );
} /* I2CInit() */

//
// Returns 0 for timeout error
// returns 1 for success
//
int I2CRead(uint8_t u8Addr, uint8_t *pData, int iLen)
{
	int iTimeout = 0;

    I2C_GenerateSTART( I2C1, ENABLE );
    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );

    I2C_Send7bitAddress( I2C1, u8Addr<<1, I2C_Direction_Receiver );

    while(iTimeout < 10000 && !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) ) {
    	iTimeout++;
    }
    if (iTimeout >= 10000) return 0; // error

    iTimeout = 0;
    while(iLen && iTimeout < 10000)
    {
        if( I2C_GetFlagStatus( I2C1, I2C_FLAG_RXNE ) !=  RESET )
        {
        	iTimeout = 0;
            pData[0] = I2C_ReceiveData( I2C1 );
            pData++;
            iLen--;
        } else {
        	iTimeout++;
        }
    }

    I2C_GenerateSTOP( I2C1, ENABLE );
    return (iLen == 0);

} /* I2CRead() */

void I2CWrite(uint8_t u8Addr, uint8_t *pData, int iLen)
{
    I2C_GenerateSTART( I2C1, ENABLE );
    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );

    I2C_Send7bitAddress( I2C1, u8Addr<<1, I2C_Direction_Transmitter );

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );

    while(iLen)
    {
        if( I2C_GetFlagStatus( I2C1, I2C_FLAG_TXE ) !=  RESET )
        {
            I2C_SendData( I2C1, pData[0] );
            pData++;
            iLen--;
        }
    }

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
    I2C_GenerateSTOP( I2C1, ENABLE );

} /* I2CWrite() */

int I2CTest(uint8_t u8Addr)
{
	int iTimeout = 0;

	I2C_ClearFlag(I2C1, I2C_FLAG_AF);
    I2C_GenerateSTART( I2C1, ENABLE );
    while(iTimeout < 10000 && !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) ) {
    	iTimeout++;
    }
    if (iTimeout >= 10000) return 0; // no pull-ups, open bus

    I2C_Send7bitAddress( I2C1, u8Addr<<1, I2C_Direction_Transmitter );

    while(iTimeout < 10000 && !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) ) {
    	iTimeout++;
    }
    if (iTimeout >= 10000) return 0; // no device at that address; the MTMS flag will never get set

    I2C_GenerateSTOP( I2C1, ENABLE );
    // check ACK failure flag
    return (I2C_GetFlagStatus(I2C1, /*I2C_FLAG_TXE*/I2C_FLAG_AF) == RESET); // 0 = fail, 1 = succeed

} /* I2CTest() */
#endif // !BITBANG

//
// Read N bytes starting at a specific I2C internal register
// returns 1 for success, 0 for error
//
void I2CReadRegister(uint8_t iAddr, uint8_t u8Register, uint8_t *pData, int iLen)
{
  I2CWrite(iAddr, &u8Register, 1);
  I2CRead(iAddr, pData, iLen);
} /* I2CReadRegister() */

void SPI_begin(int iSpeed, int iMode, uint8_t MOSI, uint8_t CLK)
{
uint32_t u32, u32Prescaler = 0;

#ifdef SPI_BITBANG
    u8MOSI = MOSI; u8CLK = CLK;
	if (MOSI != 0) { // bit bang
		pinMode(MOSI, OUTPUT);
		pinMode(CLK, OUTPUT);
		return;
	}
 #endif
 
    u32 = SystemCoreClock >> 1; // core/2 is the fastest (prescaler = 0)
    while (iSpeed < u32) {
    	u32Prescaler += 8; // each division by 2 adds 8
    	u32 >>= 1;
    }
    if (u32Prescaler > 0x38) u32Prescaler = 0x38; // max /256

#ifdef CH32V003
        // Enable GPIOC and SPI
        RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1;

	// PC5 is SCK, 10MHz Output, alt func, p-p
        GPIOC->CFGLR &= ~(0xf<<(4*5));
        GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_Mode_AF_PP)<<(4*5);

        // PC6 is MOSI, 10MHz Output, alt func, p-p
        GPIOC->CFGLR &= ~(0xf<<(4*6));
        GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_Mode_AF_PP)<<(4*6);
#else // CH32V203
    GPIO_InitTypeDef GPIO_InitStructure={0};
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE );
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; // SPI1 CLK
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOA, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; // SPI1 MOSI
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOA, &GPIO_InitStructure );
#endif
        // Configure SPI 
        SPI1->CTLR1 =
                SPI_NSS_Soft | SPI_CPHA_1Edge | SPI_CPOL_Low | SPI_DataSize_8b |
                SPI_Mode_Master | SPI_Direction_1Line_Tx |
                u32Prescaler;

	// Enable DMA on SPI
	SPI1->CTLR2 |= SPI_I2S_DMAReq_Tx;

        // enable SPI port
        SPI1->CTLR1 |= CTLR1_SPE_Set;
    //SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE); // enable DMA on transmit

    //SPI_Cmd( SPI1, ENABLE );
} /* SPI_begin() */

#ifdef SPI_BITBANG
// bitbang write
void SPI_writeByte(uint8_t u8Data)
{
int iBit;
   if (u8Data == 0 || u8Data == 0xff) { // faster
      digitalWrite(u8MOSI, u8Data & 1);
      for (iBit=0; iBit<8; iBit++) {
          digitalWrite(u8CLK, 1);
          digitalWrite(u8CLK, 0);
      }
   } else {
      for (iBit=0; iBit<8; iBit++) {
          digitalWrite(u8MOSI, u8Data & 0x80); // MSB first
          digitalWrite(u8CLK, 1);
          u8Data <<= 1;
          digitalWrite(u8CLK, 0);
      }
   }
} /* SPI_writeByte() */
#endif

// polling write
void SPI_write(uint8_t *pData, int iLen)
{
int i = 0;

    while (i < iLen) {
#ifdef SPI_BITBANG
        if (u8MOSI != 0) { // bit bang
            SPI_writeByte(*pData++);
        } else
#endif
        {
        	while(!(SPI1->STATR & SPI_STATR_TXE)); // wait for TXE
        	SPI1->DATAR = *pData++; // send data
        }
	i++;
    }
#ifdef SPI_BITBANG
    if (u8MOSI == 0)
#endif
    {
        while(SPI1->STATR & SPI_STATR_BSY); // wait for not busy
    }
} /* SPI_write() */
// 
// 64-bit unsigned divide
// takes less flash space than the 2K used by the RISC-V math library
// 
uint64_t udiv64(uint64_t num, uint64_t den)
{
uint64_t place = 1;
uint64_t ret = 0;
   while ((num >> 1) >= den) {
      place<<=1;
      den<<=1;
   }
   for ( ; place>0; place>>=1, den>>=1) {
      if (num>=den) {
         num-=den;
         ret+=place;
      }
   }
   return ret;
} /* udiv64() */

//
// 32-bit unsigned divide and modulus
// takes less flash space than the 2K used by the RISC-V math library
//
uint32_t udivmod32(uint32_t num, uint32_t den, uint32_t *pRemainder)
{
uint32_t place = 1;
uint32_t ret = 0;
   while ((num >> 1) >= den) {
      place<<=1;
      den<<=1;
   }
   for ( ; place>0; place>>=1, den>>=1) {
      if (num>=den) {
         num-=den;
         ret+=place;
      }
   }
   if (pRemainder) *pRemainder = num;
   return ret;
} /* udivmod32() */

void my_memcpy(void *pDest, void *pSrc, int iLen)
{
uint8_t *s = (uint8_t *)pSrc;
uint8_t *d = (uint8_t *)pDest;
	while (iLen) {
		*d++ = *s++;
		iLen--;
	}
} /* my_memcpy() */

void my_memset(void *pDest, uint8_t u8, int iLen)
{
uint8_t *d = (uint8_t *)pDest;
	while (iLen) {
		*d++ = u8;
		iLen--;
	}
} /* my_memset() */

void my_strcpy(void *pDest, void *pSrc)
{
	uint8_t *s = (uint8_t *)pSrc;
	uint8_t *d = (uint8_t *)pDest;
	while (*s != 0) {
		*d++ = *s++;
	}
} /* my_strcpy() */

int my_strlen(const char *pString)
{
int iLen = 0;
	while (*pString != 0) {
		iLen++;
		pString++;
	}
return iLen;
} /* my_strlen() */
