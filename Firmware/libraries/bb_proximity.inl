//       
// BitBank ALS/Proximity Sensor Library
// Written by Larry Bank
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
//===========================================================================
#include "bb_proximity.h"

static int _iAddr;
static int _iType;
static uint32_t _u32Caps;

uint16_t bbp_readWord(uint8_t reg)
{
uint16_t u16;
uint8_t u8;

        I2CReadRegister(_iAddr, reg | 0x80, &u8, 1);
        u16 = u8;
        I2CReadRegister(_iAddr, (reg+1) | 0x80, &u8, 1);
        u16 |= (u8 << 8);
        return u16;
} /* bbp_readWord() */

// 
// Initialize the library
// Detects if a supported sensor is available
// returns 1 for success, 0 for failure
// 
int bbp_init(void)
{
uint8_t ucTemp[4];

    _iType = BBP_TYPE_UNKNOWN;
// Detect the sensor type
    if (I2CTest(BBP_APDS99xx_ADDR)) { // could be APDS99xx
         _iAddr = BBP_APDS99xx_ADDR;
        I2CReadRegister(_iAddr, 0x80 | BBP_APDS_WHO_AM_I, ucTemp, 1); // get ID
        if (ucTemp[0] == BBP_APDS9930_ID) {
            _iType = BBP_TYPE_APDS9930;
            _u32Caps = BBP_CAPS_ALS | BBP_CAPS_PROXIMITY;
            return 1;
        } else if (ucTemp[0] == BBP_APDS9960_ID) {
            _iType = BBP_TYPE_APDS9960;
            _u32Caps = BBP_CAPS_ALS | BBP_CAPS_PROXIMITY | BBP_CAPS_GESTURE | BBP_CAPS_COLORS;
            return 1;
        }
    }
    if (I2CTest(BBP_LTR553_ADDR)) { // could be LTR-553ALS
        _iAddr = BBP_LTR553_ADDR;
        I2CReadRegister(_iAddr, BBP_LTR553_WHO_AM_I, ucTemp, 1); // get ID
        if (ucTemp[0] == BBP_LTR553_ID) {
            _iType = BBP_TYPE_LTR553;
            _u32Caps = BBP_CAPS_ALS | BBP_CAPS_PROXIMITY;
        }
    } 
    return 0; // no recognized sensor found
} /* bbp_init() */

int bbp_type(void)
{
    return _iType;
} /* bbp_type() */

// Sets the LED current boost value:
// APDS99xx: 0=12.5mA, 1=25mA, 2=50mA, 3=100mA
// LTR553: 0=5mA, 1 = 10mA, 2 = 20mA, 3 = 50mA, 4 = 100mA
void bbp_setLEDBoost(uint8_t boost) {
uint8_t u8, u8Temp[4];

    if (_iType == BBP_TYPE_LTR553) {
       u8 = 0x78; // default pulse period of 60kHz, 100% duty cycle
       u8 |= (boost & 0x7);
       u8Temp[0] = 0x82; // LED control register
       u8Temp[1] = u8;
    } else { // APDS-99xx
	if (_iType == BBP_TYPE_APDS9930) {
       		u8 = (3-boost)<<6; // PDRIVE 0=100mA, 1=50mA, 2=25mA, 3=12.5mA
       		u8 |= 0x20; // reserved PDIODE value of 10 = Ch1
       		u8 |= 0x00; // 1X gain for proximity, 1X gain for ALS
	} else { // APDS9960
		u8 = (3-boost)<<6; // PDRIVE 0=100ma, 1=50mA, 2=25mA, 3=12.5mA
		u8 |= 0x3; // proximity gain = 8x, ALS Gain = 1x
	}
       u8Temp[0] = 0x8f; // control register
       u8Temp[1] = u8;
    }
    I2CWrite(_iAddr, u8Temp, 2);
} /* bbp_setLEDBoost() */ 

void bbp_start(uint32_t iCaps)
{
uint8_t u8, u8Temp[4];

    if (_iType == BBP_TYPE_LTR553) {
        if (iCaps & BBP_CAPS_ALS) { // activate the ALS
             u8Temp[0] = 0x80; // ALS control
             u8Temp[1] = 0x01; // gain = 1X, not SW reset, active mode
             I2CWrite(_iAddr, u8Temp, 2);
        }
        if (iCaps & BBP_CAPS_PROXIMITY) { // activate the PS
             u8Temp[0] = 0x81; // PS control
             u8Temp[1] = 0x22; // saturation indicator on, active mode
             I2CWrite(_iAddr, u8Temp, 2);
        }
    } else if (_iType == BBP_TYPE_APDS9960 || _iType == BBP_TYPE_APDS9930) {
        u8 = 0x1; // | 0x8; // enable power + wait
        if (iCaps & BBP_CAPS_ALS)
             u8 |= 2; // AEN
        if (iCaps & BBP_CAPS_PROXIMITY)
             u8 |= 4; // PEN
        u8Temp[0] = 0x80; // control reg
        u8Temp[1] = u8;
        I2CWrite(_iAddr, u8Temp, 2);
	if (_iType == BBP_TYPE_APDS9930) {
        	u8Temp[0] = 0xA1; // ATIME, PTIME, WTIME
        	u8Temp[1] = 0xdf; // ALS integration time (0x100 - x) * 2.7ms
        	u8Temp[2] = 0xdf; // min prox integration time
        	u8Temp[3] = 0xdf; // min wait time
        	I2CWrite(_iAddr, u8Temp, 4);
        	u8Temp[0] = 0x8e; 
        	u8Temp[1] = 8; // min prox pulse count = 8
        	I2CWrite(_iAddr, u8Temp, 2);
		u8Temp[0] = 0x8f;
        	u8Temp[1] = 0x2d; // 100mA PDRIVE, Ch1 diode, PGAIN = 8x, AGAIN = 8x
        	I2CWrite(_iAddr, u8Temp, 2);
	} else { // APDS9960

		u8Temp[0] = 0x8e; // prox pulse count and width
		u8Temp[1] = 0xc8; // 32us pw, count=8
		I2CWrite(_iAddr, u8Temp, 2);
	}
    }
} /* bbp_start() */

void bbp_stop(void)
{
uint8_t u8Temp[4];

    if (_iType == BBP_TYPE_LTR553) {
         u8Temp[0] = 0x80; // ALS control
         u8Temp[1] = 0x00; // gain = 1X, not SW reset, inactive mode
         I2CWrite(_iAddr, u8Temp, 2);
         u8Temp[0] = 0x81; // PS control
         u8Temp[1] = 0x00; // saturation indicator off, inactive mode
         I2CWrite(_iAddr, u8Temp, 2);
    } else if (_iType == BBP_TYPE_APDS9930 || _iType == BBP_TYPE_APDS9960) {
        u8Temp[0] = 0xa0; // control reg
        u8Temp[1] = 0x00; // disable power
        I2CWrite(_iAddr, u8Temp, 2);
    }
} /* bbp_stop() */

int bbp_getGesture(void)
{
	return BBP_GESTURE_NONE; // DEBUG
} /* bbp_getGesture() */

int bbp_getColor(int *r, int *g, int *b, int *c)
{
    return 0; // DEBUG
} /* bbp_getColor() */

int bbp_getProximity(void)
{
uint8_t u8Temp[4];
int iDist = 0;

    if (_iType == BBP_TYPE_APDS9930 || _iType == BBP_TYPE_APDS9960) {
        I2CReadRegister(_iAddr, 0x93, u8Temp, 1); // see if PVALID is true
        if (u8Temp[0] & 2) { // PVALID
            if (_iType == BBP_TYPE_APDS9930) {
                //iDist = bbp_readWord(0x18);
                I2CReadRegister(_iAddr, 0x98, u8Temp, 1);
                I2CReadRegister(_iAddr, 0x99, &u8Temp[1], 1);
                iDist = u8Temp[0] + (u8Temp[1]<<8);
            } else { // APDS-9960
                I2CReadRegister(_iAddr, 0x9c, u8Temp, 1);
                iDist = u8Temp[0] << 8;
            }
        }
    }
    return iDist; // DEBUG - returns the photon counts (inverse of distance)
} /* bbp_getProximity() */

void bbp_setGestureSensitivity(uint8_t sensitivity)
{
   (void)sensitivity; // DEBUG
} /* bbp_setGestureSensitivity() */

void bbp_setInterruptMode(int iMode, int iThreshLow, int iThreshHigh)
{
    (void)iMode; (void) iThreshLow; (void)iThreshHigh;
} /* bbp_setInterruptMode() */

int bbp_getLight(void)
{
uint8_t u8Temp[4];
int LUX=0, iCH0, iCH1, IAC1, IAC2, IAC;

LUX = bbp_readWord(0x14); // clear data
return LUX;

        I2CReadRegister(_iAddr, 0x93, u8Temp, 1);
        if (u8Temp[0] & 1) { // ALS data is valid
	// read the Channel 0 (visible+ir) and 1 (ir) counts
        iCH0 = bbp_readWord(0x14);
        iCH1 = bbp_readWord(0x16);
// LUX equation
// IAC1 = CH0 - B * CH1
// IAC2 = C * CH0 - D * CH1
// IAC = Max(IAC1, IAC2, 0);
// LPC = GA * DF / (ALSIT * AGAIN)
// LUX = IAC * LPC
// Coefficients in open air:
// LPC (assumed) = 0.06, GA = 0.49, B = 1.862, C = 0.746, D = 1.291
        IAC1 = (iCH0*256) - (477 * iCH1); // keep as integers
        IAC2 = (iCH0*191) - (330 * iCH1);
        IAC = (IAC1 > IAC2) ? IAC1 : IAC2;
        if (IAC < 0) IAC = 0;
        LUX = (IAC * 61) >> 8;
        }
    return LUX;
}  /* bbp_getLight() */
