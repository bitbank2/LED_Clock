//
// LED Clock
//
#include "debug.h"
#define CH32V003
#define BITBANG
#include "../../libraries/ch32v_hal.inl"
#include "../../libraries/rtc_eeprom.inl"
#include "../../libraries/bb_proximity.inl"

#define SDA_PIN 0xc0
#define SCL_PIN 0xd0
#define BUTT0_PIN 0xa1
#define BUTT1_PIN 0xa2
#define COLON_PIN 0xd6
struct tm myTime;
int iLux, iBrightness; // a value from 0 to 255 indicating PWM duty
#define MAX_BRIGHTNESS 256
uint8_t u8Cols[] = {0xd2, 0xd3, 0xd4, 0xd5, 0xd7};
const uint8_t u8Dig2Seg[10] = {0x3f<<1, 0x06<<1, 0x5b<<1, 0x4f<<1, 0x66<<1, 0x6d<<1, 0x7d<<1, 0x07<<1, 0x7f<<1, 0x6f<<1};

void Option_Byte_CFG(void)
{
    FLASH_Unlock();
    FLASH_EraseOptionBytes();
    FLASH_UserOptionByteConfig(OB_IWDG_SW, OB_STOP_NoRST, OB_STDBY_NoRST, OB_RST_NoEN);
    FLASH_Lock();
}

void ShowDigit(uint8_t u8Digit, uint8_t u8Column, int iBright, int iDelay)
{
	int iOn, iOff;
	uint8_t u8;

	if (iBright <= 0) {
		Delay_Us(iDelay);
		return;
	}
	iOn = (iDelay * iBright) >> 8;
	iOff = (iDelay - iOn);
	if (u8Column == 4) { // colon
		if (u8Digit) {
			digitalWrite(COLON_PIN, 1);
			Delay_Us(iOn);
			digitalWrite(COLON_PIN, 0);
			Delay_Us(iOff);
		} else {
			Delay_Us(iOn+iOff); // keep time consistent
		}
	} else {
		u8 = u8Dig2Seg[u8Digit];
		digitalWrite8(0xc0, u8);
		digitalWrite(u8Cols[u8Column], 1);
		Delay_Us(iOn);
		digitalWrite(u8Cols[u8Column], 0);
		Delay_Us(iOff);
	}
} /* ShowDigit() */

void AddMinutes(int iMin)
{
	myTime.tm_min += iMin;
	if (myTime.tm_min >= 60) {
		myTime.tm_min -= 60;
		myTime.tm_hour++;
		if (myTime.tm_hour > 23)
			myTime.tm_hour = 0;
	}
	rtcSetTime(&myTime);
} /* AddMinutes()*/

int GetDistance(void)
{
    int i, iLux, iDist;
    const uint8_t iDistLow[26] = {200,200,200,200,200,200,200,200,
                               200,200,150,140,130,120,110,105,
                               100,94,90,86,82,78,74,73,72,71};
    const uint8_t iDistHigh[26] = {  70, 68, 65, 60, 58, 55, 50, 48,
                                44, 42, 40, 38, 37, 36, 35, 34,
                                33, 32, 31, 30, 30, 30, 30, 30, 30, 30};
    i = bbp_getProximity();
    if (i < 192) iDist = iDistLow[i >> 3];
    else iDist = iDistHigh[(i >> 5)-6];
    return iDist;
} /* GetDistance() */

void UpdateBrightness(int iTicks)
{
    iLux = bbp_getLight();
    if (iLux <= 20 && iBrightness != 0 && iTicks == 0) { // room went dark
    	iBrightness = 0; // go dark
    } else if (iLux > 20 && iBrightness == 0) { // room went light
		iBrightness = 50+iLux;
		if (iBrightness > MAX_BRIGHTNESS) {
			iBrightness = MAX_BRIGHTNESS; // max brightness
		}
    }
} /* UpdateBrightness() */

int main(void)
{
int i, j, iTicks = 0;
int iDistance;
int iColor;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
//    Option_Byte_CFG(); // allow PD7 to be used as GPIO
    rtcInit(RTC_DS3231, SDA_PIN, SCL_PIN);
    pinMode(BUTT0_PIN, INPUT_PULLUP);
    pinMode(BUTT1_PIN, INPUT_PULLUP);
    pinMode(COLON_PIN, OUTPUT);
    bbp_init();
    bbp_start(BBP_CAPS_ALS | BBP_CAPS_PROXIMITY);
    bbp_setLEDBoost(3);

    for (i=0; i<5; i++) {
    	pinMode(u8Cols[i], OUTPUT);
    	digitalWrite(u8Cols[i], 0);
    }
    for (i=0xc1; i<=0xc7; i++) { // enable all rows as outputs
    	pinMode(i, OUTPUT);
    	digitalWrite(i, 0);
    }
    rtcGetTime(&myTime);
    UpdateBrightness(0); // start from reset with the appropriate brightness
    while(1)
    {
 //   	i = bbp_getLight();
//    	i = bbp_getProximity();
        for (j=0; j<200; j++) {
#ifdef BOGUS
        	ShowDigit(i/1000, 0, iBrightness,1000);
        	ShowDigit((i % 1000)/100, 1, iBrightness, 1000);
        	ShowDigit((i % 100)/10, 2, iBrightness,1000);
        	ShowDigit(i % 10, 3, iBrightness, 1000);
#else
        	if (myTime.tm_hour > 9) {
        		i = myTime.tm_hour / 10;
   				ShowDigit(i, 0, iBrightness, 1000);
        	} else {
        		Delay_Us(1000);
        	}
        	i = myTime.tm_hour % 10;
   			ShowDigit(i, 1, iBrightness, 1000);
        	i = myTime.tm_min / 10;
   			ShowDigit(i, 2, iBrightness, 1000);
        	i = myTime.tm_min % 10;
   			ShowDigit(i, 3, iBrightness, 1000);
   			ShowDigit(myTime.tm_sec & 1, 4, iBrightness, 1000); // blinking colon
   			if ((j == 0 || j == 100) && digitalRead(BUTT1_PIN) == 0) { // fast change
   				AddMinutes(10);
   			}
#endif
        } // for j
		if (digitalRead(BUTT0_PIN) == 0) { // slow change
			AddMinutes(1);
		}

#ifndef FUTURE
		if ((myTime.tm_sec & 7) == 7) { // check light level every 8 seconds
			UpdateBrightness(iTicks);
		}
#endif
		if ((myTime.tm_sec & 15) == 15) {
	        rtcGetTime(&myTime); // keep the time accurate
		} else { // manually increment the time to prevent flicker on the LEDs
			myTime.tm_sec++;
			if (myTime.tm_sec >= 60) {
				myTime.tm_sec -= 60;
				myTime.tm_min++;
				if (myTime.tm_min >= 60) {
					myTime.tm_min -= 60;
					myTime.tm_hour++;
				}
			}
		}
#ifndef FUTURE
		if (iLux < 20) { // dark room
//			iDistance = GetDistance();
//			i = 160 - iDistance; // hand nearby?
		    i = bbp_getProximity();
			if (i > 2100)  { // hand nearby
				iTicks = 5;
				iBrightness = 20;
			} else if (iTicks > 0) {
				iTicks--;
				if (iTicks == 0) iBrightness = 0; // go black again after 5 seconds
			} else {
				Delay_Ms(250); // waiting for more samples
			}
		}
#endif
    }
} /* main() */
