#include "debug.h"
#define CH32V003
#include "../../libraries/ch32v_hal.inl"
#include "../../libraries/rtc_eeprom.inl"
#include "../../libraries/bb_proximity.inl"

#define SDA_PIN 0xc0
#define SCL_PIN 0xd0
#define BUTT0_PIN 0xa1
#define BUTT1_PIN 0xa2
struct tm myTime;
int iBrightness = 85; // a value from 0 to 255 indicating PWM duty
uint8_t u8Cols[] = {0xd2, 0xd3, 0xd4, 0xd5, 0xd7};
const uint8_t u8Dig2Seg[12] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x03, 0x00};
enum {
	COLOR_RED,
	COLOR_GREEN,
	COLOR_YELLOW
};

void Option_Byte_CFG(void)
{
    FLASH_Unlock();
    FLASH_EraseOptionBytes();
    FLASH_UserOptionByteConfig(OB_IWDG_SW, OB_STOP_NoRST, OB_STDBY_NoRST, OB_RST_NoEN);
    FLASH_Lock();
}

void ShowDigit(uint8_t u8Digit, uint8_t u8Column, uint8_t u8Color, int iDelay)
{
	int i, iOn, iOff;
	uint8_t u8;

	if (iBrightness <= 0) return;
	u8 = u8Dig2Seg[u8Digit];
	pinMode(u8Cols[u8Column], OUTPUT);
    digitalWrite(u8Cols[u8Column], u8Color == COLOR_RED);
	if (u8Color == COLOR_RED) u8 = ~u8;
	digitalWrite8(0xc0, u8 << 1);
//    for (i=0; i<7; i++) {
//    	digitalWrite(0xc1+i, u8 & 1);
//    	u8 >>= 1;
//    }
    iOn = (iDelay * iBrightness) >> 8;
    iOff = (iDelay - iOn);
    Delay_Us(iOn);
    pinMode(u8Cols[u8Column], INPUT);
    digitalWrite(u8Cols[u8Column], 1);
    Delay_Us(iOff);

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

int GetALS(int *lux)
{
    int i, iLux, iDist;
    const uint8_t iDistLow[26] = {200,200,200,200,200,200,200,200,
                               200,200,150,140,130,120,110,105,
                               100,94,90,86,82,78,74,73,72,71};
    const uint8_t iDistHigh[26] = {  70, 68, 65, 60, 58, 55, 50, 48,
                                44, 42, 40, 38, 37, 36, 35, 34,
                                33, 32, 31, 30, 30, 30, 30, 30, 30, 30};
    iLux = bbp_getLight();
    i = bbp_getProximity();
    if (i < 192) iDist = iDistLow[i >> 3];
    else iDist = iDistHigh[(i >> 5)-6];
    if (lux) *lux = iLux;
    return iDist;
} /* GetALS() */

int main(void)
{
int i, j, iTicks;
int iDistance, iLux, iDayMode = 1;
int iColor;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    Delay_Ms(2000);
//    Option_Byte_CFG(); // allow PD7 to be used as GPIO
    rtcInit(RTC_DS3231, SDA_PIN, SCL_PIN);
    pinMode(BUTT0_PIN, INPUT_PULLUP);
    pinMode(BUTT1_PIN, INPUT_PULLUP);
    bbp_init();
    bbp_start(BBP_CAPS_ALS | BBP_CAPS_PROXIMITY);
    bbp_setLEDBoost(3);

    for (i=0; i<5; i++) {
    	pinMode(u8Cols[i], INPUT);
    	digitalWrite(u8Cols[i], 1);
    }
    for (i=0xc1; i<=0xc7; i++) { // enable all rows as outputs
    	pinMode(i, OUTPUT);
    	digitalWrite(i, 0);
    }
    j = 0;
    iColor = COLOR_RED;
    while(1)
    {
        rtcGetTime(&myTime);
        for (j=0; j<200; j++) {
        	i = myTime.tm_hour / 10;
   			ShowDigit(i, 0, iColor, 1000);
        	i = myTime.tm_hour % 10;
   			ShowDigit(i, 1, iColor, 1000);
        	i = myTime.tm_min / 10;
   			ShowDigit(i, 2, iColor, 1000);
        	i = myTime.tm_min % 10;
   			ShowDigit(i, 3, iColor, 1000);
   			ShowDigit((myTime.tm_sec & iDayMode) ? 10:11, 4, iColor, 1000);
        }
		if (digitalRead(BUTT0_PIN) == 0) { // slow change
			AddMinutes(1);
		}
		if (digitalRead(BUTT1_PIN) == 0) { // fast change
			AddMinutes(10);
		}
		iDistance = GetALS(&iLux);
		if (iLux == 0) { // dark room
			iDayMode = 0; // don't blink colons at night
			i = 160 - iDistance; // dark room
			if (i > iBrightness)  {
				// brightness just increased; keep it that way for 5 seconds
				iTicks = 5;
				iBrightness = i;
			} else {
				if (iTicks) iTicks--;
				if (iTicks == 0) iBrightness = i;
			}
		} else {
			iDayMode = 1;
			if (iLux < 30) {
				iBrightness = 25;
			} else {
				iBrightness = iLux;
				if (iBrightness > 256) {
					iBrightness = 256;
				}
			}
		}
    }
} /* main() */
