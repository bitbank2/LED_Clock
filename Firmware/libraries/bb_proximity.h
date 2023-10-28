//
// ALS/Proximity Sensor Library
// written by Larry Bank
// Project started 10/21/2023
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

#ifndef __BB_PROXIMITY__
#define __BB_PROXIMITY__

enum {
   BBP_TYPE_UNKNOWN = 0,
   BBP_TYPE_APDS9930,
   BBP_TYPE_APDS9960,
   BBP_TYPE_LTR553
};

#define BBP_APDS99xx_ADDR 0x39
#define BBP_LTR553_ADDR 0x23

#define BBP_APDS_WHO_AM_I 0x12
#define BBP_APDS9930_ID 0x39
#define BBP_APDS9960_ID 0xAB

#define BBP_LTR553_WHO_AM_I 0x86
#define BBP_LTR553_ID 0x92

#define BBP_CAPS_ALS       1
#define BBP_CAPS_PROXIMITY 2
#define BBP_CAPS_GESTURE   4
#define BBP_CAPS_COLORS    8

enum {
  BBP_GESTURE_NONE = 0,
  BBP_GESTURE_UP,
  BBP_GESTURE_DOWN,
  BBP_GESTURE_LEFT,
  BBP_GESTURE_RIGHT
};

    int bbp_type(void);
    uint32_t bbp_caps(void);
    int bbp_init(void);
    void bbp_start(uint32_t iCaps);
    void bbp_stop(void);
    int bbp_getLight(void);
    int bbp_getGesture(void);
    int bbp_getColor(int *r, int *g, int *b, int *c);
    int bbp_getProximity(void);
    void bbp_setGestureSensitivity(uint8_t sensitivity);
    void bbp_setInterruptMode(int iMode, int iThreshLow, int iThreshHigh);
    void bbp_setLEDBoost(uint8_t boost);

#endif // __BB_PROXIMITY__
