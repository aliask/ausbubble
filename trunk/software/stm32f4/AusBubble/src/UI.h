/************************************************************************/
/* AusBubble:                                                           */
/* An open-source RF jammer designed to work in the 2.4 GHz Wi-Fi       */
/* frequency block.                                                     */
/*                                                                      */
/* ui.h                                                                 */
/*                                                                      */
/* Will Robertson <aliask@gmail.com>                                    */
/* Nick D'Ademo <nickdademo@gmail.com>                                  */
/*                                                                      */
/* Copyright (c) 2012 Will Robertson, Nick D'Ademo                      */
/*                                                                      */
/* Permission is hereby granted, free of charge, to any person          */
/* obtaining a copy of this software and associated documentation       */
/* files (the "Software"), to deal in the Software without restriction, */
/* including without limitation the rights to use, copy, modify, merge, */
/* publish, distribute, sublicense, and/or sell copies of the Software, */
/* and to permit persons to whom the Software is furnished to do so,    */
/* subject to the following conditions:                                 */
/*                                                                      */
/* The above copyright notice and this permission notice shall be       */
/* included in all copies or substantial portions of the Software.      */
/*                                                                      */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,      */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF   */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND                */
/* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS  */
/* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   */
/* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN    */
/* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE     */
/* SOFTWARE.                                                            */
/*                                                                      */
/************************************************************************/

#ifndef _UI_H
#define _UI_H

#include "Includes.h"

#include "OLED.h"
#include "ScanAlgorithms.h"

// Used for information items handling
#define DISP_MAX 2

/* Enumerations */
typedef enum {
    ButtonNone =     0,
    ButtonUp =    1<<0,
    ButtonDown =  1<<1,
    ButtonLeft =  1<<2,
    ButtonRight = 1<<3,
    ButtonEnter = 1<<4
} buttonStates;

typedef enum {
    DisclaimerScreen = 0,
    HomeScreen,
    SynthScreen
} fsmStates;

/* Global Variables */
extern int gEnabled;
extern fsmStates gWhereAmI;
extern int gPendingButton;
extern float gPDETVoltage;
extern bool gInSetting;
extern bool gSplashActive;

/* Function prototypes */
void drawUI(fsmStates location);
void centredString(const char *stringPointer);
void safeString(const char *dataPointer, unsigned char row, unsigned char xPos);
void splash(const char* text);
void doMenu(int buttons);
void drawHomescreen(void);
// FreeRTOS millisecond delay function
void DelayMS(uint32_t t);

#endif
