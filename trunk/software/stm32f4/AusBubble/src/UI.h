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

#ifndef UI_H
#define UI_H

#include "Includes.h"
#include "Jammer.h"
/* Peripheral libraries */
#include "SSD1306_OLED.h"
#include "RFMD_IntSynth.h"

/* Enumerations */
typedef enum {
    ButtonNone      = 0,
    ButtonUp        = 1 << 0,
    ButtonDown      = 1 << 1,
    ButtonLeft      = 1 << 2,
    ButtonRight     = 1 << 3,
    ButtonSelect    = 1 << 4
} buttonStates;

typedef enum {
    Disclaimer = 0,
    Home,
    JammerGeneral,
    JammerFrequency
} screenStates;

class UI
{
    public:
        static void drawScreen(screenStates screen);
        static void drawHeader(void);
        static void updateStatsData(Stats input);
        static void splash(const char* text, int duration_ms);
        static void doMenu(int buttons);
        static void setToggle(bool state);
        static screenStates currentScreen;
    private:
        static void safeString(const char *dataPointer, unsigned char row, unsigned char xPos);
        static void safeFont57(char ascii, unsigned char row, unsigned char xPos);
        static void centredString(const char *stringPointer, unsigned char line);
        static void toggleSetting(int index);
        static void drawScreen_Disclaimer(void);
        static void drawScreen_Home(void);
        static void drawScreen_JammerGeneral(void);
        static void drawScreen_JammerFrequency(void);
        static void drawItem_JammerFMOD(int line, FMODSetting_t fmod);
        static void drawItem_JammerAlgorithm(int line, JamAlgorithms_t algorithm);
        static void drawItem_JammerWaitForPLLLock(int line, bool wait);
        static void drawItem_JammerStep(int line, double stepSize);
        static void doItem_JammerFMOD(buttonStates action);
        static void doItem_JammerAlgorithm(buttonStates action);
        static void doItem_JammerRate(buttonStates action);
        static void doItem_WaitForPLLLock(buttonStates action);
        static void doItem_JammerFreqCentre(buttonStates action);
        static void doItem_JammerBW(buttonStates action);
        static void doItem_JammerFreqStart(buttonStates action);
        static void doItem_JammerFreqStop(buttonStates action);
        static void doItem_JammerStepSize(buttonStates action);
        static void doScreen_Disclaimer(buttonStates action);
        static void doScreen_JammerGeneral(buttonStates action);
        static void doScreen_JammerFrequency(buttonStates action);
        static void doScreen_Home(buttonStates action);
        static uint64_t roundToMultiple(uint64_t numToRound, uint64_t multiple, bool doRoundUp);
        static int cursorPos;
        static bool isInSetting;
        static bool isSplashActive;
        static Stats stats;
};

#endif
