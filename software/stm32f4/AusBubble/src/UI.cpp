/************************************************************************/
/* AusBubble:                                                           */
/* An open-source RF jammer designed to work in the 2.4 GHz Wi-Fi       */
/* frequency block.                                                     */
/*                                                                      */
/* ui.cpp                                                               */
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

#include "UI.h"

// Global variables
int gEnabled = 0;
fsmStates gWhereAmI = DisclaimerScreen;
int gPendingButton = ButtonNone;
float gPDETVoltage;
bool gInSetting = false;
bool gSplashActive = false;

int cursorPos = 0;

void safeString(const char *dataPointer, unsigned char row, unsigned char xPos)
{
    taskENTER_CRITICAL();
    showString(dataPointer, row, xPos);
    taskEXIT_CRITICAL();
}

void safeFont57(char ascii, unsigned char row, unsigned char xPos)
{
    taskENTER_CRITICAL();
    showFont57(ascii, row, xPos);
    taskEXIT_CRITICAL();
}

void centredString(const char *stringPointer, unsigned char line) {
    unsigned char len = strlen(stringPointer);

    if(len<22)
      safeString(stringPointer, line, 64-3*len);
}

void splash(const char* text)
{
    // Used to prevent splash being overwritten
    gSplashActive = true;

    int len = strlen(text);
    fillBlock(0x00, 3, 5, 60-3*len, 6*len+8);
    centredString(text, 4);
    DelayMS(1000);
    drawUI(gWhereAmI);

    gSplashActive = false;
}

void drawStep(int line, double stepSize)
{
    if(stepSize == STEP_1K)
        safeString("Step Size:   1 kHz", line, 14);
    else if(stepSize == STEP_10K)
        safeString("Step Size:  10 kHz", line, 14);
    else if(stepSize == STEP_25K)
        safeString("Step Size:  25 kHz", line, 14);
    else if(stepSize == STEP_50K)
        safeString("Step Size:  50 kHz", line, 14);
    else if(stepSize == STEP_100K)
        safeString("Step Size: 100 kHz", line, 14);
    else if(stepSize == STEP_250K)
        safeString("Step Size: 250 kHz", line, 14);
    else if(stepSize == STEP_500K)
        safeString("Step Size: 500 kHz", line, 14);
    else if(stepSize == STEP_1M)
        safeString("Step Size:   1 MHz", line, 14);
}

void drawAlgorithm(int line, ScanAlgorithms_t algo)
{
    switch(algo)
    {
        case ScanSawtooth:
            safeString("Algo:     Sawtooth", line, 14);
            break;
        case ScanTriangle:
            safeString("Algo:     Triangle", line, 14);
            break;
        case ScanRandom:
            safeString("Algo:       Random", line, 14);
            break;
    }
}

void drawDisclaimer()
{
    const char* disclaimer1 = "Use of this device is";
    const char* disclaimer2 = "subject to terms &   ";
    const char* disclaimer3 = "conditions located at";
    const char* disclaimer4 = "http://goo.gl/sxsv4  ";
    const char* disclaimer5 = "Enabling RF output   ";
    const char* disclaimer6 = "constitutes an (...) ";
    const char* disclaimer7 = "acceptance of these  ";
    const char* disclaimer8 = "terms & conditions.  ";

    centredString("Disclaimer", 1);

    if(cursorPos == 0)
    {
        centredString(disclaimer1, 2);
        centredString(disclaimer2, 3);
        centredString(disclaimer3, 4);
        centredString(disclaimer4, 5);
        centredString(disclaimer5, 6);
        centredString(disclaimer6, 7);
    }
    else if(cursorPos == 1)
    {
        centredString(disclaimer3, 2);
        centredString(disclaimer4, 3);
        centredString(disclaimer5, 4);
        centredString(disclaimer6, 5);
        centredString(disclaimer7, 6);
        centredString(disclaimer8, 7);
    }
}

void drawHomescreen()
{
    char menuText[21];

    centredString("Home Screen", 1);

    if(gEnabled)
        centredString("JAMMING ENABLED", 3);
    else
        centredString("JAMMING DISABLED", 3);

    snprintf(menuText, sizeof(menuText), "Battery: %d%%", 15);
    centredString(menuText, 4);
    snprintf(menuText, sizeof(menuText), "Batt Rem: %d:%d", 1, 13);
    centredString(menuText, 5);
    snprintf(menuText, sizeof(menuText), "PDET Voltage: %1.2fv", gPDETVoltage);
    centredString(menuText, 6);
}

void drawSynthMenu()
{
    char menuText[21];

    centredString("Synth Settings", 1);

    snprintf(menuText, sizeof(menuText), "Start: %4.2f MHz", gScanSettings.start);
    safeString(menuText, 2, 14);
    snprintf(menuText, sizeof(menuText), "Stop:  %4.2f MHz", gScanSettings.stop);
    safeString(menuText, 3, 14);
    drawAlgorithm(4, gScanSettings.algorithm);
    drawStep(5, gScanSettings.stepSize);

    if(gInSetting)
    {
        safeFont57(131, cursorPos+2, 6);
    } else {
        safeFont57(131, cursorPos+2, 4);
    }

}

void drawUI(fsmStates location)
{
    fillScreen(0x00);

    drawFrame();
    centredString(" AusBubble ", 0);
    safeFont57(127, 1, 0);
    safeFont57(131, 1, 128-6);

    if(!gSplashActive)
    {
        switch(location)
        {
            case DisclaimerScreen:
                drawDisclaimer();
                break;
            case HomeScreen:
                drawHomescreen();
                break;
            case SynthScreen:
                drawSynthMenu();
                break;
            default:
                // Looks like we're lost! Go back home.
                gWhereAmI = DisclaimerScreen;
                drawDisclaimer();
                break;
        }
    }
}

void toggleSetting(int index)
{
    // Toggle the inSetting flag
    gInSetting ^= 1;

    // Draw the >> icon in the appropriate spot
    if(gInSetting)
    {
        safeFont57(' ', index+2, 4);
        safeFont57(131, index+2, 6);
    }
    else
    {
        safeFont57(' ', index+2, 6);
        safeFont57(131, index+2, 4);
    }
}

void doDisclaimer(buttonStates action)
{
    switch(action)
    {
        case ButtonUp:
            cursorPos = 0;
            drawDisclaimer();
            break;
        case ButtonDown:
            cursorPos = 1;
            drawDisclaimer();
            break;
        case ButtonLeft:
            if(cursorPos == 1)
            {
                gWhereAmI = SynthScreen;
                cursorPos = 0;
                drawUI(gWhereAmI);
            }
            else
                splash("Please read");
            break;
        case ButtonRight:
        case ButtonEnter:
            if(cursorPos == 1)
            {
                gWhereAmI = HomeScreen;
                cursorPos = 0;
                drawUI(gWhereAmI);
            }
            else
                splash("Please read");
            break;
        default:
            break;
    }
}

void doSynthMin(buttonStates action)
{
    char menuText[21];

    switch(action)
    {
        case ButtonUp:
            if(gScanSettings.start < gScanSettings.stop)
            {
                gScanSettings.start += 1.0;
                snprintf(menuText, sizeof(menuText), "Start: %4.2f MHz", gScanSettings.start);
                safeString(menuText, 2, 14);
            }
            break;
        case ButtonDown:
            if(gScanSettings.start > MIN_FREQ_MHZ)
            {
                gScanSettings.start -= 1.0;
                snprintf(menuText, sizeof(menuText), "Start: %4.2f MHz", gScanSettings.start);
                safeString(menuText, 2, 14);
            }
            else
            {
                gScanSettings.start = MIN_FREQ_MHZ;
                snprintf(menuText, sizeof(menuText), "Start: %4.2f MHz", gScanSettings.start);
                safeString(menuText, 2, 14);
            }
            break;
        case ButtonEnter:
        case ButtonLeft:
        case ButtonRight:
        default:
            toggleSetting(cursorPos);
            break;
    }
}

void doSynthMax(buttonStates action)
{
    char menuText[21];

    switch(action)
    {
        case ButtonUp:
            if(gScanSettings.stop < MAX_FREQ_MHZ)
            {
                gScanSettings.stop += 1.0;
                snprintf(menuText, sizeof(menuText), "Stop:  %4.2f MHz", gScanSettings.stop);
                safeString(menuText, 3, 14);
            }
            else
            {
                gScanSettings.stop = MAX_FREQ_MHZ;
                snprintf(menuText, sizeof(menuText), "Stop:  %4.2f MHz", gScanSettings.stop);
                safeString(menuText, 3, 14);
            }
            break;
        case ButtonDown:
            if(gScanSettings.stop>gScanSettings.start)
            {
                gScanSettings.stop -= 1.0;
                snprintf(menuText, sizeof(menuText), "Stop:  %4.2f MHz", gScanSettings.stop);
                safeString(menuText, 3, 14);
            }
            break;
        case ButtonEnter:
        case ButtonLeft:
        case ButtonRight:
        default:
            toggleSetting(cursorPos);
            break;
    }
}

void doAlgorithm(buttonStates action)
{
    switch(action)
    {
        case ButtonEnter:
        case ButtonRight:
        case ButtonLeft:
            toggleSetting(cursorPos);
            break;
        case ButtonUp:
            switch(gScanSettings.algorithm)
            {
                case ScanSawtooth:
                    gScanSettings.algorithm = ScanRandom;
                    drawAlgorithm(4,gScanSettings.algorithm);
                    break;
                case ScanTriangle:
                    gScanSettings.algorithm = ScanSawtooth;
                    drawAlgorithm(4,gScanSettings.algorithm);
                    break;
                case ScanRandom:
                    gScanSettings.algorithm = ScanTriangle;
                    drawAlgorithm(4,gScanSettings.algorithm);
                    break;
            }
            break;
        case ButtonDown:
            switch(gScanSettings.algorithm)
            {
                case ScanSawtooth:
                    drawAlgorithm(4,gScanSettings.algorithm);
                    gScanSettings.algorithm = ScanTriangle;
                    break;
                case ScanTriangle:
                    drawAlgorithm(4,gScanSettings.algorithm);
                    gScanSettings.algorithm = ScanRandom;
                    break;
                case ScanRandom:
                    drawAlgorithm(4,gScanSettings.algorithm);
                    gScanSettings.algorithm = ScanSawtooth;
                    break;
            }
            break;
        default:
            break;
    }
}

void doStepSize(buttonStates action)
{
    switch(action)
    {
        case ButtonEnter:
        case ButtonRight:
        case ButtonLeft:
            toggleSetting(cursorPos);
            break;
        case ButtonUp:
            // Step forwards through the cycle
            if(gScanSettings.stepSize == STEP_1K)
                gScanSettings.stepSize = STEP_10K;
            else if(gScanSettings.stepSize == STEP_10K)
                gScanSettings.stepSize = STEP_25K;
            else if(gScanSettings.stepSize == STEP_25K)
                gScanSettings.stepSize = STEP_50K;
            else if(gScanSettings.stepSize == STEP_50K)
                gScanSettings.stepSize = STEP_100K;
            else if(gScanSettings.stepSize == STEP_100K)
                gScanSettings.stepSize = STEP_250K;
            else if(gScanSettings.stepSize == STEP_250K)
                gScanSettings.stepSize = STEP_500K;
            else if(gScanSettings.stepSize == STEP_500K)
                gScanSettings.stepSize = STEP_1M;
            else if(gScanSettings.stepSize == STEP_1M)
                gScanSettings.stepSize = STEP_1K;

            drawStep(5,gScanSettings.stepSize);
            break;
        case ButtonDown:
            // Step backwards through the cycle
            if(gScanSettings.stepSize == STEP_1M)
                gScanSettings.stepSize = STEP_500K;
            else if(gScanSettings.stepSize == STEP_500K)
                gScanSettings.stepSize = STEP_250K;
            else if(gScanSettings.stepSize == STEP_250K)
                gScanSettings.stepSize = STEP_100K;
            else if(gScanSettings.stepSize == STEP_100K)
                gScanSettings.stepSize = STEP_50K;
            else if(gScanSettings.stepSize == STEP_50K)
                gScanSettings.stepSize = STEP_25K;
            else if(gScanSettings.stepSize == STEP_25K)
                gScanSettings.stepSize = STEP_10K;
            else if(gScanSettings.stepSize == STEP_10K)
                gScanSettings.stepSize = STEP_1K;
            else if(gScanSettings.stepSize == STEP_1K)
                gScanSettings.stepSize = STEP_1M;

            drawStep(5,gScanSettings.stepSize);
            break;
        default:
            break;
    }
}

void doSynthMenu(buttonStates action)
{
    // We're in a setting, let's handle the key press depending on which one
    if(gInSetting)
    {
        switch(cursorPos)
        {
            case 0:
                doSynthMin(action);
                break;
            case 1:
                doSynthMax(action);
                break;
            case 2:
                doAlgorithm(action);
                break;
            case 3:
                doStepSize(action);
                break;
            default:
                break;
        }
        return;
    }

    // We're not in a setting, so let's move around the menus
    switch(action)
    {
        case ButtonUp:
            if(cursorPos>0)
            {
                safeFont57(' ', cursorPos+2, 4);
                cursorPos--;
                safeFont57(131, cursorPos+2, 4);
            }
            break;
        case ButtonDown:
            if(cursorPos<3)
            {
                safeFont57(' ', cursorPos+2, 4);
                cursorPos++;
                safeFont57(131, cursorPos+2, 4);
            }
            break;
        case ButtonLeft:
            gWhereAmI = HomeScreen;
            cursorPos = 0;
            drawUI(gWhereAmI);
            break;
        case ButtonRight:
            gWhereAmI = HomeScreen;
            cursorPos = 0;
            drawUI(gWhereAmI);
            break;
        case ButtonEnter:
            switch(cursorPos)
            {
                case 0:
                case 1:
                case 2:
                case 3:
                    toggleSetting(cursorPos);
                    break;
            }
            break;
        default:
            break;
    }
}

void doHomeScreen(buttonStates action)
{
    switch(action)
    {
        case ButtonLeft:
            gWhereAmI = SynthScreen;
            cursorPos = 0;
            drawUI(gWhereAmI);
            break;
        case ButtonRight:
            gWhereAmI = SynthScreen;
            cursorPos = 0;
            drawUI(gWhereAmI);
            break;
        default:
            break;
    }
}

void doMenu(int buttons)
{
    buttonStates action;
    // The if statements below dictate the button priority if multiple are
    // held down - highest priority at the top
    if(buttons & ButtonLeft)
        action = ButtonLeft;
    else if(buttons & ButtonRight)
        action = ButtonRight;
    else if(buttons & ButtonUp)
        action = ButtonUp;
    else if(buttons & ButtonDown)
        action = ButtonDown;
    else if(buttons & ButtonEnter)
        action = ButtonEnter;
    else
        action = ButtonNone;

    switch(gWhereAmI)
    {
        default:
        case DisclaimerScreen:
            doDisclaimer(action);
            break;
        case SynthScreen:
            doSynthMenu(action);
            break;
        case HomeScreen:
            doHomeScreen(action);
            break;
    }
}
