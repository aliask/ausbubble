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

/* Initialize static members */
int UI::cursorPos = 0;
fsmStates UI::currentState = DisclaimerScreen;
bool UI::isInSetting = false;
bool UI::isSplashActive = false;
Stats UI::stats;

void UI::updateStatsData(Stats input)
{
    /* System */
    UI::stats.heartbeat = input.heartbeat;
    /* Battery */
    UI::stats.batteryLevel = input.batteryLevel;
    UI::stats.isCharging = input.isCharging;
    /* Jammer */
    UI::stats.isJamming = input.isJamming;
    UI::stats.isPLLLocked = input.isPLLLocked;
}

void UI::drawHeader(void)
{
    SSD1306_OLED::fillBlock(0x00, 0, 0, 0, 128);

    centredString("AusBubble", 0);

    if(stats.isJamming)
        safeFont57(138, 0, 5);

    if(stats.isPLLLocked)
        safeFont57(139, 0, 13);
    else
        safeFont57(140, 0, 13);

    if(stats.heartbeat)
        safeFont57(141, 0, 20);

    SSD1306_OLED::drawBatt(stats.batteryLevel,97,0);
}

void UI::draw(fsmStates state)
{
    /* Save location */
    currentState = state;

    if(!isSplashActive)
    {
        /* Clear entire screen */
        SSD1306_OLED::fillScreen(0x00);

        /* Draw header */
        UI::drawHeader();

        switch(state)
        {
            case DisclaimerScreen:
                drawDisclaimerScreen();
                break;
            case HomeScreen:
                /* Draw RIGHT arrow */
                safeFont57(131, 1, 128-6);
                /* Draw screen */
                drawHomeScreen();
                break;
            case JammerScreen:
                /* Draw LEFT arrow */
                safeFont57(127, 1, 0);
                /* Draw screen */
                drawJammerScreen();
                break;
            default:
                /* Looks like we're lost! Set state to Disclaimer screen */
                currentState = DisclaimerScreen;
                /* Draw screen */
                drawDisclaimerScreen();
                break;
        }
    }
}

void UI::splash(const char* text, int duration_ms)
{
    /* Set flag to prevent splash being overwritten */
    isSplashActive = true;

    /* Show splash */
    int len = strlen(text);
    SSD1306_OLED::fillBlock(0x00, 3, 5, 60-3*len, 6*len+8);
    centredString(text, 4);
    DelayMS(duration_ms);

    /* Set flag */
    isSplashActive = false;

    /* Redraw UI */
    UI::draw(currentState);
}

void UI::doMenu(int buttons)
{
    buttonStates action;

    /* The if statements below dictate the button priority if multiple are
    held down - highest priority at the top */
    if(buttons & ButtonLeft)
        action = ButtonLeft;
    else if(buttons & ButtonRight)
        action = ButtonRight;
    else if(buttons & ButtonUp)
        action = ButtonUp;
    else if(buttons & ButtonDown)
        action = ButtonDown;
    else if(buttons & ButtonSelect)
        action = ButtonSelect;
    else
        action = ButtonNone;

    /* Action dependent on UI state */
    switch(currentState)
    {
        case DisclaimerScreen:
            doDisclaimer(action);
            break;
        case JammerScreen:
            doJammerScreen(action);
            break;
        case HomeScreen:
            doHomeScreen(action);
            break;
        default:
            break;
    }
}

void UI::safeString(const char *dataPointer, unsigned char row, unsigned char xPos)
{
    taskENTER_CRITICAL();
    SSD1306_OLED::showString(dataPointer, row, xPos);
    taskEXIT_CRITICAL();
}

void UI::safeFont57(char ascii, unsigned char row, unsigned char xPos)
{
    taskENTER_CRITICAL();
    SSD1306_OLED::showFont57(ascii, row, xPos);
    taskEXIT_CRITICAL();
}

void UI::centredString(const char *stringPointer, unsigned char line)
{
    unsigned char len = strlen(stringPointer);

    if(len < 22)
        safeString(stringPointer, line, 64-3*len);
}

void UI::toggleSetting(int index)
{
    // Toggle flag
    isInSetting ^= true;

    // Draw the >> icon in the appropriate location
    if(isInSetting)
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

void UI::setToggle(bool state)
{
    isInSetting = state;
}

void UI::drawJammerStep(int line, double stepSize)
{
    if(stepSize == STEP_1K_HZ)
        safeString("Step Size:   1 kHz", line, 14);
    else if(stepSize == STEP_10K_HZ)
        safeString("Step Size:  10 kHz", line, 14);
    else if(stepSize == STEP_25K_HZ)
        safeString("Step Size:  25 kHz", line, 14);
    else if(stepSize == STEP_50K_HZ)
        safeString("Step Size:  50 kHz", line, 14);
    else if(stepSize == STEP_100K_HZ)
        safeString("Step Size: 100 kHz", line, 14);
    else if(stepSize == STEP_250K_HZ)
        safeString("Step Size: 250 kHz", line, 14);
    else if(stepSize == STEP_500K_HZ)
        safeString("Step Size: 500 kHz", line, 14);
    else if(stepSize == STEP_1M_HZ)
        safeString("Step Size:   1 MHz", line, 14);
    else if(stepSize == STEP_NONE)
        safeString("Step Size:    None", line, 14);
}

void UI::drawJammerAlgorithm(int line, ScanAlgorithms_t algorithm)
{
    switch(algorithm)
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

void UI::drawDisclaimerScreen()
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

void UI::drawHomeScreen()
{
    char menuText[21];

    centredString("Home", 1);

    snprintf(menuText, sizeof(menuText), "Battery:  %d%%", 15);
    safeString(menuText, 3, 14);
    snprintf(menuText, sizeof(menuText), "Batt Rem: %d:%d", 1, 13);
    safeString(menuText, 4, 14);
    snprintf(menuText, sizeof(menuText), "PDET:     %1.2fv", gPDETVoltage);
    safeString(menuText, 5, 14);
    snprintf(menuText, sizeof(menuText), "VBAT:     %1.2fv", VBATVoltage);
    safeString(menuText, 6, 14);
}

void UI::drawJammerScreen()
{
    char menuText[21];

    centredString("Jammer Settings", 1);

    snprintf(menuText, sizeof(menuText), "Start: %4.2f MHz", (float) (Jammer::settings.start / 1000000.0f));
    safeString(menuText, 2, 14);

    snprintf(menuText, sizeof(menuText), "Stop:  %4.2f MHz", (float) (Jammer::settings.stop / 1000000.0f));
    safeString(menuText, 3, 14);

    drawJammerAlgorithm(4, Jammer::settings.algorithm);
    drawJammerStep(5, Jammer::settings.stepSize);

    snprintf(menuText, sizeof(menuText), "Rate:      %4d Hz", Jammer::settings.rate);
    safeString(menuText, 6, 14);

    if(isInSetting)
        safeFont57(131, cursorPos+2, 6);
    else
        safeFont57(131, cursorPos+2, 4);
}

void UI::doDisclaimer(buttonStates action)
{
    switch(action)
    {
        case ButtonUp:
            cursorPos = 0;
            drawDisclaimerScreen();
            break;
        case ButtonDown:
            cursorPos = 1;
            drawDisclaimerScreen();
            break;
        case ButtonLeft:
            break;
        case ButtonRight:
            break;
        case ButtonSelect:
            if(cursorPos == 1)
            {
                cursorPos = 0;
                UI::draw(HomeScreen);
            }
            else
                splash("Please read", 1000);
            break;
        default:
            break;
    }
}

void UI::doJammerFreqStart(buttonStates action)
{
    char menuText[21];

    switch(action)
    {
        case ButtonUp:
            if((Jammer::settings.start + FREQ_STEP_HZ) <= Jammer::settings.stop)
            {
                Jammer::settings.start += FREQ_STEP_HZ;
                snprintf(menuText, sizeof(menuText), "Start: %4.2f MHz", Jammer::settings.start / 1000000.0f);
                safeString(menuText, 2, 14);
                showChannelHint_Start(Jammer::settings.start, 500);
            }
            break;
        case ButtonDown:
            if((Jammer::settings.start - FREQ_STEP_HZ) >= MIN_FREQ_HZ)
            {
                Jammer::settings.start -= FREQ_STEP_HZ;
                snprintf(menuText, sizeof(menuText), "Start: %4.2f MHz", Jammer::settings.start / 1000000.0f);
                safeString(menuText, 2, 14);
                showChannelHint_Start(Jammer::settings.start, 500);
            }
            else
            {
                Jammer::settings.start = MIN_FREQ_HZ;
                snprintf(menuText, sizeof(menuText), "Start: %4.2f MHz", Jammer::settings.start / 1000000.0f);
                safeString(menuText, 2, 14);
                showChannelHint_Start(Jammer::settings.start, 500);
            }
            break;
        case ButtonSelect:
        case ButtonLeft:
        case ButtonRight:
        default:
            toggleSetting(cursorPos);
            break;
    }
}

void UI::doJammerFreqStop(buttonStates action)
{
    char menuText[21];

    switch(action)
    {
        case ButtonUp:
            if((Jammer::settings.stop + FREQ_STEP_HZ) <= MAX_FREQ_HZ)
            {
                Jammer::settings.stop += FREQ_STEP_HZ;
                snprintf(menuText, sizeof(menuText), "Stop:  %4.2f MHz", Jammer::settings.stop / 1000000.0f);
                safeString(menuText, 3, 14);
                showChannelHint_Stop(Jammer::settings.stop, 500);
            }
            else
            {
                Jammer::settings.stop = MAX_FREQ_HZ;
                snprintf(menuText, sizeof(menuText), "Stop:  %4.2f MHz", Jammer::settings.stop / 1000000.0f);
                safeString(menuText, 3, 14);
                showChannelHint_Stop(Jammer::settings.stop, 500);
            }
            break;
        case ButtonDown:
            if((Jammer::settings.stop - FREQ_STEP_HZ) >= Jammer::settings.start)
            {
                Jammer::settings.stop -= FREQ_STEP_HZ;
                snprintf(menuText, sizeof(menuText), "Stop:  %4.2f MHz", Jammer::settings.stop / 1000000.0f);
                safeString(menuText, 3, 14);
                showChannelHint_Stop(Jammer::settings.stop, 500);
            }
            break;
        case ButtonSelect:
        case ButtonLeft:
        case ButtonRight:
        default:
            toggleSetting(cursorPos);
            break;
    }
}

void UI::doJammerRate(buttonStates action)
{
    char menuText[21];

    switch(action)
    {
        case ButtonUp:
            if((Jammer::settings.rate + RATE_STEP_HZ) <= MAX_RATE_HZ)
            {
                Jammer::settings.rate += RATE_STEP_HZ;
                snprintf(menuText, sizeof(menuText), "Rate:      %4d Hz", Jammer::settings.rate);
                safeString(menuText, 6, 14);
            }
            else
            {
                Jammer::settings.rate = MAX_RATE_HZ;
                snprintf(menuText, sizeof(menuText), "Rate:      %4d Hz", Jammer::settings.rate);
                safeString(menuText, 6, 14);
            }
            break;
        case ButtonDown:
            if((Jammer::settings.rate - RATE_STEP_HZ) >= MIN_RATE_HZ)
            {
                Jammer::settings.rate -= RATE_STEP_HZ;
                snprintf(menuText, sizeof(menuText), "Rate:      %4d Hz", Jammer::settings.rate);
                safeString(menuText, 6, 14);
            }
            break;
        case ButtonSelect:
        case ButtonLeft:
        case ButtonRight:
        default:
            toggleSetting(cursorPos);
            break;
    }
}

void UI::doJammerAlgorithm(buttonStates action)
{
    switch(action)
    {
        case ButtonSelect:
        case ButtonRight:
        case ButtonLeft:
            toggleSetting(cursorPos);
            break;
        case ButtonUp:
            switch(Jammer::settings.algorithm)
            {
                case ScanSawtooth:
                    Jammer::settings.algorithm = ScanRandom;
                    drawJammerAlgorithm(4,Jammer::settings.algorithm);
                    drawJammerStep(5, STEP_NONE);
                    break;
                case ScanTriangle:
                    Jammer::settings.algorithm = ScanSawtooth;
                    drawJammerAlgorithm(4,Jammer::settings.algorithm);
                    drawJammerStep(5, SCAN_SETTINGS_DEFAULT_STEPSIZE);
                    break;
                case ScanRandom:
                    Jammer::settings.algorithm = ScanTriangle;
                    drawJammerAlgorithm(4,Jammer::settings.algorithm);
                    drawJammerStep(5, SCAN_SETTINGS_DEFAULT_STEPSIZE);
                    break;
            }
            break;
        case ButtonDown:
            switch(Jammer::settings.algorithm)
            {
                case ScanSawtooth:
                    Jammer::settings.algorithm = ScanTriangle;
                    drawJammerAlgorithm(4,Jammer::settings.algorithm);
                    drawJammerStep(5, SCAN_SETTINGS_DEFAULT_STEPSIZE);
                    break;
                case ScanTriangle:
                    Jammer::settings.algorithm = ScanRandom;
                    drawJammerAlgorithm(4,Jammer::settings.algorithm);
                    drawJammerStep(5, STEP_NONE);
                    break;
                case ScanRandom:
                    Jammer::settings.algorithm = ScanSawtooth;
                    drawJammerAlgorithm(4,Jammer::settings.algorithm);
                    drawJammerStep(5, SCAN_SETTINGS_DEFAULT_STEPSIZE);
                    break;
            }
            break;
        default:
            break;
    }
}

void UI::doJammerStepSize(buttonStates action)
{
    switch(action)
    {
        case ButtonSelect:
        case ButtonRight:
        case ButtonLeft:
            toggleSetting(cursorPos);
            break;
        case ButtonUp:
            /* Only toggle step size if algorithm is not set to RANDOM */
            if(Jammer::settings.algorithm != ScanRandom)
            {
                // Step forwards through the cycle
                if(Jammer::settings.stepSize == STEP_1K_HZ)
                    Jammer::settings.stepSize = STEP_10K_HZ;
                else if(Jammer::settings.stepSize == STEP_10K_HZ)
                    Jammer::settings.stepSize = STEP_25K_HZ;
                else if(Jammer::settings.stepSize == STEP_25K_HZ)
                    Jammer::settings.stepSize = STEP_50K_HZ;
                else if(Jammer::settings.stepSize == STEP_50K_HZ)
                    Jammer::settings.stepSize = STEP_100K_HZ;
                else if(Jammer::settings.stepSize == STEP_100K_HZ)
                    Jammer::settings.stepSize = STEP_250K_HZ;
                else if(Jammer::settings.stepSize == STEP_250K_HZ)
                    Jammer::settings.stepSize = STEP_500K_HZ;
                else if(Jammer::settings.stepSize == STEP_500K_HZ)
                    Jammer::settings.stepSize = STEP_1M_HZ;
                drawJammerStep(5,Jammer::settings.stepSize);
            }
            break;
        case ButtonDown:
            /* Only toggle step size if algorithm is not set to RANDOM */
            if(Jammer::settings.algorithm != ScanRandom)
            {
                // Step backwards through the cycle
                if(Jammer::settings.stepSize == STEP_1M_HZ)
                    Jammer::settings.stepSize = STEP_500K_HZ;
                else if(Jammer::settings.stepSize == STEP_500K_HZ)
                    Jammer::settings.stepSize = STEP_250K_HZ;
                else if(Jammer::settings.stepSize == STEP_250K_HZ)
                    Jammer::settings.stepSize = STEP_100K_HZ;
                else if(Jammer::settings.stepSize == STEP_100K_HZ)
                    Jammer::settings.stepSize = STEP_50K_HZ;
                else if(Jammer::settings.stepSize == STEP_50K_HZ)
                    Jammer::settings.stepSize = STEP_25K_HZ;
                else if(Jammer::settings.stepSize == STEP_25K_HZ)
                    Jammer::settings.stepSize = STEP_10K_HZ;
                else if(Jammer::settings.stepSize == STEP_10K_HZ)
                    Jammer::settings.stepSize = STEP_1K_HZ;
                drawJammerStep(5,Jammer::settings.stepSize);
            }
            break;
        default:
            break;
    }
}

void UI::doJammerScreen(buttonStates action)
{
    // We're in a setting, let's handle the key press depending on which one
    if(isInSetting)
    {
        switch(cursorPos)
        {
            case 0:
                doJammerFreqStart(action);
                break;
            case 1:
                doJammerFreqStop(action);
                break;
            case 2:
                doJammerAlgorithm(action);
                break;
            case 3:
                doJammerStepSize(action);
                break;
            case 4:
                doJammerRate(action);
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
            if(cursorPos<4)
            {
                safeFont57(' ', cursorPos+2, 4);
                cursorPos++;
                safeFont57(131, cursorPos+2, 4);
            }
            break;
        case ButtonLeft:
            cursorPos = 0;
            UI::draw(HomeScreen);
            break;
        case ButtonRight:
            break;
        case ButtonSelect:
            switch(cursorPos)
            {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                    toggleSetting(cursorPos);
                    break;
            }
            break;
        default:
            break;
    }
}

void UI::doHomeScreen(buttonStates action)
{
    switch(action)
    {
        case ButtonLeft:
            break;
        case ButtonRight:
            cursorPos = 0;
            UI::draw(JammerScreen);
            break;
        default:
            break;
    }
}

void UI::showChannelHint_Start(uint64_t freq_Hz, int duration_ms)
{
    switch(freq_Hz)
    {
        // CH1
        case 2401000000:
            UI::splash("Ch 1 (b: 22M BW)", duration_ms);
            break;
        case 2402000000:
            UI::splash("Ch 1 (g/n: 20M BW)", duration_ms);
            break;
        // CH2
        case 2404000000:
            UI::splash("Ch 2 (b: 22M BW)", duration_ms);
            break;
        case 2405000000:
            UI::splash("Ch 2 (g/n: 20M BW)", duration_ms);
            break;
        // CH3
        case 2411000000:
            UI::splash("Ch 3 (b: 22M BW)", duration_ms);
            break;
        case 2412000000:
            UI::splash("Ch 3 (g/n: 20M BW)", duration_ms);
            break;
        // CH4
        case 2416000000:
            UI::splash("Ch 4 (b: 22M BW)", duration_ms);
            break;
        case 2417000000:
            UI::splash("Ch 4 (g/n: 20M BW)", duration_ms);
            break;
        // CH5
        case 2421000000:
            UI::splash("Ch 5 (b: 22M BW)", duration_ms);
            break;
        case 2422000000:
            UI::splash("Ch 5 (g/n: 20M BW)", duration_ms);
            break;
        // CH6
        case 2426000000:
            UI::splash("Ch 6 (b: 22M BW)", duration_ms);
            break;
        case 2427000000:
            UI::splash("Ch 6 (g/n: 20M BW)", duration_ms);
            break;
        // CH7
        case 2431000000:
            UI::splash("Ch 7 (b: 22M BW)", duration_ms);
            break;
        case 2432000000:
            UI::splash("Ch 7 (g/n: 20M BW)", duration_ms);
            break;
        // CH8
        case 2436000000:
            UI::splash("Ch 8 (b: 22M BW)", duration_ms);
            break;
        case 2437000000:
            UI::splash("Ch 8 (g/n: 20M BW)", duration_ms);
            break;
        // CH9
        case 2441000000:
            UI::splash("Ch 9 (b: 22M BW)", duration_ms);
            break;
        case 2442000000:
            UI::splash("Ch 9 (g/n: 20M BW)", duration_ms);
            break;
        // CH10
        case 2446000000:
            UI::splash("Ch 10 (b: 22M BW)", duration_ms);
            break;
        case 2447000000:
            UI::splash("Ch 10 (g/n: 20M BW)", duration_ms);
            break;
        // CH11
        case 2451000000:
            UI::splash("Ch 11 (b: 22M BW)", duration_ms);
            break;
        case 2452000000:
            UI::splash("Ch 11 (g/n: 20M BW)", duration_ms);
            break;
        // CH12
        case 2456000000:
            UI::splash("Ch 12 (b: 22M BW)", duration_ms);
            break;
        case 2457000000:
            UI::splash("Ch 12 (g/n: 20M BW)", duration_ms);
            break;
        // CH13
        case 2461000000:
            UI::splash("Ch 13 (b: 22M BW)", duration_ms);
            break;
        case 2462000000:
            UI::splash("Ch 13 (g/n: 20M BW)", duration_ms);
            break;
        // CH14
        case 2473000000:
            UI::splash("Ch 14 (b: 22M BW)", duration_ms);
            break;
        case 2474000000:
            UI::splash("Ch 14 (g/n: 20M BW)", duration_ms);
            break;
    }
}

void UI::showChannelHint_Stop(uint64_t freq_Hz, int duration_ms)
{
    switch(freq_Hz)
    {
        // CH1
        case 2423000000:
            UI::splash("Ch 1 (b: 22M BW)", duration_ms);
            break;
        case 2422000000:
            UI::splash("Ch 1 (g/n: 20M BW)", duration_ms);
            break;
        // CH2
        case 2428000000:
            UI::splash("Ch 2 (b: 22M BW)", duration_ms);
            break;
        case 2427000000:
            UI::splash("Ch 2 (g/n: 20M BW)", duration_ms);
            break;
        // CH3
        case 2433000000:
            UI::splash("Ch 3 (b: 22M BW)", duration_ms);
            break;
        case 2432000000:
            UI::splash("Ch 3 (g/n: 20M BW)", duration_ms);
            break;
        // CH4
        case 2438000000:
            UI::splash("Ch 4 (b: 22M BW)", duration_ms);
            break;
        case 2437000000:
            UI::splash("Ch 4 (g/n: 20M BW)", duration_ms);
            break;
        // CH5
        case 2443000000:
            UI::splash("Ch 5 (b: 22M BW)", duration_ms);
            break;
        case 2442000000:
            UI::splash("Ch 5 (g/n: 20M BW)", duration_ms);
            break;
        // CH6
        case 2448000000:
            UI::splash("Ch 6 (b: 22M BW)", duration_ms);
            break;
        case 2447000000:
            UI::splash("Ch 6 (g/n: 20M BW)", duration_ms);
            break;
        // CH7
        case 2453000000:
            UI::splash("Ch 7 (b: 22M BW)", duration_ms);
            break;
        case 2452000000:
            UI::splash("Ch 7 (g/n: 20M BW)", duration_ms);
            break;
        // CH8
        case 2458000000:
            UI::splash("Ch 8 (b: 22M BW)", duration_ms);
            break;
        case 2457000000:
            UI::splash("Ch 8 (g/n: 20M BW)", duration_ms);
            break;
        // CH9
        case 2463000000:
            UI::splash("Ch 9 (b: 22M BW)", duration_ms);
            break;
        case 2462000000:
            UI::splash("Ch 9 (g/n: 20M BW)", duration_ms);
            break;
        // CH10
        case 2468000000:
            UI::splash("Ch 10 (b: 22M BW)", duration_ms);
            break;
        case 2467000000:
            UI::splash("Ch 10 (g/n: 20M BW)", duration_ms);
            break;
        // CH11
        case 2473000000:
            UI::splash("Ch 11 (b: 22M BW)", duration_ms);
            break;
        case 2472000000:
            UI::splash("Ch 11 (g/n: 20M BW)", duration_ms);
            break;
        // CH12
        case 2478000000:
            UI::splash("Ch 12 (b: 22M BW)", duration_ms);
            break;
        case 2477000000:
            UI::splash("Ch 12 (g/n: 20M BW)", duration_ms);
            break;
        // CH13
        case 2483000000:
            UI::splash("Ch 13 (b: 22M BW)", duration_ms);
            break;
        case 2482000000:
            UI::splash("Ch 13 (g/n: 20M BW)", duration_ms);
            break;
        // CH14
        case 2495000000:
            UI::splash("Ch 14 (b: 22M BW)", duration_ms);
            break;
        case 2494000000:
            UI::splash("Ch 14 (g/n: 20M BW)", duration_ms);
            break;
    }
}
