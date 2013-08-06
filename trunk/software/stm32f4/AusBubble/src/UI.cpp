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
screenStates UI::currentScreen = Disclaimer;
bool UI::isInSetting = false;
bool UI::isSplashActive = false;
Stats UI::stats;

void UI::updateStatsData(Stats input)
{
    /* System */
    stats.heartbeat = input.heartbeat;
    stats.VBAT_V = input.VBAT_V;
    stats.OnChipTS_T_degC = input.OnChipTS_T_degC;
    stats.isUSBConnected = input.isUSBConnected;
    /* Battery */
    stats.batteryLevel = input.batteryLevel;
    stats.isCharging = input.isCharging;
    /* Jammer */
    stats.isJamming = input.isJamming;
    stats.isPLLLocked = input.isPLLLocked;
    stats.PDET_V = input.PDET_V;
    stats.Pout_dBm = input.Pout_dBm;
    stats.RFAmpTS_T_degC = input.RFAmpTS_T_degC;
}

void UI::drawHeader(void)
{
    SSD1306_OLED::fillBlock(0x00, 0, 0, 0, 128);

    centredString("AusBubble", 0);

    if(stats.isJamming)
        safeFont57(138, 0, 2);

    if(stats.isPLLLocked)
        safeFont57(139, 0, 10);
    else
        safeFont57(140, 0, 10);

    if(stats.heartbeat)
        safeFont57(141, 0, 18);

    if(stats.isUSBConnected)
        safeFont57(142, 0, 26);

    SSD1306_OLED::drawBatt(stats.batteryLevel, 97, 0);
}

void UI::drawScreen(screenStates screen)
{
    /* Save location */
    currentScreen = screen;

    if(!isSplashActive)
    {
        /* Clear entire screen */
        SSD1306_OLED::fillScreen(0x00);

        /* Draw header */
        UI::drawHeader();

        switch(screen)
        {
            case Disclaimer:
                drawScreen_Disclaimer();
                break;
            case Home:
                /* Draw RIGHT arrow */
                safeFont57(131, 1, 128-6);
                /* Draw screen */
                drawScreen_Home();
                break;
            case JammerGeneral:
                /* Draw LEFT arrow */
                safeFont57(127, 1, 0);
                /* Draw RIGHT arrow */
                safeFont57(131, 1, 128-6);
                /* Draw screen */
                drawScreen_JammerGeneral();
                break;
            case JammerFrequency:
                /* Draw LEFT arrow */
                safeFont57(127, 1, 0);
                /* Draw screen */
                drawScreen_JammerFrequency();
                break;
            default:
                /* Looks like we're lost! Set state to Disclaimer screen */
                currentScreen = Disclaimer;
                /* Draw screen */
                drawScreen_Disclaimer();
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
    UI::drawScreen(currentScreen);
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
    switch(currentScreen)
    {
        case Disclaimer:
            doScreen_Disclaimer(action);
            break;
        case Home:
            doScreen_Home(action);
            break;
        case JammerGeneral:
            doScreen_JammerGeneral(action);
            break;
        case JammerFrequency:
            doScreen_JammerFrequency(action);
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

void UI::drawItem_JammerStep(int line, double step)
{
    if(step == STEP_1K_HZ)
        safeString("Step Size:    1 kHz", line, 14);
    else if(step == STEP_10K_HZ)
        safeString("Step Size:   10 kHz", line, 14);
    else if(step == STEP_25K_HZ)
        safeString("Step Size:   25 kHz", line, 14);
    else if(step == STEP_50K_HZ)
        safeString("Step Size:   50 kHz", line, 14);
    else if(step == STEP_100K_HZ)
        safeString("Step Size:  100 kHz", line, 14);
    else if(step == STEP_250K_HZ)
        safeString("Step Size:  250 kHz", line, 14);
    else if(step == STEP_500K_HZ)
        safeString("Step Size:  500 kHz", line, 14);
    else if(step == STEP_1M_HZ)
        safeString("Step Size:    1 MHz", line, 14);
    else if(step == STEP_NONE)
        safeString("Step Size:     None", line, 14);
}

void UI::drawItem_JammerFMOD(int line, FMODSetting_t fmod)
{
    switch(fmod)
    {
        case Auto:
            safeString("FMOD:          Auto", line, 14);
            break;
        case Force:
            safeString("FMOD:         Force", line, 14);
            break;
        case Off:
            safeString("FMOD:           Off", line, 14);
            break;
    }
}

void UI::drawItem_JammerAlgorithm(int line, JamAlgorithms_t algorithm)
{
    switch(algorithm)
    {
        case Sawtooth:
            safeString("Algo:      Sawtooth", line, 14);
            break;
        case Triangle:
            safeString("Algo:      Triangle", line, 14);
            break;
        case Random:
            safeString("Algo:        Random", line, 14);
            break;
    }
}

void UI::drawItem_JammerWaitForPLLLock(int line, bool wait)
{
    if(wait)
        safeString("WaitForPLLLock: Yes", line, 14);
    else
        safeString("WaitForPLLLock:  No", line, 14);
}

void UI::drawScreen_Disclaimer()
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

void UI::drawScreen_Home()
{
    char menuText[21];

    centredString("Home", 1);

    snprintf(menuText, sizeof(menuText), "Vpdet:   %04.2f V", stats.PDET_V);
    safeString(menuText, 2, 14);
    snprintf(menuText, sizeof(menuText), "Pout:    %05.2f dBm", stats.Pout_dBm);
    safeString(menuText, 3, 14);
    snprintf(menuText, sizeof(menuText), "Vbat:    %04.2f V", stats.VBAT_V);
    safeString(menuText, 4, 14);
    snprintf(menuText, sizeof(menuText), "Tmcu:    %05.2f degC", stats.OnChipTS_T_degC);
    safeString(menuText, 5, 14);
    snprintf(menuText, sizeof(menuText), "Trfamp:  %05.2f degC", stats.RFAmpTS_T_degC);
    safeString(menuText, 6, 14);
}

void UI::drawScreen_JammerGeneral()
{
    char menuText[21];

    centredString("Jammer - General", 1);

    drawItem_JammerFMOD(2, Jammer::settings.fmod);

    drawItem_JammerAlgorithm(3, Jammer::settings.algorithm);

    snprintf(menuText, sizeof(menuText), "Rate:      %5d Hz", Jammer::settings.rate);
    safeString(menuText, 4, 14);

    drawItem_JammerWaitForPLLLock(5, Jammer::settings.waitForPLLLock);

    if(isInSetting)
        safeFont57(131, cursorPos+2, 6);
    else
        safeFont57(131, cursorPos+2, 4);
}

void UI::drawScreen_JammerFrequency()
{
    char menuText[21];
    uint64_t f_start;
    uint64_t f_stop;

    centredString("Jammer - Frequency", 1);

    if(Jammer::settings.fmod == Force)
    {
        /* Centre Frequency */
        snprintf(menuText, sizeof(menuText), "Fc:     %4.2f MHz", Jammer::settings.fc / 1000000.0f);
        safeString(menuText, 2, 14);
        /* Calculate Start and Stop Frequency */
        RFMD_IntSynth::GetFMODFreqLimits(Jammer::settings.fc, Jammer::settings.step, f_start, f_stop);
        /* Preserve user setting */
        if(Jammer::settings.start < f_start || Jammer::settings.stop > f_stop)
        {
            Jammer::settings.start = roundToMultiple(f_start, FREQ_STEP_HZ, true);
            Jammer::settings.stop = roundToMultiple(f_stop, FREQ_STEP_HZ, false);
        }
        /* BW */
        Jammer::settings.BW = Jammer::settings.stop - Jammer::settings.start;
        snprintf(menuText, sizeof(menuText), "BW:      %6.2f MHz", Jammer::settings.BW / 1000000.0f);
        safeString(menuText, 3, 14);
        /* Start Freq */
        snprintf(menuText, sizeof(menuText), "Start:  %4.2f MHz", Jammer::settings.start / 1000000.0f);
        safeString(menuText, 4, 14);
        /* Stop Freq */
        snprintf(menuText, sizeof(menuText), "Stop:   %4.2f MHz", Jammer::settings.stop / 1000000.0f);
        safeString(menuText, 5, 14);
    }
    else
    {
        /* Centre Frequency */
        snprintf(menuText, sizeof(menuText), "Fc:     %4.2f MHz", (float) (Jammer::settings.fc / 1000000.0f));
        safeString(menuText, 2, 14);
        /* BW */
        Jammer::settings.BW = Jammer::settings.stop - Jammer::settings.start;
        snprintf(menuText, sizeof(menuText), "BW:      %6.2f MHz", (float) (Jammer::settings.BW / 1000000.0f));
        safeString(menuText, 3, 14);
        /* Start Freq */
        snprintf(menuText, sizeof(menuText), "Start:  %4.2f MHz", (float) (Jammer::settings.start / 1000000.0f));
        safeString(menuText, 4, 14);
        /* Stop Freq */
        snprintf(menuText, sizeof(menuText), "Stop:   %4.2f MHz", (float) (Jammer::settings.stop / 1000000.0f));
        safeString(menuText, 5, 14);
    }

    drawItem_JammerStep(6, Jammer::settings.step);

    if(isInSetting)
        safeFont57(131, cursorPos+2, 6);
    else
        safeFont57(131, cursorPos+2, 4);
}

void UI::doScreen_Disclaimer(buttonStates action)
{
    switch(action)
    {
        case ButtonUp:
            cursorPos = 0;
            drawScreen_Disclaimer();
            break;
        case ButtonDown:
            cursorPos = 1;
            drawScreen_Disclaimer();
            break;
        case ButtonLeft:
            break;
        case ButtonRight:
            break;
        case ButtonSelect:
            if(cursorPos == 1)
            {
                cursorPos = 0;
                UI::drawScreen(Home);
            }
            else
                splash("Please read", 1000);
            break;
        default:
            break;
    }
}

void UI::doItem_JammerFreqCentre(buttonStates action)
{
    char menuText[21];
    uint64_t f_start;
    uint64_t f_stop;

    switch(action)
    {
        case ButtonUp:
            /* Force FMOD */
            if(Jammer::settings.fmod == Force)
            {
                /* Calculate Start and Stop Frequency */
                RFMD_IntSynth::GetFMODFreqLimits(Jammer::settings.fc + FREQ_STEP_HZ, Jammer::settings.step, f_start, f_stop);
                /* Check limits */
                if(f_start < MIN_FREQ_HZ || f_stop > MAX_FREQ_HZ)
                {
                    UI::splash("At Limit", 1000);
                    toggleSetting(cursorPos);
                }
                else
                {
                    /* Centre Frequency */
                    Jammer::settings.fc += FREQ_STEP_HZ;
                    snprintf(menuText, sizeof(menuText), "Fc:     %4.2f MHz", Jammer::settings.fc / 1000000.0f);
                    safeString(menuText, 2, 14);
                    /* Update other dependent frequency variables */
                    // Calculate Start and Stop Frequency
                    RFMD_IntSynth::GetFMODFreqLimits(Jammer::settings.fc, Jammer::settings.step, f_start, f_stop);
                    Jammer::settings.start = roundToMultiple(f_start, FREQ_STEP_HZ, true); // Round up
                    Jammer::settings.stop = roundToMultiple(f_stop, FREQ_STEP_HZ, false); // Round down
                    // BW
                    Jammer::settings.BW = Jammer::settings.stop - Jammer::settings.start;
                    snprintf(menuText, sizeof(menuText), "BW:      %6.2f MHz", Jammer::settings.BW / 1000000.0f);
                    safeString(menuText, 3, 14);
                    // Start Frequency
                    snprintf(menuText, sizeof(menuText), "Start:  %4.2f MHz", Jammer::settings.start / 1000000.0f);
                    safeString(menuText, 4, 14);
                    // Stop Frequency
                    snprintf(menuText, sizeof(menuText), "Stop:   %4.2f MHz", Jammer::settings.stop / 1000000.0f);
                    safeString(menuText, 5, 14);
                }
            }
            /* Normal */
            else
            {
                /* Check limits */
                if((((Jammer::settings.fc + FREQ_STEP_HZ) + (Jammer::settings.BW/2)) > MAX_FREQ_HZ) ||
                        ((Jammer::settings.fc + FREQ_STEP_HZ) > MAX_FREQ_HZ))
                {
                    UI::splash("At Limit", 1000);
                    toggleSetting(cursorPos);
                }
                else
                {
                    Jammer::settings.fc += FREQ_STEP_HZ;
                    snprintf(menuText, sizeof(menuText), "Fc:     %4.2f MHz", Jammer::settings.fc / 1000000.0f);
                    safeString(menuText, 2, 14);
                    /* Update other dependent frequency variables */
                    // Start Frequency
                    Jammer::settings.start = Jammer::settings.fc - (Jammer::settings.BW/2);
                    snprintf(menuText, sizeof(menuText), "Start:  %4.2f MHz", Jammer::settings.start / 1000000.0f);
                    safeString(menuText, 4, 14);
                    // Stop Frequency
                    Jammer::settings.stop = Jammer::settings.fc + (Jammer::settings.BW/2);
                    snprintf(menuText, sizeof(menuText), "Stop:   %4.2f MHz", Jammer::settings.stop / 1000000.0f);
                    safeString(menuText, 5, 14);
                }
            }
            break;
        case ButtonDown:
            /* Force FMOD */
            if(Jammer::settings.fmod == Force)
            {
                /* Calculate Start and Stop Frequency */
                RFMD_IntSynth::GetFMODFreqLimits(Jammer::settings.fc - FREQ_STEP_HZ, Jammer::settings.step, f_start, f_stop);
                /* Check limits */
                if(f_start < MIN_FREQ_HZ || f_stop > MAX_FREQ_HZ)
                {
                    UI::splash("At Limit", 1000);
                    toggleSetting(cursorPos);
                }
                else
                {
                    Jammer::settings.fc -= FREQ_STEP_HZ;
                    snprintf(menuText, sizeof(menuText), "Fc:     %4.2f MHz", Jammer::settings.fc / 1000000.0f);
                    safeString(menuText, 2, 14);
                    /* Update other dependent frequency variables */
                    // Calculate Start and Stop Frequency
                    RFMD_IntSynth::GetFMODFreqLimits(Jammer::settings.fc, Jammer::settings.step, f_start, f_stop);
                    Jammer::settings.start = roundToMultiple(f_start, FREQ_STEP_HZ, true);
                    Jammer::settings.stop = roundToMultiple(f_stop, FREQ_STEP_HZ, false);
                    // BW
                    Jammer::settings.BW = Jammer::settings.stop - Jammer::settings.start;
                    snprintf(menuText, sizeof(menuText), "BW:      %6.2f MHz", Jammer::settings.BW / 1000000.0f);
                    safeString(menuText, 3, 14);
                    // Start Frequency
                    snprintf(menuText, sizeof(menuText), "Start:  %4.2f MHz", Jammer::settings.start / 1000000.0f);
                    safeString(menuText, 4, 14);
                    // Stop Frequency
                    snprintf(menuText, sizeof(menuText), "Stop:   %4.2f MHz", Jammer::settings.stop / 1000000.0f);
                    safeString(menuText, 5, 14);
                }
            }
            /* Normal */
            else
            {
                /* Check limits */
                if((((Jammer::settings.fc - FREQ_STEP_HZ) - (Jammer::settings.BW/2)) < MIN_FREQ_HZ) ||
                        ((Jammer::settings.fc - FREQ_STEP_HZ) < MIN_FREQ_HZ))
                {
                    UI::splash("At Limit", 1000);
                    toggleSetting(cursorPos);
                }
                else
                {
                    Jammer::settings.fc -= FREQ_STEP_HZ;
                    snprintf(menuText, sizeof(menuText), "Fc:     %4.2f MHz", Jammer::settings.fc / 1000000.0f);
                    safeString(menuText, 2, 14);
                    /* Update other dependent frequency variables */
                    // Start Frequency
                    Jammer::settings.start = Jammer::settings.fc - (Jammer::settings.BW/2);
                    snprintf(menuText, sizeof(menuText), "Start:  %4.2f MHz", Jammer::settings.start / 1000000.0f);
                    safeString(menuText, 4, 14);
                    // Stop Frequency
                    Jammer::settings.stop = Jammer::settings.fc + (Jammer::settings.BW/2);
                    snprintf(menuText, sizeof(menuText), "Stop:   %4.2f MHz", Jammer::settings.stop / 1000000.0f);
                    safeString(menuText, 5, 14);
                }
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

void UI::doItem_JammerBW(buttonStates action)
{
    char menuText[21];

    switch(action)
    {
        case ButtonUp:
            if(((Jammer::settings.fc + ((Jammer::settings.BW + FREQ_STEP_HZ) / 2)) <= MAX_FREQ_HZ) &&
                ((Jammer::settings.fc - ((Jammer::settings.BW + FREQ_STEP_HZ) / 2)) >= MIN_FREQ_HZ))
            {
                Jammer::settings.BW += FREQ_STEP_HZ;
                snprintf(menuText, sizeof(menuText), "BW:      %6.2f MHz", Jammer::settings.BW / 1000000.0f);
                safeString(menuText, 3, 14);
                /* Update other dependent frequency variables */
                // Start Frequency
                Jammer::settings.start = Jammer::settings.fc - (Jammer::settings.BW/2);
                snprintf(menuText, sizeof(menuText), "Start:  %4.2f MHz", Jammer::settings.start / 1000000.0f);
                safeString(menuText, 4, 14);
                // Stop Frequency
                Jammer::settings.stop = Jammer::settings.fc + (Jammer::settings.BW/2);
                snprintf(menuText, sizeof(menuText), "Stop:   %4.2f MHz", Jammer::settings.stop / 1000000.0f);
                safeString(menuText, 5, 14);
            }
            else
            {
                UI::splash("At Limit", 1000);
                toggleSetting(cursorPos);
            }
            break;
        case ButtonDown:
            if(Jammer::settings.BW == 0)
            {
                UI::splash("At Limit", 1000);
                toggleSetting(cursorPos);
            }
            else
            {
                Jammer::settings.BW -= FREQ_STEP_HZ;
                snprintf(menuText, sizeof(menuText), "BW:      %6.2f MHz", Jammer::settings.BW / 1000000.0f);
                safeString(menuText, 3, 14);
                /* Update other dependent frequency variables */
                // Start Frequency
                Jammer::settings.start = Jammer::settings.fc - (Jammer::settings.BW/2);
                snprintf(menuText, sizeof(menuText), "Start:  %4.2f MHz", Jammer::settings.start / 1000000.0f);
                safeString(menuText, 4, 14);
                // Stop Frequency
                Jammer::settings.stop = Jammer::settings.fc + (Jammer::settings.BW/2);
                snprintf(menuText, sizeof(menuText), "Stop:   %4.2f MHz", Jammer::settings.stop / 1000000.0f);
                safeString(menuText, 5, 14);
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

void UI::doItem_JammerFreqStart(buttonStates action)
{
    char menuText[21];
    uint64_t f_start;
    uint64_t f_stop;

    switch(action)
    {
        case ButtonUp:
            /* Prevent user from exceeding frequency limits */
            if(Jammer::settings.fmod == Force)
            {
                // Calculate Start and Stop Frequency
                RFMD_IntSynth::GetFMODFreqLimits(Jammer::settings.fc, Jammer::settings.step, f_start, f_stop);
                if((Jammer::settings.start + FREQ_STEP_HZ) > f_stop)
                {
                    UI::splash("At Limit", 1000);
                    toggleSetting(cursorPos);
                }
                else
                {
                    /* Check limits */
                    if((Jammer::settings.start + FREQ_STEP_HZ) > Jammer::settings.stop)
                    {
                        UI::splash("At Limit", 1000);
                        toggleSetting(cursorPos);
                    }
                    else
                    {
                        Jammer::settings.start += FREQ_STEP_HZ;
                        snprintf(menuText, sizeof(menuText), "Start:  %4.2f MHz", Jammer::settings.start / 1000000.0f);
                        safeString(menuText, 4, 14);
                        /* Update other dependent frequency variables */
                        // BW
                        Jammer::settings.BW = Jammer::settings.stop - Jammer::settings.start;
                        snprintf(menuText, sizeof(menuText), "BW:      %6.2f MHz", Jammer::settings.BW / 1000000.0f);
                        safeString(menuText, 3, 14);
                    }
                }
            }
            else
            {
                /* Check limits */
                if((Jammer::settings.start + FREQ_STEP_HZ) > Jammer::settings.stop)
                {
                    UI::splash("At Limit", 1000);
                    toggleSetting(cursorPos);
                }
                else
                {
                    Jammer::settings.start += FREQ_STEP_HZ;
                    snprintf(menuText, sizeof(menuText), "Start:  %4.2f MHz", Jammer::settings.start / 1000000.0f);
                    safeString(menuText, 4, 14);
                    /* Update other dependent frequency variables */
                    // Centre Frequency
                    Jammer::settings.fc = Jammer::settings.start + ((Jammer::settings.stop - Jammer::settings.start)/2);
                    snprintf(menuText, sizeof(menuText), "Fc:     %4.2f MHz", Jammer::settings.fc / 1000000.0f);
                    safeString(menuText, 2, 14);
                    // BW
                    Jammer::settings.BW = Jammer::settings.stop - Jammer::settings.start;
                    snprintf(menuText, sizeof(menuText), "BW:      %6.2f MHz", Jammer::settings.BW / 1000000.0f);
                    safeString(menuText, 3, 14);
                }
            }
            break;
        case ButtonDown:
            /* Prevent user from exceeding frequency limits */
            if(Jammer::settings.fmod == Force)
            {
                // Calculate Start and Stop Frequency
                RFMD_IntSynth::GetFMODFreqLimits(Jammer::settings.fc, Jammer::settings.step, f_start, f_stop);
                if((Jammer::settings.start - FREQ_STEP_HZ) < f_start)
                {
                    UI::splash("At Limit", 1000);
                    toggleSetting(cursorPos);
                }
                else
                {
                    /* Check limits */
                    if((Jammer::settings.start - FREQ_STEP_HZ) < MIN_FREQ_HZ)
                    {
                        UI::splash("At Limit", 1000);
                        toggleSetting(cursorPos);
                    }
                    else
                    {
                        Jammer::settings.start -= FREQ_STEP_HZ;
                        snprintf(menuText, sizeof(menuText), "Start:  %4.2f MHz", Jammer::settings.start / 1000000.0f);
                        safeString(menuText, 4, 14);
                        /* Update other dependent frequency variables */
                        // BW
                        Jammer::settings.BW = Jammer::settings.stop - Jammer::settings.start;
                        snprintf(menuText, sizeof(menuText), "BW:      %6.2f MHz", Jammer::settings.BW / 1000000.0f);
                        safeString(menuText, 3, 14);
                    }
                }
            }
            else
            {
                /* Check limits */
                if((Jammer::settings.start - FREQ_STEP_HZ) < MIN_FREQ_HZ)
                {
                    UI::splash("At Limit", 1000);
                    toggleSetting(cursorPos);
                }
                else
                {
                    Jammer::settings.start -= FREQ_STEP_HZ;
                    snprintf(menuText, sizeof(menuText), "Start:  %4.2f MHz", Jammer::settings.start / 1000000.0f);
                    safeString(menuText, 4, 14);
                    /* Update other dependent frequency variables */
                    // Centre Frequency
                    Jammer::settings.fc = Jammer::settings.start + ((Jammer::settings.stop - Jammer::settings.start)/2);
                    snprintf(menuText, sizeof(menuText), "Fc:     %4.2f MHz", Jammer::settings.fc / 1000000.0f);
                    safeString(menuText, 2, 14);
                    // BW
                    Jammer::settings.BW = Jammer::settings.stop - Jammer::settings.start;
                    snprintf(menuText, sizeof(menuText), "BW:      %6.2f MHz", Jammer::settings.BW / 1000000.0f);
                    safeString(menuText, 3, 14);
                }
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

void UI::doItem_JammerFreqStop(buttonStates action)
{
    char menuText[21];
    uint64_t f_start;
    uint64_t f_stop;

    switch(action)
    {
        case ButtonUp:
            /* Prevent user from exceeding frequency limits */
            if(Jammer::settings.fmod == Force)
            {
                // Calculate Start and Stop Frequency
                RFMD_IntSynth::GetFMODFreqLimits(Jammer::settings.fc, Jammer::settings.step, f_start, f_stop);
                if((Jammer::settings.stop + FREQ_STEP_HZ) > f_stop)
                {
                    UI::splash("At Limit", 1000);
                    toggleSetting(cursorPos);
                }
                else
                {
                    /* Check limits */
                    if((Jammer::settings.stop + FREQ_STEP_HZ) > MAX_FREQ_HZ)
                    {
                        UI::splash("At Limit", 1000);
                        toggleSetting(cursorPos);
                    }
                    else
                    {
                        Jammer::settings.stop += FREQ_STEP_HZ;
                        snprintf(menuText, sizeof(menuText), "Stop:   %4.2f MHz", Jammer::settings.stop / 1000000.0f);
                        safeString(menuText, 5, 14);
                        /* Update other dependent frequency variables */
                        // BW
                        Jammer::settings.BW = Jammer::settings.stop - Jammer::settings.start;
                        snprintf(menuText, sizeof(menuText), "BW:      %6.2f MHz", Jammer::settings.BW / 1000000.0f);
                        safeString(menuText, 3, 14);
                    }
                }
            }
            else
            {
                /* Check limits */
                if((Jammer::settings.stop + FREQ_STEP_HZ) > MAX_FREQ_HZ)
                {
                    UI::splash("At Limit", 1000);
                    toggleSetting(cursorPos);
                }
                else
                {
                    Jammer::settings.stop += FREQ_STEP_HZ;
                    snprintf(menuText, sizeof(menuText), "Stop:   %4.2f MHz", Jammer::settings.stop / 1000000.0f);
                    safeString(menuText, 5, 14);
                    /* Update other dependent frequency variables */
                    // Centre Frequency
                    Jammer::settings.fc = Jammer::settings.start + ((Jammer::settings.stop - Jammer::settings.start)/2);
                    snprintf(menuText, sizeof(menuText), "Fc:     %4.2f MHz", Jammer::settings.fc / 1000000.0f);
                    safeString(menuText, 2, 14);
                    // BW
                    Jammer::settings.BW = Jammer::settings.stop - Jammer::settings.start;
                    snprintf(menuText, sizeof(menuText), "BW:      %6.2f MHz", Jammer::settings.BW / 1000000.0f);
                    safeString(menuText, 3, 14);
                }
            }
            break;
        case ButtonDown:
            /* Prevent user from exceeding frequency limits */
            if(Jammer::settings.fmod == Force)
            {
                // Calculate Start and Stop Frequency
                RFMD_IntSynth::GetFMODFreqLimits(Jammer::settings.fc, Jammer::settings.step, f_start, f_stop);
                if((Jammer::settings.stop - FREQ_STEP_HZ) < f_start)
                {
                    UI::splash("At Limit", 1000);
                    toggleSetting(cursorPos);
                }
                else
                {
                    /* Check limits */
                    if((Jammer::settings.stop - FREQ_STEP_HZ) < Jammer::settings.start)
                    {
                        UI::splash("At Limit", 1000);
                        toggleSetting(cursorPos);
                    }
                    else
                    {
                        Jammer::settings.stop -= FREQ_STEP_HZ;
                        snprintf(menuText, sizeof(menuText), "Stop:   %4.2f MHz", Jammer::settings.stop / 1000000.0f);
                        safeString(menuText, 5, 14);
                        /* Update other dependent frequency variables */
                        // BW
                        Jammer::settings.BW = Jammer::settings.stop - Jammer::settings.start;
                        snprintf(menuText, sizeof(menuText), "BW:      %6.2f MHz", Jammer::settings.BW / 1000000.0f);
                        safeString(menuText, 3, 14);
                    }
                }
            }
            else
            {
                /* Check limits */
                if((Jammer::settings.stop - FREQ_STEP_HZ) < Jammer::settings.start)
                {
                    UI::splash("At Limit", 1000);
                    toggleSetting(cursorPos);
                }
                else
                {
                    Jammer::settings.stop -= FREQ_STEP_HZ;
                    snprintf(menuText, sizeof(menuText), "Stop:   %4.2f MHz", Jammer::settings.stop / 1000000.0f);
                    safeString(menuText, 5, 14);
                    /* Update other dependent frequency variables */
                    // Centre Frequency
                    Jammer::settings.fc = Jammer::settings.start + ((Jammer::settings.stop - Jammer::settings.start)/2);
                    snprintf(menuText, sizeof(menuText), "Fc:     %4.2f MHz", Jammer::settings.fc / 1000000.0f);
                    safeString(menuText, 2, 14);
                    // BW
                    Jammer::settings.BW = Jammer::settings.stop - Jammer::settings.start;
                    snprintf(menuText, sizeof(menuText), "BW:      %6.2f MHz", Jammer::settings.BW / 1000000.0f);
                    safeString(menuText, 3, 14);
                }
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

void UI::doItem_JammerFMOD(buttonStates action)
{
    switch(action)
    {
        case ButtonSelect:
        case ButtonRight:
        case ButtonLeft:
            toggleSetting(cursorPos);
            break;
        case ButtonUp:
            switch(Jammer::settings.fmod)
            {
                case Auto:
                    Jammer::settings.fmod = Off;
                    drawItem_JammerFMOD(2, Jammer::settings.fmod);
                    break;
                case Force:
                    Jammer::settings.fmod = Auto;
                    drawItem_JammerFMOD(2, Jammer::settings.fmod);
                    /* Reset Centre Frequency to default */
                    Jammer::settings.fc = JAMMER_SETTINGS_DEFAULT_FC_HZ;
                    break;
                case Off:
                    Jammer::settings.fmod = Force;
                    drawItem_JammerFMOD(2, Jammer::settings.fmod);
                    break;
            }
            break;
        case ButtonDown:
            switch(Jammer::settings.fmod)
            {
                case Auto:
                    Jammer::settings.fmod = Force;
                    drawItem_JammerFMOD(2, Jammer::settings.fmod);
                    break;
                case Force:
                    Jammer::settings.fmod = Off;
                    drawItem_JammerFMOD(2, Jammer::settings.fmod);
                    /* Reset Centre Frequency to default */
                    Jammer::settings.fc = JAMMER_SETTINGS_DEFAULT_FC_HZ;
                    break;
                case Off:
                    Jammer::settings.fmod = Auto;
                    drawItem_JammerFMOD(2, Jammer::settings.fmod);
                    break;
            }
            break;
        default:
            break;
    }
}

void UI::doItem_JammerAlgorithm(buttonStates action)
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
                case Sawtooth:
                    Jammer::settings.algorithm = Random;
                    drawItem_JammerAlgorithm(3, Jammer::settings.algorithm);
                    break;
                case Triangle:
                    Jammer::settings.algorithm = Sawtooth;
                    drawItem_JammerAlgorithm(3, Jammer::settings.algorithm);
                    break;
                case Random:
                    Jammer::settings.algorithm = Triangle;
                    drawItem_JammerAlgorithm(3, Jammer::settings.algorithm);
                    break;
            }
            break;
        case ButtonDown:
            switch(Jammer::settings.algorithm)
            {
                case Sawtooth:
                    Jammer::settings.algorithm = Triangle;
                    drawItem_JammerAlgorithm(3, Jammer::settings.algorithm);
                    break;
                case Triangle:
                    Jammer::settings.algorithm = Random;
                    drawItem_JammerAlgorithm(3, Jammer::settings.algorithm);
                    break;
                case Random:
                    Jammer::settings.algorithm = Sawtooth;
                    drawItem_JammerAlgorithm(3, Jammer::settings.algorithm);
                    break;
            }
            break;
        default:
            break;
    }
}

void UI::doItem_JammerRate(buttonStates action)
{
    char menuText[21];

    switch(action)
    {
        case ButtonUp:
            if((Jammer::settings.rate + RATE_STEP_HZ) <= MAX_RATE_HZ)
            {
                Jammer::settings.rate += RATE_STEP_HZ;
                snprintf(menuText, sizeof(menuText), "Rate:      %5d Hz", Jammer::settings.rate);
                safeString(menuText, 4, 14);
            }
            else
            {
                Jammer::settings.rate = MAX_RATE_HZ;
                snprintf(menuText, sizeof(menuText), "Rate:      %5d Hz", Jammer::settings.rate);
                safeString(menuText, 4, 14);
            }
            break;
        case ButtonDown:
            if((Jammer::settings.rate - RATE_STEP_HZ) >= MIN_RATE_HZ)
            {
                Jammer::settings.rate -= RATE_STEP_HZ;
                snprintf(menuText, sizeof(menuText), "Rate:      %5d Hz", Jammer::settings.rate);
                safeString(menuText, 4, 14);
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

void UI::doItem_WaitForPLLLock(buttonStates action)
{
    switch(action)
    {
        case ButtonUp:
            Jammer::settings.waitForPLLLock = !Jammer::settings.waitForPLLLock;
            drawItem_JammerWaitForPLLLock(5, Jammer::settings.waitForPLLLock);
            break;
        case ButtonDown:
            Jammer::settings.waitForPLLLock = !Jammer::settings.waitForPLLLock;
            drawItem_JammerWaitForPLLLock(5, Jammer::settings.waitForPLLLock);
            break;
        case ButtonSelect:
        case ButtonLeft:
        case ButtonRight:
        default:
            toggleSetting(cursorPos);
            break;
    }
}

void UI::doItem_JammerStepSize(buttonStates action)
{
    char menuText[21];
    uint64_t f_start;
    uint64_t f_stop;

    switch(action)
    {
        case ButtonSelect:
        case ButtonRight:
        case ButtonLeft:
            toggleSetting(cursorPos);
            break;
        case ButtonUp:
            /* Only toggle step size if algorithm is not set to RANDOM */
            if(Jammer::settings.algorithm != Random)
            {
                // Step forwards through the cycle
                if(Jammer::settings.step == STEP_1K_HZ)
                    Jammer::settings.step = STEP_10K_HZ;
                else if(Jammer::settings.step == STEP_10K_HZ)
                    Jammer::settings.step = STEP_25K_HZ;
                else if(Jammer::settings.step == STEP_25K_HZ)
                    Jammer::settings.step = STEP_50K_HZ;
                else if(Jammer::settings.step == STEP_50K_HZ)
                    Jammer::settings.step = STEP_100K_HZ;
                else if(Jammer::settings.step == STEP_100K_HZ)
                    Jammer::settings.step = STEP_250K_HZ;
                else if(Jammer::settings.step == STEP_250K_HZ)
                    Jammer::settings.step = STEP_500K_HZ;
                else if(Jammer::settings.step == STEP_500K_HZ)
                    Jammer::settings.step = STEP_1M_HZ;
                drawItem_JammerStep(6, Jammer::settings.step);

                /* Update other dependent frequency variables */
                if(Jammer::settings.fmod == Force)
                {
                    // Calculate Start and Stop Frequency
                    RFMD_IntSynth::GetFMODFreqLimits(Jammer::settings.fc, Jammer::settings.step, f_start, f_stop);
                    Jammer::settings.start = roundToMultiple(f_start, FREQ_STEP_HZ, true);
                    Jammer::settings.stop = roundToMultiple(f_stop, FREQ_STEP_HZ, false);
                    // BW
                    Jammer::settings.BW = Jammer::settings.stop - Jammer::settings.start;
                    snprintf(menuText, sizeof(menuText), "BW:      %6.2f MHz", Jammer::settings.BW / 1000000.0f);
                    safeString(menuText, 3, 14);
                    // Start Frequency
                    Jammer::settings.start = Jammer::settings.fc - (Jammer::settings.BW/2);
                    snprintf(menuText, sizeof(menuText), "Start:  %4.2f MHz", Jammer::settings.start / 1000000.0f);
                    safeString(menuText, 4, 14);
                    // Stop Frequency
                    Jammer::settings.stop = Jammer::settings.fc + (Jammer::settings.BW/2);
                    snprintf(menuText, sizeof(menuText), "Stop:   %4.2f MHz", Jammer::settings.stop / 1000000.0f);
                    safeString(menuText, 5, 14);
                }
            }
            break;
        case ButtonDown:
            /* Only toggle step size if algorithm is not set to RANDOM */
            if(Jammer::settings.algorithm != Random)
            {
                // Step backwards through the cycle
                if(Jammer::settings.step == STEP_1M_HZ)
                    Jammer::settings.step = STEP_500K_HZ;
                else if(Jammer::settings.step == STEP_500K_HZ)
                    Jammer::settings.step = STEP_250K_HZ;
                else if(Jammer::settings.step == STEP_250K_HZ)
                    Jammer::settings.step = STEP_100K_HZ;
                else if(Jammer::settings.step == STEP_100K_HZ)
                    Jammer::settings.step = STEP_50K_HZ;
                else if(Jammer::settings.step == STEP_50K_HZ)
                    Jammer::settings.step = STEP_25K_HZ;
                else if(Jammer::settings.step == STEP_25K_HZ)
                    Jammer::settings.step = STEP_10K_HZ;
                else if(Jammer::settings.step == STEP_10K_HZ)
                    Jammer::settings.step = STEP_1K_HZ;
                drawItem_JammerStep(6, Jammer::settings.step);

                /* Update other dependent frequency variables */
                if(Jammer::settings.fmod == Force)
                {
                    // Calculate Start and Stop Frequency
                    RFMD_IntSynth::GetFMODFreqLimits(Jammer::settings.fc, Jammer::settings.step, f_start, f_stop);
                    Jammer::settings.start = roundToMultiple(f_start, FREQ_STEP_HZ, true);
                    Jammer::settings.stop = roundToMultiple(f_stop, FREQ_STEP_HZ, false);
                    // BW
                    Jammer::settings.BW = Jammer::settings.stop - Jammer::settings.start;
                    snprintf(menuText, sizeof(menuText), "BW:      %6.2f MHz", Jammer::settings.BW / 1000000.0f);
                    safeString(menuText, 3, 14);
                    // Start Frequency
                    Jammer::settings.start = Jammer::settings.fc - (Jammer::settings.BW/2);
                    snprintf(menuText, sizeof(menuText), "Start:  %4.2f MHz", Jammer::settings.start / 1000000.0f);
                    safeString(menuText, 4, 14);
                    // Stop Frequency
                    Jammer::settings.stop = Jammer::settings.fc + (Jammer::settings.BW/2);
                    snprintf(menuText, sizeof(menuText), "Stop:   %4.2f MHz", Jammer::settings.stop / 1000000.0f);
                    safeString(menuText, 5, 14);
                }
            }
            break;
        default:
            break;
    }
}

void UI::doScreen_JammerGeneral(buttonStates action)
{
    // We're in a setting, let's handle the key press depending on which one
    if(isInSetting)
    {
        switch(cursorPos)
        {
            case 0:
                doItem_JammerFMOD(action);
                break;
            case 1:
                doItem_JammerAlgorithm(action);
                break;
            case 2:
                doItem_JammerRate(action);
                break;
            case 3:
                doItem_WaitForPLLLock(action);
                break;
            case 4:
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
            cursorPos = 0;
            UI::drawScreen(Home);
            break;
        case ButtonRight:
            cursorPos = 0;
            UI::drawScreen(JammerFrequency);
            break;
        case ButtonSelect:
            switch(cursorPos)
            {
                case 0:
                    toggleSetting(cursorPos);
                    break;
                case 1:
                    toggleSetting(cursorPos);
                    break;
                case 2:
                    toggleSetting(cursorPos);
                    break;
                case 3:
                    toggleSetting(cursorPos);
                    break;
                case 4:
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

void UI::doScreen_JammerFrequency(buttonStates action)
{
    // We're in a setting, let's handle the key press depending on which one
    if(isInSetting)
    {
        switch(cursorPos)
        {
            case 0:
                doItem_JammerFreqCentre(action);
                break;
            case 1:
                doItem_JammerBW(action);
                break;
            case 2:
                doItem_JammerFreqStart(action);
                break;
            case 3:
                doItem_JammerFreqStop(action);
                break;
            case 4:
                doItem_JammerStepSize(action);
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
            UI::drawScreen(JammerGeneral);
            break;
        case ButtonRight:
            break;
        case ButtonSelect:
            switch(cursorPos)
            {
                case 0:
                    toggleSetting(cursorPos);
                    break;
                case 1:
                    if(Jammer::settings.fmod == Force)
                        splash("Cannot set (AUTO)", 1000);
                    else
                        toggleSetting(cursorPos);
                    break;
                case 2:
                    toggleSetting(cursorPos);
                    break;
                case 3:
                    toggleSetting(cursorPos);
                    break;
                case 4:
                    toggleSetting(cursorPos);
                    break;
            }
            break;
        default:
            break;
    }
}

void UI::doScreen_Home(buttonStates action)
{
    switch(action)
    {
        case ButtonLeft:
            break;
        case ButtonRight:
            cursorPos = 0;
            UI::drawScreen(JammerGeneral);
            break;
        default:
            break;
    }
}

uint64_t UI::roundToMultiple(uint64_t numToRound, uint64_t multiple, bool doRoundUp)
{
    if(multiple == 0)
        return numToRound;

    uint64_t remainder = numToRound % multiple;
    if (remainder == 0)
        return numToRound;
    return doRoundUp ? (numToRound + multiple - remainder) : (numToRound - remainder);
}
