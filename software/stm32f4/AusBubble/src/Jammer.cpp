/************************************************************************/
/* AusBubble:                                                           */
/* An open-source RF jammer designed to operate in the 2.4 GHz Wi-Fi    */
/* frequency block.                                                     */
/*                                                                      */
/* Jammer.cpp                                                           */
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

#include "Jammer.h"

/* Initialize static members */
struct jamSettings_t Jammer::settings;
bool Jammer::enabled = false;
bool Jammer::firstAlgoRun = false;

void Jammer::Init(void)
{
    /* Initialize settings with defaults */
    settings.start     = SCAN_SETTINGS_DEFAULT_START_FREQ_HZ;
    settings.stop      = SCAN_SETTINGS_DEFAULT_STOP_FREQ_HZ;
    settings.algorithm = SCAN_SETTINGS_DEFAULT_ALGO;
    settings.stepSize  = SCAN_SETTINGS_DEFAULT_STEPSIZE;
    settings.rate      = SCAN_SETTINGS_DEFAULT_RATE_HZ;

    /* Ensure jamming is disabled */
    SetEnabled(false);
}

void Jammer::SetEnabled(bool enable)
{
    if(enable)
    {
        /* Enable amplifier */
        RFPA5201_Amp::SetEnabled(true);
        /* Set gain to maximum allowable */
        RDA1005L_VarGainAmp::SetGain(VARGAINAMP_MAX_GAIN_LIMIT_DB);
        /* Enable synthesizer */
        RFFCx07x_Synth::SetEnabled(true);

        /* Set jam update rate */
        SetUpdateRate(settings.rate);
    }
    else
    {
        /* Disable synthesizer */
        RFFCx07x_Synth::SetEnabled(false);
        /* Set gain to minimum allowable */
        RDA1005L_VarGainAmp::SetGain(VARGAINAMP_MIN_GAIN_LIMIT_DB);
        /* Disable amplifier */
        RFPA5201_Amp::SetEnabled(false);

        /* TIM IT disable */
        TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
        /* TIM2 disable counter */
        TIM_Cmd(TIM2, DISABLE);
    }

    /* Set internal flags */
    enabled = enable;
    firstAlgoRun = enable;
}

bool Jammer::isEnabled(void)
{
    return enabled;
}

void Jammer::SetUpdateRate(uint16_t updateRate_Hz)
{
    /* Time base configuration */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = (1000000/updateRate_Hz) - 1; // 1 MHz timer clock
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;                   // 1 MHz timer clock
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    /* TIM IT enable */
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    /* TIM2 enable counter */
    TIM_Cmd(TIM2, ENABLE);
}

void Jammer::Advance(void)
{
    static uint64_t freq = settings.start;
    static ScanDirection_t direction = Up;
    static uint32_t random32bit;
    uint64_t newFreq = freq;

    switch(settings.algorithm)
    {
        /* SAWTOOTH */
        case ScanSawtooth:
            /* Increase frequency */
            if((freq + settings.stepSize) <= settings.stop)
                newFreq += settings.stepSize;
            /* If at upper bound, return to start frequency */
            else
                newFreq = settings.start;
            break;
        /* TRIANGLE */
        case ScanTriangle:
            /* Current direction is UP */
            if(direction == Up)
            {
                /* Increase frequency */
                if((freq + settings.stepSize) < settings.stop)
                    newFreq += settings.stepSize;
                /* At (or exceeded) upper bound, reverse direction */
                else
                {
                    newFreq = settings.stop;
                    direction = Down;
                }
            }
            /* Current direction is DOWN */
            else
            {
                /* Decrease frequency */
                if((freq - settings.stepSize) > settings.start)
                    newFreq -= settings.stepSize;
                /* At (or exceeded) lower bound, reverse direction */
                else
                {
                    newFreq = settings.start;
                    direction = Up;
                }
            }
            break;
        /* RANDOM */
        case ScanRandom:
            /* Wait until one RNG number is ready */
            while(RNG_GetFlagStatus(RNG_FLAG_DRDY)== RESET);
            /* Get a 32bit Random number */
            random32bit = RNG_GetRandomNumber();
            /* Get random number between START and STOP frequency range */
            newFreq = settings.start + (uint32_t)(((float) random32bit / (float) UINT32_MAX)*(settings.stop - settings.start));
            break;
        /* Invalid algorithm */
        default:
            break;
    }

    /* Set synthesizer frequency (only if different to current frequency OR if first call when set to single freq) */
    if((newFreq != freq) || (firstAlgoRun && (settings.start==settings.stop)))
    {
        RFFCx07x_Synth::SetFreq(newFreq, true, false);  // Wait for PLL lock, Frequency modulation OFF
        freq = newFreq;
    }

    /* Set flag */
    firstAlgoRun = false;
}
