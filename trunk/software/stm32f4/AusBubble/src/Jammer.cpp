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
struct JamSettings_t Jammer::settings;
bool Jammer::enabled = false;
uint64_t Jammer::currentFreq = 0;

void Jammer::Init(void)
{
    /* Initialize settings with defaults */
    settings.fmod           = JAMMER_SETTINGS_DEFAULT_FMOD;
    settings.algorithm      = JAMMER_SETTINGS_DEFAULT_ALGO;
    settings.rate           = JAMMER_SETTINGS_DEFAULT_RATE_HZ;
    settings.waitForPLLLock = JAMMER_SETTINGS_DEFAULT_WAIT_FOR_PLL_LOCK;
    settings.start          = JAMMER_SETTINGS_DEFAULT_START_FREQ_HZ;
    settings.stop           = JAMMER_SETTINGS_DEFAULT_STOP_FREQ_HZ;
    settings.step           = JAMMER_SETTINGS_DEFAULT_STEP_HZ;
    settings.fc             = JAMMER_SETTINGS_DEFAULT_FC_HZ;
    settings.BW             = JAMMER_SETTINGS_DEFAULT_BW_HZ;

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
        RFMD_IntSynth::SetEnabled(true);

        /* Set jam update rate */
        SetUpdateRate(settings.rate);
    }
    else
    {
        /* Disable synthesizer */
        RFMD_IntSynth::SetEnabled(false);
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
    currentFreq = 0;
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
    static uint64_t newFreq;
    static uint32_t random32bit;
    static Direction_t direction;

    /* Set initial frequency */
    if(currentFreq == 0)
    {
        if(settings.fmod == Force)
            newFreq = settings.fc;
        else
            newFreq = settings.start;
    }
    /* Execute algorithm */
    else
    {
        switch(settings.algorithm)
        {
            /* SAWTOOTH */
            case Sawtooth:
                /* Increase frequency */
                if((currentFreq + settings.step) <= settings.stop)
                    newFreq += settings.step;
                /* If at upper bound, return to start frequency */
                else
                    newFreq = settings.start;
                break;
            /* TRIANGLE */
            case Triangle:
                /* Current direction is UP */
                if(direction == Up)
                {
                    /* Increase frequency */
                    if((currentFreq + settings.step) <= settings.stop)
                        newFreq += settings.step;
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
                    if((currentFreq - settings.step) >= settings.start)
                        newFreq -= settings.step;
                    /* At (or exceeded) lower bound, reverse direction */
                    else
                    {
                        newFreq = settings.start;
                        direction = Up;
                    }
                }
                break;
            /* RANDOM */
            case Random:
                /* Wait until one RNG number is ready */
                while(RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET);
                /* Get a 32bit Random number */
                random32bit = RNG_GetRandomNumber();
                /* Get random number between START and STOP frequency range */
                newFreq = settings.start + (uint32_t)(((float) random32bit / (float) UINT32_MAX)*(settings.stop - settings.start));
                break;
            /* Invalid algorithm */
            default:
                break;
        }
    }

    /* Set synthesizer frequency (only if different to current frequency) */
    if(newFreq != currentFreq)
    {
        RFMD_IntSynth::SetFreq(newFreq, settings.waitForPLLLock, (settings.fmod == Off) ? false : true);
        /* Save current frequency in "currentFreq" static variable */
        currentFreq = newFreq;
    }
}
