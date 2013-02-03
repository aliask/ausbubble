/************************************************************************/
/* AusBubble:                                                           */
/* An open-source RF jammer designed to operate in the 2.4 GHz Wi-Fi    */
/* frequency block.                                                     */
/*                                                                      */
/* ScanAlgorithms.cpp                                                   */
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

#include "ScanAlgorithms.h"

struct scanSettings_t gScanSettings;

void ScanInit(scanSettings_t* settings)
{
    settings->algorithm = ScanTriangle;
    settings->start     = 2400000000;
    settings->stop      = 2500000000;
    settings->stepSize  = STEP_100K_HZ;
}

void AdvanceScan(scanSettings_t* settings)
{
    static uint64_t freq = settings->start;
    static bool direction = DIR_UP;
    uint64_t newFreq = -1;
    uint32_t random32bit = 0;

    switch(settings->algorithm)
    {
        case ScanSawtooth:
            if((freq + settings->stepSize) > settings->stop)
                newFreq = settings->start;
            else
                newFreq = freq + settings->stepSize;
            break;
        case ScanTriangle:
            if(direction == DIR_UP)
            {
                // Going up
                if((freq + settings->stepSize) >= settings->stop)
                {
                    newFreq = settings->stop;
                    direction = DIR_DOWN;
                }
                else
                    newFreq = freq + settings->stepSize;
            }
            else
            {
                // Going down
                if((freq - settings->stepSize) <= settings->start)
                {
                    newFreq = settings->start;
                    direction = DIR_UP;
                }
                else
                    newFreq = freq - settings->stepSize;
            }
            break;
        case ScanRandom:
        	/* Wait until one RNG number is ready */
        	while(RNG_GetFlagStatus(RNG_FLAG_DRDY)== RESET);
            /* Get a 32bit Random number */
        	random32bit = RNG_GetRandomNumber();
            /* Get random floating point number between START and STOP frequency range */
            newFreq = settings->start + (float)random32bit/((float)UINT32_MAX/(settings->stop-settings->start));
            break;
        default:
            newFreq = freq;
            break;
    }

    if(newFreq != freq)
    {
        // Set synth frequency
        SynthSet_Freq(newFreq);
        freq = newFreq;
    }
}
