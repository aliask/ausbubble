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
    settings->algorithm = ScanRandom;
    settings->start     = 2400.0;
    settings->stop      = 2500.0;
    settings->stepSize  = STEP_10K;
}

void AdvanceScan(scanSettings_t* settings)
{
    static double freq = settings->start;
    static bool direction = DIR_UP;
    float newFreq = -1;

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
                if((freq + settings->stepSize) > settings->stop)
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
                if((freq - settings->stepSize) < settings->start)
                {
                    newFreq = settings->start;
                    direction = DIR_UP;
                }
                else
                    newFreq = freq - settings->stepSize;
            }
            break;
        case ScanRandom:
            // Get random floating point number between START AND STOP frequency range
            newFreq = settings->start + (float)rand()/((float)RAND_MAX/(settings->stop-settings->start));
            break;
        default:
            newFreq = freq;
            break;
    }

    if(newFreq != freq)
    {
        // Set frequency in synth
        taskENTER_CRITICAL();
        SynthSetFreq(newFreq);
        taskEXIT_CRITICAL();
        freq = newFreq;
    }
}
