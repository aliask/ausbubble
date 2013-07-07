/************************************************************************/
/* AusBubble:                                                           */
/* An open-source RF jammer designed to operate in the 2.4 GHz Wi-Fi    */
/* frequency block.                                                     */
/*                                                                      */
/* RFPA5201_Amp.cpp                                                     */
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

#include "RFPA5201_Amp.h"

/* Initialize static members */
bool RFPA5201_Amp::enabled = false;

/* RFPA5201 EVM: 11n MCS7 HT40; Vcc=5v; Vreg=2.9v; Temp=25degC; Duty Cycle=50%; f=2450MHz */
// Data points format: Pdet(V) => Pout(dB)
float RFPA5201_Amp::dataPoints[N_SAMPLES][2] =
      {
        { 0.103384, 0.83 },
        { 0.103922, 1 },
        { 0.105509, 1.5 },
        { 0.107157, 2 },
        { 0.108910, 2.5 },
        { 0.110815, 3 },
        { 0.112917, 3.5 },
        { 0.115261, 4 },
        { 0.117887, 4.5 },
        { 0.120831, 5 },
        { 0.124126, 5.5 },
        { 0.127807, 6 },
        { 0.131907, 6.5 },
        { 0.136461, 7 },
        { 0.141508, 7.5 },
        { 0.147101, 8 },
        { 0.153290, 8.5 },
        { 0.160125, 9 },
        { 0.167661, 9.5 },
        { 0.175949, 10 },
        { 0.185034, 10.5 },
        { 0.194957, 11 },
        { 0.205760, 11.5 },
        { 0.217483, 12 },
        { 0.230164, 12.5 },
        { 0.243848, 13 },
        { 0.258599, 13.5 },
        { 0.274519, 14 },
        { 0.291717, 14.5 },
        { 0.310298, 15 },
        { 0.330370, 15.5 },
        { 0.352041, 16 },
        { 0.375404, 16.5 },
        { 0.400535, 17 },
        { 0.427504, 17.5 },
        { 0.456380, 18 },
        { 0.487229, 18.5 },
        { 0.520135, 19 },
        { 0.555185, 19.5 },
        { 0.592644, 20 },
        { 0.632811, 20.5 },
        { 0.675997, 21 },
        { 0.722506, 21.5 },
        { 0.772656, 22 },
        { 0.826708, 22.5 },
        { 0.884415, 23 },
        { 0.944988, 23.5 },
        { 1.008000, 24 },
        { 1.072000, 24.5 },
        { 1.141000, 25 },
        { 1.217000, 25.5 },
        { 1.305000, 26 },
        { 1.403000, 26.5 },
        { 1.508000, 27 },
        { 1.610000, 27.5 },
        { 1.704000, 28 },
        { 1.797000, 28.5 },
        { 1.893000, 29 },
        { 1.990000, 29.5 },
        { 2.084000, 30 },
        { 2.180000, 30.5 },
        { 2.281000, 31 },
        { 2.392000, 31.5 },
        { 2.500000, 32 },
        { 2.585000, 32.5 },
        { 2.663000, 33 },
        { 2.735000, 33.5 },
        { 2.803000, 34 },
        { 2.892000, 34.5 },
        { 2.922000, 34.95 }
      };

void RFPA5201_Amp::HWInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* PENABLE */
    // Note: Use external pull-down resistor (1k) on PENABLE to ensure amplifier is OFF at power-on
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = AMP_PENABLE_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
    GPIO_Init(AMP_PENABLE_PORT, &GPIO_InitStructure);
}

void RFPA5201_Amp::SetEnabled(bool enable)
{
    if(enable)
        GPIO_SetBits(AMP_PENABLE_PORT, AMP_PENABLE_PIN);
    else
        GPIO_ResetBits(AMP_PENABLE_PORT, AMP_PENABLE_PIN);

    /* Set internal flag */
    enabled = enable;
}

float RFPA5201_Amp::GetOutputPower_dBm(float pDETVoltage)
{
    int imin = 0;
    int imax = N_SAMPLES;
    int imid;

    /* If amplifier is disabled, output power is zero */
    if(!enabled)
        return 0.00;

    while(imax > imin+1)
    {
        /* Calculate midpoint index of search interval */
        /* Note: Midpoint value will be rounded up to nearest integer if remainder >= 0.5 */
        imid = (imax-imin)/2 + imin;

        /* Calculate new index bounds of search interval */
        if(dataPoints[imid][0] < pDETVoltage)
            imin = imid;
        else if(dataPoints[imid][0] > pDETVoltage)
            imax = imid;
        /* Input voltage exactly matches the value in the LUT (highly unlikely) */
        else
            return dataPoints[imid][1];
    }

    /* Return value at imax */
    if(fabs(dataPoints[imin][0]-pDETVoltage) > fabs(dataPoints[imax][0]-pDETVoltage))
        return dataPoints[imax][1];
    /* Return value at imin */
    else
        return dataPoints[imin][1];
}
