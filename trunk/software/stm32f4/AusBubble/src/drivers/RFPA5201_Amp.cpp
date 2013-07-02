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

void RFPA5201_Amp::HWInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // PENABLE
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
    /* RFPA5201 EVM: 11n MCS7 HT40; Vcc=5v; Vreg=2.9v; Temp=25degC; Duty Cycle=50%; f=2450MHz */
    /* 70 data points: Residual Sum of Squares: rss = 83.9760863 */
    /* From: http://www.xuru.org/rt/LnR.asp */
    return 8.906874645*log(pDETVoltage) + 24.24875014;
}
