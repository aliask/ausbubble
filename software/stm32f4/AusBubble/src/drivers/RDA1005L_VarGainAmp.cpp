/************************************************************************/
/* AusBubble:                                                           */
/* An open-source RF jammer designed to operate in the 2.4 GHz Wi-Fi    */
/* frequency block.                                                     */
/*                                                                      */
/* RDA1005L_VarGainAmp.cpp                                              */
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

#include "RDA1005L_VarGainAmp.h"

void RDA1005L_VarGainAmp::HWInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // PUP
    // Note: Use external pull-down resistor (1k) on PUP to ensure attenuation is at maximum at power-on
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = VARGAINAMP_PUP_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
    GPIO_Init(VARGAINAMP_PUP_PORT, &GPIO_InitStructure);
    // Initially set LOW (Attenuation at Max, 31.5dB)
    GPIO_ResetBits(VARGAINAMP_PUP_PORT, VARGAINAMP_PUP_PIN);
    // CLK
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = VARGAINAMP_CLK_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
    GPIO_Init(VARGAINAMP_CLK_PORT, &GPIO_InitStructure);
    // Initially set LOW
    GPIO_ResetBits(VARGAINAMP_CLK_PORT, VARGAINAMP_CLK_PIN);
    // DATA
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = VARGAINAMP_DATA_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
    GPIO_Init(VARGAINAMP_DATA_PORT, &GPIO_InitStructure);
    // Initially set LOW
    GPIO_ResetBits(VARGAINAMP_DATA_PORT, VARGAINAMP_DATA_PIN);
    // LE
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = VARGAINAMP_LE_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
    GPIO_Init(VARGAINAMP_LE_PORT, &GPIO_InitStructure);
    // Initially set LOW
    GPIO_ResetBits(VARGAINAMP_LE_PORT, VARGAINAMP_LE_PIN);
}

void RDA1005L_VarGainAmp::SetGain(double gain_dB)
{
    /* This function accepts a gain value between VARGAINAMP_MIN_GAIN_LIMIT_DB and VARGAINAMP_MAX_GAIN_LIMIT_DB
    Note: Value is written to the amp in steps of 0.5dB */

    double gainRelToMaxAbs;
    uint8_t stepN;

    /* Convert gain input to gain relative to maximum (absolute value) */
    // Saturate gain value if HIGHER than defined limit
    if(gain_dB >= VARGAINAMP_MAX_GAIN_LIMIT_DB)
      gainRelToMaxAbs=VARGAINAMP_MAX_GAIN_DB-VARGAINAMP_MAX_GAIN_LIMIT_DB;
    // Saturate gain value if LOWER than defined limit
    else if(gain_dB <= VARGAINAMP_MIN_GAIN_LIMIT_DB)
      gainRelToMaxAbs=-1*(VARGAINAMP_MIN_GAIN_DB-VARGAINAMP_MAX_GAIN_DB);
    else
      gainRelToMaxAbs=fabs(gain_dB-VARGAINAMP_MAX_GAIN_DB);

    // Calculate step number (0 to 63)
    stepN = round(gainRelToMaxAbs*2.0);

    // Calculate correct bit-pattern and write
    // MAX RELATIVE GAIN: 0dB = 6b'111111
    // MIN RELATIVE GAIN: -31.5dB = 6b'000000
    Write((~stepN) & 0x3F);
}

void RDA1005L_VarGainAmp::Write(uint8_t data)
{
    // Initialize count variable to zero
    uint8_t count = 0;

    // Pulse LE
    GPIO_ResetBits(VARGAINAMP_LE_PORT, VARGAINAMP_LE_PIN);
    GPIO_SetBits(VARGAINAMP_LE_PORT, VARGAINAMP_LE_PIN);
    GPIO_ResetBits(VARGAINAMP_LE_PORT, VARGAINAMP_LE_PIN);

    // Put 6 bits on DATA line (MSB first)
    while (count < 6)
    {
        if (data & (0x20 >> count))
        {
            // DATA=1
            GPIO_SetBits(VARGAINAMP_DATA_PORT, VARGAINAMP_DATA_PIN);
            asm volatile("nop");

            // Pulse CLK
            GPIO_SetBits(VARGAINAMP_CLK_PORT, VARGAINAMP_CLK_PIN);
            GPIO_ResetBits(VARGAINAMP_CLK_PORT, VARGAINAMP_CLK_PIN);
        }
        else
        {
            // DATA=0
            GPIO_ResetBits(VARGAINAMP_DATA_PORT, VARGAINAMP_DATA_PIN);
            asm volatile("nop");

            // Pulse CLK
            GPIO_SetBits(VARGAINAMP_CLK_PORT, VARGAINAMP_CLK_PIN);
            GPIO_ResetBits(VARGAINAMP_CLK_PORT, VARGAINAMP_CLK_PIN);
        }
        count++;
    }

    // Pulse LE
    GPIO_SetBits(VARGAINAMP_LE_PORT, VARGAINAMP_LE_PIN);
    GPIO_ResetBits(VARGAINAMP_LE_PORT, VARGAINAMP_LE_PIN);
}
