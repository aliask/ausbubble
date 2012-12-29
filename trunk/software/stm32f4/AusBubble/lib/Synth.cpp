/************************************************************************/
/* AusBubble:                                                           */
/* An open-source RF jammer designed to operate in the 2.4 GHz Wi-Fi    */
/* frequency block.                                                     */
/*                                                                      */
/* Synth.cpp                                                            */
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

#include "Synth.h"

void SynthInit(void)
{
    /* Set initial state of GPIO pins and perform hardware reset */
    // RESETX=0
	GPIO_ResetBits(SYNTH_RESETX_PORT, SYNTH_RESETX_PIN);
    // ENX=1
	GPIO_SetBits(SYNTH_ENX_PORT, SYNTH_ENX_PIN);
    // SCLK=0
	GPIO_ResetBits(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN);
    // SDATA=0
	GPIO_ResetBits(SYNTH_SDATA_PORT, SYNTH_SDATA_PIN);
    // ENBL=0
	GPIO_ResetBits(SYNTH_ENBLGPO5_PORT, SYNTH_ENBLGPO5_PIN);
    // RESETX=1
	GPIO_SetBits(SYNTH_RESETX_PORT, SYNTH_RESETX_PIN);

    /* Software Reset */
    SynthWrite(REG_SDI_CTRL | (1<<SHIFT_RESET));

    /* Configure device */
    // Set GPO4 LED (Lock)
    SynthWrite(REG_GPO | (1<<SHIFT_LOCK));
    // Enable modulation: MODULATION<<7
    SynthWrite(REG_EXT_MOD | (1<<SHIFT_MODSETUP) |
                             (7<<SHIFT_MODSTEP));
    // Bypass the mixer
    SynthWrite(REG_DEV_CTRL | (1<<SHIFT_BYPASS));
    // Enable device
    SynthEnable(true);
}

void SynthEnable(bool enable)
{
    #if USE_SW_CONTROL
        SynthWrite(REG_SDI_CTRL | (1<<SHIFT_SIPIN) |       // SW control
                                  (enable<<SHIFT_ENBL));   // Enable
    #else
        if(enable)
        	GPIO_WriteBit(SYNTH_ENBLGPO5_PORT, SYNTH_ENBLGPO5_PIN, Bit_SET);
        else
        	GPIO_WriteBit(SYNTH_ENBLGPO5_PORT, SYNTH_ENBLGPO5_PIN, Bit_RESET);
    #endif
}

void SynthWrite(uint32_t dataBits)
{
    // Initialize count variable to zero
    uint8_t count = 0;

    // Pulse SCLK
    GPIO_SetBits(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN);
    GPIO_ResetBits(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN);

    // ENX=0
    GPIO_ResetBits(SYNTH_ENX_PORT, SYNTH_ENX_PIN);

    // Pulse SCLK (for don't care SDATA bit)
    GPIO_SetBits(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN);
    GPIO_ResetBits(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN);

    // SDATA=0 (WRITE MODE)
    GPIO_ResetBits(SYNTH_SDATA_PORT, SYNTH_SDATA_PIN);
    asm volatile("nop");

    // Pulse SCLK
    GPIO_SetBits(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN);
    GPIO_ResetBits(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN);

    // Put 23 bits on SDATA line (MSB first)
    while(count < 23)
    {
        if(dataBits & (0x00400000 >> count))
        {
        	// SDATA=1
        	GPIO_SetBits(SYNTH_SDATA_PORT, SYNTH_SDATA_PIN);
        	asm volatile("nop");

            // Pulse SCLK
            GPIO_SetBits(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN);
            GPIO_ResetBits(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN);
        }
        else
        {
        	// SDATA=0
        	GPIO_ResetBits(SYNTH_SDATA_PORT, SYNTH_SDATA_PIN);
        	asm volatile("nop");

            // Pulse SCLK
            GPIO_SetBits(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN);
            GPIO_ResetBits(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN);
        }
        count++;
    }

    // ENX=1
    GPIO_SetBits(SYNTH_ENX_PORT, SYNTH_ENX_PIN);

    // Pulse SCLK
    GPIO_SetBits(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN);
    GPIO_ResetBits(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN);
}

void SynthSendAddress(bool write, uint8_t address)
{
    // Initialize count variable to zero
    uint8_t count = 0;

    // Pulse SCLK
    GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_SET);
    GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_RESET);

    // ENX=0
    GPIO_WriteBit(SYNTH_ENX_PORT, SYNTH_ENX_PIN, Bit_RESET);

    // Pulse SCLK (for don't care SDATA bit)
    GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_SET);
    GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_RESET);

    // R=0 (WRITE MODE)
    if(write)
    {
        // SDATA=0
    	GPIO_WriteBit(SYNTH_SDATA_PORT, SYNTH_SDATA_PIN, Bit_RESET);
        asm volatile("nop");

        // Pulse SCLK
        GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_SET);
        GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_RESET);
    }
    // R=1 (READ MODE)
    else
    {
        // SDATA=1
    	GPIO_WriteBit(SYNTH_SDATA_PORT, SYNTH_SDATA_PIN, Bit_SET);
        asm volatile("nop");

        // Pulse SCLK
        GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_SET);
        GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_RESET);
    }

    // Put 7 bits on SDATA line (MSB first)
    while(count < 7)
    {
        if(address & (0x40 >> count))
        {
            // SDATA=1
        	GPIO_WriteBit(SYNTH_SDATA_PORT, SYNTH_SDATA_PIN, Bit_SET);
            asm volatile("nop");

            // Pulse SCLK
            GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_SET);
            GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_RESET);
        }
        else
        {
            // SDATA=0
        	GPIO_WriteBit(SYNTH_SDATA_PORT, SYNTH_SDATA_PIN, Bit_RESET);
            asm volatile("nop");

            // Pulse SCLK
            GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_SET);
            GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_RESET);
        }
        count++;
    }

    // If reading device, insert extra clock edges
    if(!write)
    {
        asm volatile("nop");
        GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_SET);
        GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_RESET);
        GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_SET);
    }
}

void SynthSendData(uint16_t data)
{
    // Initialize count variable to zero
    uint8_t count = 0;

    // Put 16 bits on SDATA line (MSB first)
    while (count < 16)
    {
        if (data & (0x8000 >> count))
        {
            // SDATA=1
        	GPIO_WriteBit(SYNTH_SDATA_PORT, SYNTH_SDATA_PIN, Bit_SET);
            asm volatile("nop");

            // Pulse SCLK
            GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_SET);
            GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_RESET);
        }
        else
        {
            // SDATA=0
        	GPIO_WriteBit(SYNTH_SDATA_PORT, SYNTH_SDATA_PIN, Bit_RESET);
            asm volatile("nop");

            // Pulse SCLK
            GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_SET);
            GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_RESET);
        }
        count++;
    }

    // ENX=1
    GPIO_WriteBit(SYNTH_ENX_PORT, SYNTH_ENX_PIN, Bit_SET);

    // Pulse SCLK
    GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_SET);
    GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_RESET);
}

uint16_t SynthReceiveData(void)
{
    // Initialize local variables
    uint8_t count = 0;
    uint16_t read_data = 0;

    // TODO: Set SDATA as input

    while (count < 16)
    {
        // Build data word, MSB read back first
        asm volatile("nop");
        GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_RESET);
        read_data = (read_data << 1) | ((SYNTH_SDATA_PORT->IDR & SYNTH_SDATA_PIN) == 1);
        asm volatile("nop");
        GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_SET);
        count++;
    }

    // Reception complete
    GPIO_WriteBit(SYNTH_ENX_PORT, SYNTH_ENX_PIN, Bit_SET);
    asm volatile("nop");
    GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_RESET);

    // TODO: Set SDATA as output

    return read_data;
}

void SynthSetFreq(float f_lo)
{
    /* Register calculations taken from RFMD Programming Guide
    Source: http://www.rfmd.com/CS/Documents/IntegratedSyntMixerProgrammingGuide.pdf */
    int n_lo 		= log2f((float)(F_VCO_MAX_MHZ/f_lo));
    int lodiv 		= 1<<n_lo;
    int f_vco 		= lodiv*f_lo;
    float n_div 	= f_vco/(float)(FBKDIV*F_REFERENCE_MHZ);
    int n 			= n_div;
    double nummsb;
    float fraction 	= modf((1<<16)*(n_div-n), &nummsb);
    uint16_t numlsb = (1<<8)*fraction;

    // Set N divider, LO path divider and feedback divider
    SynthWrite(REG_P1_FREQ1 | (n<<SHIFT_NDIV) |
                              ((int)log2(lodiv)<<SHIFT_LODIV) |
                              (2<<SHIFT_PRESC));

    SynthWrite(REG_P1_FREQ2 | (int)nummsb);
    SynthWrite(REG_P1_FREQ3 | numlsb);

    /* Reset FMOD (so that the desired frequency is set if frequency modulation is/was being used)
    Note: This register sets the Frequency Deviation applied to frac-N */
    SynthWrite(REG_FMOD | 0);

    // Re-lock the PLL
    SynthWrite(REG_PLL_CTRL | (1<<SHIFT_DIVBY) |
                              (8<<SHIFT_TVCO) |
                              (1<<SHIFT_LDEN) |
                              (1<<SHIFT_RELOK));
}
