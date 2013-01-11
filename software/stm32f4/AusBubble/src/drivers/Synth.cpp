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
    SynthWrite((REG_SDI_CTRL<<16) | (1<<SHIFT_RESET));      // [1] When this bit is taken high the part is reset

    /* Configure device */
    // Set GPO4 to output LOCK flag
    SynthWrite((REG_GPO<<16) | (1<<SHIFT_LOCK));            // [0] Sends LOCK flag to GPO4
    // Enable frequency modulation
    SynthWrite((REG_EXT_MOD<<16) | (1<<SHIFT_MODSETUP) |    // [15:14] Modulation is analog, on every update of modulation the frac-N responds by adding value to frac-N
                                   (1<<SHIFT_MODSTEP));     // [13:10] Modulation scale factor. Modulation is multiplied by 2^modstep before being added to frac-N. Maximum usable value is 8
    // Bypass the mixer
    SynthWrite((REG_DEV_CTRL<<16) | (1<<SHIFT_BYPASS));     // [1] If high, offsets mixer so that LO signal can be viewed at mixer output

    /* Set frequency to 2450 MHz */
    SynthSetFreq(2450, true);
}

void SynthEnable(bool enable)
{
    #if USE_SW_CONTROL
        SynthWrite((REG_SDI_CTRL<<16) | (1<<SHIFT_SIPIN) |       // [15] 1=ENBL and MODE pins are ignored and become available as GPO5 and GPO6
                                        (enable<<SHIFT_ENBL));   // [14] If sipin=1 this field will replace the functionality of the ENBL pin
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

    // Set SDATA as input
    SYNTH_SDATA_PORT->MODER  &= ~(GPIO_MODER_MODER0 << (SYNTH_SDATA_PIN_N * 2));
    SYNTH_SDATA_PORT->MODER |= (((uint32_t)GPIO_Mode_IN) << (SYNTH_SDATA_PIN_N * 2));

    while (count < 16)
    {
        // Build data word, MSB read back first
        asm volatile("nop");
        GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_RESET);
        read_data |= ((SYNTH_SDATA_PORT->IDR & SYNTH_SDATA_PIN) != 0) << (15 - count);
        asm volatile("nop");
        GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_SET);
        count++;
    }

    // Reception complete
    GPIO_WriteBit(SYNTH_ENX_PORT, SYNTH_ENX_PIN, Bit_SET);
    asm volatile("nop");
    GPIO_WriteBit(SYNTH_SCLK_PORT, SYNTH_SCLK_PIN, Bit_RESET);

    // Set SDATA as output
    SYNTH_SDATA_PORT->MODER  &= ~(GPIO_MODER_MODER0 << (SYNTH_SDATA_PIN_N * 2));
    SYNTH_SDATA_PORT->MODER |= (((uint32_t)GPIO_Mode_OUT) << (SYNTH_SDATA_PIN_N * 2));

    return read_data;
}

uint16_t SynthRead(uint8_t address)
{
    SynthSendAddress(false, address);
    return SynthReceiveData();
}

void SynthSetFreq(float f_lo, bool waitForLock)
{
    /* Register calculations taken from RFMD Programming Guide
    Source: http://www.rfmd.com/CS/Documents/IntegratedSyntMixerProgrammingGuide.pdf */
    int n_lo        = log2f((float)(F_VCO_MAX_MHZ/f_lo));
    int lodiv       = 1<<n_lo;
    int f_vco       = lodiv*f_lo;
    float n_div     = f_vco/(float)(FBKDIV*F_REFERENCE_MHZ);
    int n           = n_div;
    double nummsb;
    float fraction  = modf((1<<16)*(n_div-n), &nummsb);
    uint16_t numlsb = (1<<8)*fraction;

    // Disable device
    SynthEnable(false);

    // Set N divider, LO path divider and feedback divider
    SynthWrite((REG_P1_FREQ1<<16) | (n<<SHIFT_NDIV) |
                                    ((int)log2(lodiv)<<SHIFT_LODIV) |
                                    (2<<SHIFT_PRESC));
    SynthWrite((REG_P1_FREQ2<<16) | (int)nummsb);
    SynthWrite((REG_P1_FREQ3<<16) | numlsb);

    /* Reset FMOD (so that the desired frequency is set if frequency modulation is/was being used)
    Note: This register sets the Frequency Deviation applied to frac-N */
    SynthWrite((REG_FMOD<<16) | 0); // [15:0] Frequency Deviation applied to frac-N, functionality determined by modstep and mod_setup

    // Re-lock the PLL
    SynthWrite((REG_PLL_CTRL<<16) | (1<<SHIFT_DIVBY) |  // [15] Force reference divider to divide by 1
                                    (8<<SHIFT_TVCO) |   // [10:6] VCO warm-up time. warm-up time [s] = tvco * 1/[fref*256]
                                    (1<<SHIFT_LDEN) |   // [5] Enable lock detector circuitry
                                    (1<<SHIFT_RELOK));  // [3] Self Clearing Bit. When this bit is set high it triggers a relock of the PLL and then clears

    // Enable device
    SynthEnable(true);

    // Wait for lock
    while(waitForLock && ((SYNTH_GPO4LDDO_PORT->IDR & SYNTH_GPO4LDDO_PIN) != SYNTH_GPO4LDDO_PIN));
}
