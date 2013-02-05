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
    // Bypass the mixer
    SynthWrite((REG_DEV_CTRL<<16) | (1<<SHIFT_BYPASS));     // [1] If high, offsets mixer so that LO signal can be viewed at mixer output

    /* Set frequency to 2450 MHz */
    SynthSetFreq(2450000000);
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

void SynthSetFreqLO(uint64_t f_lo_Hz, bool waitForLock, uint16_t &nummsb_ref, uint16_t &numlsb_ref)
{
    /* Register calculations taken from RFMD Programming Guide
    Source: http://www.rfmd.com/CS/Documents/IntegratedSyntMixerProgrammingGuide.pdf */
    int n_lo        = log2f((float)(F_VCO_MAX_HZ/f_lo_Hz));
    int lodiv       = 1<<n_lo;
    uint64_t f_vco  = lodiv*f_lo_Hz;
    /* If the VCO frequency is above 3.2GHz it is necessary to set the prescaler to /4
    and charge pump leakage to 3 for the CT_cal to work correctly */
    int fbkdiv;
    if(f_vco > 3200000000)
    {
        fbkdiv      = FBKDIV_4;
        SynthWrite((REG_LF<<16) | (1<<SHIFT_LFACT)      // Active loop filter enable, 1=active 0=passive
                                | (32<<SHIFT_P2CPDEF)   // Charge pump setting. If p2_kv_en=1 this value sets charge pump current during KV compensation measurement. If p2_kv_en=0, this value is used at all times. Default value is 93uA.
                                | (32<<SHIFT_P1CPDEF)   // Charge pump setting. If p1_kv_en=1 this value sets charge pump current during KV compensation measurement. If p1_kv_en=0, this value is used at all times. Default value is 93uA.
                                | (3<<SHIFT_PLLCPL));   // Charge pump leakage settings
    }
    else
    {
        fbkdiv      = FBKDIV_2;
        SynthWrite((REG_LF<<16) | (1<<SHIFT_LFACT)      // Active loop filter enable, 1=active 0=passive
                                | (32<<SHIFT_P2CPDEF)   // Charge pump setting. If p2_kv_en=1 this value sets charge pump current during KV compensation measurement. If p2_kv_en=0, this value is used at all times. Default value is 93uA.
                                | (32<<SHIFT_P1CPDEF)   // Charge pump setting. If p1_kv_en=1 this value sets charge pump current during KV compensation measurement. If p1_kv_en=0, this value is used at all times. Default value is 93uA.
                                | (2<<SHIFT_PLLCPL));   // Charge pump leakage settings
    }
    float n_div     = f_vco/(float)(fbkdiv*F_REFERENCE_HZ);
    int n           = n_div;
    double nummsb;
    float fraction  = modf((1<<16)*(n_div-n), &nummsb);
    uint16_t numlsb = (1<<8)*fraction;
    // Save values
    nummsb_ref = (uint16_t) nummsb;
    numlsb_ref = numlsb;

    // Disable device
    SynthEnable(false);

    // Set N divider, LO path divider and feedback divider
    SynthWrite((REG_P1_FREQ1<<16) | (n<<SHIFT_N) |                      // Path 1 VCO divider integer value
                                    ((int)log2(lodiv)<<SHIFT_LODIV) |   // Path 1 LO path divider setting: divide by 2^n (i.e. divide by 1 to divide by 32). 110 and 111 are reserved
                                    ((fbkdiv>>1)<<SHIFT_PRESC));        // Path 1 VCO PLL feedback path divider setting: 01 = divide by 2, 10 = divide by 4 (00 and 11 are reserved)
    SynthWrite((REG_P1_FREQ2<<16) | (uint16_t)nummsb);                  // Path 1 N divider numerator value, most significant 16 bits
    SynthWrite((REG_P1_FREQ3<<16) | numlsb);                            // Path 1 N divider numerator value, least significant 8 bits

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

void SynthSetFreq(uint64_t freq_Hz)
{
    static uint64_t f_lo_Hz = 0;
    static uint64_t freq_prev_Hz = 0;
    static int16_t fmod_lower_bound = 0;
    static int16_t fmod_upper_bound = 0;
    static int16_t cur_fmod = 0;
    static int16_t fmod_step = 0;
    static uint16_t nummsb = 0;
    static uint16_t numlsb = 0;
    static int32_t freq_delta_Hz = 0;
    static int32_t cur_freq_delta_Hz = 0;

    /* FREQUENCY MODULATION */
    // Check 1 :                                            : Is the PLL still locked?
    // Check 2 : fmod_step != 0                             : Valid frequency delta (i.e. step size)
    // Check 3 : (freq_Hz - freq_prev_Hz) == freq_delta_Hz  : Has frequency step size or direction changed? (i.e. are modulation setting still valid?)
    // Check 4 : (cur_fmod + fmod_step) <= fmod_upper_bound : New modulation setting less than or equal to upper bound
    // Check 5 : (cur_fmod + fmod_step) >= fmod_lower_bound : New modulation setting greater than or equal to lower bound
    cur_freq_delta_Hz = freq_Hz - freq_prev_Hz;
    if( ((SYNTH_GPO4LDDO_PORT->IDR & SYNTH_GPO4LDDO_PIN) == SYNTH_GPO4LDDO_PIN) &&
        (fmod_step != 0) &&
        (cur_freq_delta_Hz == freq_delta_Hz) &&
        ((cur_fmod + fmod_step) <= fmod_upper_bound) &&
        ((cur_fmod + fmod_step) >= fmod_lower_bound))
    {
        cur_fmod += fmod_step;
        SynthWrite((REG_FMOD<<16) | cur_fmod); // [15:0] Frequency Deviation applied to frac-N, functionality determined by modstep and mod_setup
    }
    /* SET FREQUENCY BY WRITING TO FREQ1, FREQ2, FREQ3 REGISTERS */
    else
    {
        // Set frequency (wait for PLL lock)
        f_lo_Hz = freq_Hz;
        SynthSetFreqLO(f_lo_Hz, true, nummsb, numlsb);
        cur_fmod = 0; // FMOD reset to 0
        // Set optimum modulation parameters depending on frequency delta
        // Note: Frequency modulation is only used for valid frequency deltas (i.e. step sizes)
        uint8_t modstep;
        freq_delta_Hz = freq_Hz - freq_prev_Hz;
        SynthGetModParams(freq_delta_Hz, modstep, fmod_step);
        // Valid frequency delta
        if(fmod_step != 0)
        {
            uint32_t n_24bit = (nummsb<<8) | (numlsb>>8);
            uint32_t max_fmod = 0x7FFF << modstep; // FMOD is multiplied by 2^MODSTEP before being added to frac-N
            // LOWER BOUND
            if(max_fmod <= n_24bit)
                fmod_lower_bound = -1*0x7FFF;
            else
                fmod_lower_bound = -1*((n_24bit & max_fmod) >> modstep);
            // UPPER BOUND
            if((n_24bit + max_fmod) <= 0xFFFFFF)
                fmod_upper_bound = 0x7FFF;
            else
                fmod_upper_bound = ((0xFFFFFF-n_24bit) & max_fmod) >> modstep;

            // Enable frequency modulation
            SynthWrite((REG_EXT_MOD<<16) | (1<<SHIFT_MODSETUP) |        // [15:14] Modulation is analog, on every update of modulation the frac-N responds by adding value to frac-N
                                           (modstep<<SHIFT_MODSTEP));   // [13:10] Modulation scale factor. Modulation is multiplied by 2^modstep before being added to frac-N. Maximum usable value is 8
        }
    }
    freq_prev_Hz = freq_Hz;
}

void SynthGetModParams(int32_t freq_delta_Hz, uint8_t &modstep, int16_t &fmod_step)
{
    /*
          Fmod = MOD * (2^MODSTEP) * step_size

          where     step_size  =   (Fpd * P) / (R * (2^24) * LO_DIV)

          where     Fpd        =   Phase detector frequency
                    MODSTEP    =   Modulation scale factor
                    MOD        =   Frequency deviation applied to frac-N
                    P          =   Prescaler division ratio    (VARIABLE:   2,4)
                    R          =   Reference division ratio    (FIXED:      1)
                    LO_DIV     =   Low divider value           (VARIABLE:   2,4,8,16,32)
    */

    // Assume P=4, R=1, LO_DIV=2

    // 1 kHz
    // f = 5 * (2^6) * ( (26e6 * 4) / (1 * (2^24) * 2) ) = 991.82 Hz
    if(abs(freq_delta_Hz) == STEP_1K_HZ)
    {
        modstep = 6;
        fmod_step = 5;
    }
    // 10 kHz
    // f = 25 * (2^7) * ( (26e6 * 4) / (1 * (2^24) * 2) ) = 9918.21 Hz
    else if(abs(freq_delta_Hz) == STEP_10K_HZ)
    {
        modstep = 7;
        fmod_step = 25;
    }
    // 25 kHz
    // f = 63 * (2^7) * ( (26e6 * 4) / (1 * (2^24) * 2) ) = 24993.90 Hz
    else if(abs(freq_delta_Hz) == STEP_25K_HZ)
    {
        modstep = 7;
        fmod_step = 63;
    }
    // 50 kHz
    // f = 63 * (2^8) * ( (26e6 * 4) / (1 * (2^24) * 2) ) = 49987.79 Hz
    else if(abs(freq_delta_Hz) == STEP_50K_HZ)
    {
        modstep = 8;
        fmod_step = 63;
    }
    // 100 kHz
    // f = 126 * (2^8) * ( (26e6 * 4) / (1 * (2^24) * 2) ) = 99975.59 Hz
    else if(abs(freq_delta_Hz) == STEP_100K_HZ)
    {
        modstep = 7;
        fmod_step = 512;
    }
    // 250 kHz
    // f = 315 * (2^8) * ( (26e6 * 4) / (1 * (2^24) * 2) ) = 249938.96 Hz
    else if(abs(freq_delta_Hz) == STEP_250K_HZ)
    {
        modstep = 8;
        fmod_step = 315;
    }
    // 500 kHz
    // f = 630 * (2^8) * ( (26e6 * 4) / (1 * (2^24) * 2) ) = 499877.93 Hz
    else if(abs(freq_delta_Hz) == STEP_500K_HZ)
    {
        modstep = 8;
        fmod_step = 630;
    }
    // 1 MHz
    // f = 1260 * (2^8) * ( (26e6 * 4) / (1 * (2^24) * 2) ) = 999755.86 Hz
    else if(abs(freq_delta_Hz) == STEP_1M_HZ)
    {
        modstep = 8;
        fmod_step = 1260;
    }
    // Unsupported frequency delta
    else
    {
        fmod_step = 0;
    }

    // Set sign of step
    fmod_step *= (freq_delta_Hz > 0 ? 1 : -1);
}
