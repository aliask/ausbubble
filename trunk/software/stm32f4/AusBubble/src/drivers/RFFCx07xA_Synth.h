/************************************************************************/
/* AusBubble:                                                           */
/* An open-source RF jammer designed to operate in the 2.4 GHz Wi-Fi    */
/* frequency block.                                                     */
/*                                                                      */
/* RFFCx07xA_Synth.h                                                    */
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

#ifndef RFFCX07XA_SYNTH_H
#define RFFCX07XA_SYNTH_H

#include "Includes.h"

#define F_VCO_MAX_HZ                5400000000
#define F_REFERENCE_HZ              26000000
#define FBKDIV_2                    2       // Prescaler divider (fvco<3.2GHz: 2)
#define FBKDIV_4                    4       // Prescaler divider (fvco>3.2GHz: 4)
#define USE_SW_CONTROL              0       // Software Control=1, Hardware Control=0 (via ENBL and MODE pins)

// Synthesizer registers
#define REG_LF                      0x00
#define REG_XO                      0x01
#define REG_CAL_TIME                0x02
#define REG_VCO_CTRL                0x03
#define REG_CT_CAL1                 0x04
#define REG_CT_CAL2                 0x05
#define REG_PLL_CAL1                0x06
#define REG_PLL_CAL2                0x07
#define REG_VCO_AUTO                0x08
#define REG_PLL_CTRL                0x09
#define REG_PLL_BIAS                0x0A
#define REG_MIX_CONT                0x0B    // RFFC parts only
#define REG_P1_FREQ1                0x0C
#define REG_P1_FREQ2                0x0D
#define REG_P1_FREQ3                0x0E
#define REG_P2_FREQ1                0x0F
#define REG_P2_FREQ2                0x10
#define REG_P2_FREQ3                0x11
#define REG_FN_CTRL                 0x12
#define REG_EXT_MOD                 0x13
#define REG_FMOD                    0x14
#define REG_SDI_CTRL                0x15
#define REG_GPO                     0x16
#define REG_T_VCO                   0x17    // Revision 2 devices only (mrev_id = 010): RFFC2071A/2072A/5071A/5072A
#define REG_IQMOD1                  0x18    // RFMD208x only
#define REG_IQMOD2                  0x19    // RFMD208x only
#define REG_IQMOD3                  0x1A    // RFMD208x only
#define REG_IQMOD4                  0x1B    // RFMD208x only
#define REG_T_CTRL                  0x1C    // Revision 2 devices only (mrev_id = 010): RFFC2071A/2072A/5071A/5072A
#define REG_DEV_CTRL                0x1D
#define REG_TEST                    0x1E
#define REG_READBACK                0x1F

// LF fields (0x00)
#define SHIFT_LF__LFACT             15
#define SHIFT_LF__P2CPDEF           9
#define SHIFT_LF__P1CPDEF           3
#define SHIFT_LF__PLLCPL            0

// XO fields (0x01)
#define SHIFT_XO__XOCH              15
#define SHIFT_XO__XOC               11
#define SHIFT_XO__XOCF              10
#define SHIFT_XO__SUWAIT            0

// CAL_TIME fields (0x02)
#define SHIFT_CAL_TIME__WAIT        15
#define SHIFT_CAL_TIME__TCT         10
#define SHIFT_CAL_TIME__TKV1        4
#define SHIFT_CAL_TIME__TKV2        0

// VCO_CTRL fields (0x03)
#define SHIFT_VCO_CTRL__XTVCO       15
#define SHIFT_VCO_CTRL__CTAVG       13
#define SHIFT_VCO_CTRL__CTPOL       12
#define SHIFT_VCO_CTRL__CLKPOL      11
#define SHIFT_VCO_CTRL__KVAVG       9
#define SHIFT_VCO_CTRL__KVRNG       8
#define SHIFT_VCO_CTRL__KVPOL       7
#define SHIFT_VCO_CTRL__XOI1        6
#define SHIFT_VCO_CTRL__XOI2        5
#define SHIFT_VCO_CTRL__XOI3        4
#define SHIFT_VCO_CTRL__REFST       3
#define SHIFT_VCO_CTRL__ICPUP       1

// CT_CAL1 fields (0x04)
#define SHIFT_CT_CAL1__P1CTGAIN     13
#define SHIFT_CT_CAL1__P1CTV        8
#define SHIFT_CT_CAL1__P1CT         7
#define SHIFT_CT_CAL1__P1CTDEF      0

// CT_CAL2 fields (0x05)
#define SHIFT_CT_CAL2__P2CTGAIN     13
#define SHIFT_CT_CAL2__P2CTV        8
#define SHIFT_CT_CAL2__P2CT         7
#define SHIFT_CT_CAL2__P2CTDEF      0

// PLL_CAL1 fields (0x06)
#define SHIFT_PLL_CAL1__P1KV        15
#define SHIFT_PLL_CAL1__P1DN        6
#define SHIFT_PLL_CAL1__P1KVGAIN    3
#define SHIFT_PLL_CAL1__P1SGN       2

// PLL_CAL2 fields (0x07)
#define SHIFT_PLL_CAL2__P2KV        15
#define SHIFT_PLL_CAL2__P2DN        6
#define SHIFT_PLL_CAL2__P2KVGAIN    3
#define SHIFT_PLL_CAL2__P2SGN       2

// VCO_AUTO fields (0x08)
#define SHIFT_VCO_AUTO__AUTO        15
#define SHIFT_VCO_AUTO__CTMAX       8
#define SHIFT_VCO_AUTO__CTMIN       1

// PLL_CTRL fields (0x09)
#define SHIFT_PLL_CTRL__DIVBY       15
#define SHIFT_PLL_CTRL__CLKDIV      12
#define SHIFT_PLL_CTRL__PLLST       11
#define SHIFT_PLL_CTRL__TVCO        6
#define SHIFT_PLL_CTRL__LDEN        5
#define SHIFT_PLL_CTRL__LDLEV       4
#define SHIFT_PLL_CTRL__RELOK       3
#define SHIFT_PLL_CTRL__ALOI        2
#define SHIFT_PLL_CTRL__PLLDY       0

// PLL_BIAS fields (0x0A)
#define SHIFT_PLL_BIAS__P1LOI       11
#define SHIFT_PLL_BIAS__P1VCOI      8
#define SHIFT_PLL_BIAS__P2LOI       3
#define SHIFT_PLL_BIAS__P2VCOI      0

// MIX_CONT fields (0x0B)
#define SHIFT_MIX_CONT__FULLD       15
#define SHIFT_MIX_CONT__P1MIXIDD    12
#define SHIFT_MIX_CONT__P2MIXIDD    9

// P1_FREQ1 fields (0x0C)
#define SHIFT_P1_FREQ1__P1N         7
#define SHIFT_P1_FREQ1__P1LODIV     4
#define SHIFT_P1_FREQ1__P1PRESC     2
#define SHIFT_P1_FREQ1__P1VCOSEL    0

// P1_FREQ2 fields (0x0D)
#define SHIFT_P1_FREQ2__P1NMSB      0

// P1_FREQ3 fields (0x0E)
#define SHIFT_P1_FREQ3__P1NLSB      8

// P2_FREQ1 fields (0x0F)
#define SHIFT_P2_FREQ1__P2N         7
#define SHIFT_P2_FREQ1__P2LODIV     4
#define SHIFT_P2_FREQ1__P2PRESC     2
#define SHIFT_P2_FREQ1__P2VCOSEL    0

// P2_FREQ2 fields (0x10)
#define SHIFT_P2_FREQ2__P2NMSB      0

// P2_FREQ3 fields (0x11)
#define SHIFT_P2_FREQ3__P2NLSB      8

// FN_CTRL fields (0x12)
#define SHIFT_FN_CTRL__FNZ          15
#define SHIFT_FN_CTRL__DITHR        14
#define SHIFT_FN_CTRL__SDM          12
#define SHIFT_FN_CTRL__PHALN        11
#define SHIFT_FN_CTRL__PHSALNGAIN   8
#define SHIFT_FN_CTRL__PHSALNDLY    6
#define SHIFT_FN_CTRL__MODE         5
#define SHIFT_FN_CTRL__DITH         4
#define SHIFT_FN_CTRL__FM           3
#define SHIFT_FN_CTRL__DMODE        2
#define SHIFT_FN_CTRL__TZPS         1

// EXT_MOD fields (0x13)
#define SHIFT_EXT_MOD__MODSETUP     14
#define SHIFT_EXT_MOD__MODSTEP      10

// FMOD fields (0x14)
#define SHIFT_FMOD__FMOD            0

// SDI_CTRL fields (0x15)
#define SHIFT_SDI_CTRL__SIPIN       15
#define SHIFT_SDI_CTRL__ENBL        14
#define SHIFT_SDI_CTRL__MODE        13
#define SHIFT_SDI_CTRL__4WIRE       12
#define SHIFT_SDI_CTRL__ADDR        11
#define SHIFT_SDI_CTRL__RESET       1

// GPO fields (0x16)
#define SHIFT_GPO__P2GPO            9
#define SHIFT_GPO__P1GPO            2
#define SHIFT_GPO__GATE             1
#define SHIFT_GPO__LOCK             0

// T_VCO fields (0x17)
#define SHIFT_T_VCO__CURVE_DEF_VCO1 13
#define SHIFT_T_VCO__CURVE_DEF_VCO2 10
#define SHIFT_T_VCO__CURVE_DEF_VCO3 7

// IQMOD1 fields (0x18)
#define SHIFT_IQMOD1__CTRL          15
#define SHIFT_IQMOD1__BBGM          14
#define SHIFT_IQMOD1__TXLO          13
#define SHIFT_IQMOD1__MOD           12
#define SHIFT_IQMOD1__MODIV         11
#define SHIFT_IQMOD1__MODBIAS       8
#define SHIFT_IQMOD1__LOBIAS        6
#define SHIFT_IQMOD1__CALON         5
#define SHIFT_IQMOD1__CALNUL        4
#define SHIFT_IQMOD1__CALBLK        3
#define SHIFT_IQMOD1__DIVBIAS       2
#define SHIFT_IQMOD1__BUFDC         0

// IQMOD2 fields (0x19)
#define SHIFT_IQMOD2__BBATTEN       12
#define SHIFT_IQMOD2__RCTUNE        6
#define SHIFT_IQMOD2__CALATTEN      4
#define SHIFT_IQMOD2__MOD           2
#define SHIFT_IQMOD2__MODBUF        0

// IQMOD3 fields (0x1A)
#define SHIFT_IQMOD3__BUFDACI       10
#define SHIFT_IQMOD3__BUFDACQ       4
#define SHIFT_IQMOD3__DACEN         3

// IQMOD4 fields (0x1B)
#define SHIFT_IQMOD4__MODDACI       10
#define SHIFT_IQMOD4__MODDACQ       4
#define SHIFT_IQMOD4__BUFBIAS1      2
#define SHIFT_IQMOD4__BUFBIAS2      0

// T_CTRL fields (0x1C)
#define SHIFT_T_CTRL__TC_EN         14
#define SHIFT_T_CTRL__TBL_S         12

// DEV_CTRL fields (0x1D)
#define SHIFT_DEV_CTRL__READSEL     12
#define SHIFT_DEV_CTRL__RSMST       11
#define SHIFT_DEV_CTRL__RSMSTOPST   6
#define SHIFT_DEV_CTRL__CPU         5
#define SHIFT_DEV_CTRL__CPD         4
#define SHIFT_DEV_CTRL__DAC         3
#define SHIFT_DEV_CTRL__CTCLK       2
#define SHIFT_DEV_CTRL__BYPASS      1

// TEST fields (0x1E)
#define SHIFT_TEST__TEN             15
#define SHIFT_TEST__TMUX            12
#define SHIFT_TEST__TSEL            10
#define SHIFT_TEST__LFSR            9
#define SHIFT_TEST__LFSRP           8
#define SHIFT_TEST__LFSRGATETIME    4
#define SHIFT_TEST__LFSRT           3
#define SHIFT_TEST__RGBYP           2
#define SHIFT_TEST__RCBYP           1

class RFFCx07xA_Synth
{
    public:
        static void HWInit(void);
        static void Init(void);
        static void SetEnabled(bool enable);
        static void SetFreq(uint64_t freq_Hz, bool waitForPLLLock=false, bool useModulation=false);
        static bool isPLLLocked(void);
    private:
        static void Write(uint32_t dataBits);
        static void SendAddress(bool write, uint8_t address);
        static void SendData(uint16_t data);
        static uint16_t ReceiveData(void);
        static uint16_t Read(uint8_t address);
        static void SetFreqLO(uint64_t f_lo_Hz, bool waitForPLLLock, uint16_t &nummsb_ref, uint16_t &numlsb_ref);
        static void GetModParams(int32_t freq_delta_Hz, uint8_t &modstep, int16_t &fmod_step);
};

#endif
