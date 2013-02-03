/************************************************************************/
/* AusBubble:                                                           */
/* An open-source RF jammer designed to operate in the 2.4 GHz Wi-Fi    */
/* frequency block.                                                     */
/*                                                                      */
/* Synth.h                                                              */
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

#ifndef SYNTH_H
#define SYNTH_H

#include "Includes.h"
#include "ScanAlgorithms.h"

#define F_VCO_MAX_HZ        5400000000
#define F_REFERENCE_HZ      26000000
#define FBKDIV_2            2       // Prescaler divider (fvco<3.2GHz: 2)
#define FBKDIV_4            4       // Prescaler divider (fvco>3.2GHz: 4)
#define USE_SW_CONTROL      0       // Software Control=1, Hardware Control=0 (via ENBL and MODE pins)

// Synthesizer register defines
#define REG_LF        0x00
#define REG_XO        0x01
#define REG_CAL_TIME  0x02
#define REG_VCO_CTRL  0x03
#define REG_CT_CAL1   0x04
#define REG_CT_CAL2   0x05
#define REG_PLL_CAL1  0x06
#define REG_PLL_CAL2  0x07
#define REG_VCO_AUTO  0x08
#define REG_PLL_CTRL  0x09
#define REG_PLL_BIAS  0x0A
#define REG_MIX_CONT  0x0B
#define REG_P1_FREQ1  0x0C
#define REG_P1_FREQ2  0x0D
#define REG_P1_FREQ3  0x0E
#define REG_P2_FREQ1  0x0F
#define REG_P2_FREQ2  0x10
#define REG_P2_FREQ3  0x11
#define REG_FN_CTRL   0x12
#define REG_EXT_MOD   0x13
#define REG_FMOD      0x14
#define REG_SDI_CTRL  0x15
#define REG_GPO       0x16
#define REG_T_VCO     0x17
#define REG_IQMOD1    0x18
#define REG_IQMOD2    0x19
#define REG_IQMOD3    0x1A
#define REG_IQMOD4    0x1B
#define REG_T_CTRL    0x1C
#define REG_DEV_CTRL  0x1D
#define REG_TEST      0x1E
#define REG_READBACK  0x1F

// LF fields
#define SHIFT_LFACT     15
#define SHIFT_P2CPDEF   9
#define SHIFT_P1CPDEF   3
#define SHIFT_PLLCPL    0

// PLL_CTRL fields
#define SHIFT_DIVBY  15
#define SHIFT_CLKDIV 12
#define SHIFT_PLLST  11
#define SHIFT_TVCO   6
#define SHIFT_LDEN   5
#define SHIFT_LDLEV  4
#define SHIFT_RELOK  3
#define SHIFT_ALOI   2
#define SHIFT_PLLDY  0

// P[12]_FREQ1 fields
#define SHIFT_N      7
#define SHIFT_LODIV  4
#define SHIFT_PRESC  2
#define SHIFT_VCOSEL 0

// EXT_MOD fields
#define SHIFT_MODSETUP 14
#define SHIFT_MODSTEP  10

// SDI_CTRL fields
#define SHIFT_SIPIN 15
#define SHIFT_ENBL  14
#define SHIFT_MODE  13
#define SHIFT_4WIRE 12
#define SHIFT_ADDR  11
#define SHIFT_RESET 1

// GPO fields
#define SHIFT_P2GPO 9
#define SHIFT_P1GPO 2
#define SHIFT_GATE  1
#define SHIFT_LOCK  0

// DEV_CTRL fields
#define SHIFT_READSEL   12
#define SHIFT_RSMST     11
#define SHIFT_RSMSTOPST 6
#define SHIFT_CPU       5
#define SHIFT_CPD       4
#define SHIFT_DAC       3
#define SHIFT_CTCLK     2
#define SHIFT_BYPASS    1

void SynthInit(void);
void SynthEnable(bool enable);
void SynthWrite(uint32_t dataBits);
void SynthSendAddress(bool write, uint8_t address);
void SynthSendData(uint16_t data);
uint16_t SynthReceiveData();
uint16_t SynthRead(uint8_t address);
void SynthSet_Freq(uint64_t freq_Hz);
void SynthSet_FreqLO(uint64_t f_lo_Hz, bool waitForLock, uint16_t &nummsb_ref, uint16_t &numlsb_ref);
void SynthGet_ModParams(int32_t freq_delta_Hz, uint8_t &modstep, int16_t &fmod_step);

#endif
