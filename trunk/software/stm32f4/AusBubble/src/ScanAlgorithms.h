/************************************************************************/
/* AusBubble:                                                           */
/* An open-source RF jammer designed to operate in the 2.4 GHz Wi-Fi    */
/* frequency block.                                                     */
/*                                                                      */
/* ScanAlgorithms.h                                                     */
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

#ifndef _SCANALGORITHMS_H
#define _SCANALGORITHMS_H

#include "Includes.h"

// Standard libraries
#include <stdlib.h>
// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
// Peripheral libraries
#include "Synth.h"

#define DIR_UP          true
#define DIR_DOWN        false

/* Structures */
extern struct scanSettings_t {
    uint64_t start;
    uint64_t stop;
    ScanAlgorithms_t algorithm;
    uint32_t stepSize;
    uint16_t rate;
} gScanSettings;

/* Function prototypes */
extern void AdvanceScan(scanSettings_t* settings);
extern void ScanInit(scanSettings_t* settings);

#endif
