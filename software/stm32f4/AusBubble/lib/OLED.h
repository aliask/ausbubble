/************************************************************************/
/* AusBubble:                                                           */
/* An open-source RF jammer designed to work in the 2.4 GHz Wi-Fi       */
/* frequency block.                                                     */
/*                                                                      */
/* OLED.h                                                               */
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

#ifndef _OLED_H
#define _OLED_H

#include "Includes.h"

/* Global Variables */
#define xLevelL			0x00
#define xLevelH			0x10
#define xLevel			((xLevelH&0x0F)*16+xLevelL)
#define maxColumn		128
#define maxRow			64
#define	oledBrightness	0xCF

/* Font Data */
extern char fontData[240][5];

/* Function Prototypes */
void oledInit(void);
void drawFrame(void);
void fillScreen(unsigned char Data);
void fillBlock(unsigned char Data, unsigned char a, unsigned char b, unsigned char c, unsigned char d);
void showString(const char *Data_Pointer, unsigned char base, unsigned char column);
void showFont57(char b, unsigned char c, unsigned char d);
void oledSleep(unsigned char sleep);
void setNOP(void);

#endif
