/************************************************************************/
/* AusBubble:                                                           */
/* An open-source RF jammer designed to work in the 2.4 GHz Wi-Fi       */
/* frequency block.                                                     */
/*                                                                      */
/* SSD1306_OLED.h                                                       */
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

#ifndef SSD1306_OLED_H
#define SSD1306_OLED_H

#include "Includes.h"

/* Definitions */
#define xLevelL         0x00
#define xLevelH         0x10
#define xLevel          ((xLevelH&0x0F)*16+xLevelL)
#define maxColumn       128
#define maxRow          64
#define oledBrightness  0xCF

/* Font Data */
extern char fontData[240][5];

class SSD1306_OLED
{
    public:
        static void HWInit(void);
        static void Init(void);
        static void drawFrame(void);
        static void fillScreen(unsigned char data);
        static void fillBlock(unsigned char data, unsigned char a, unsigned char b, unsigned char c, unsigned char d);
        static void showString(const char *dataPtr, unsigned char base, unsigned char column);
        static void showFont57(char b, unsigned char c, unsigned char d);
        static void sleep(unsigned char sleep);
        static void setNOP(void);
    private:
        static void writeCommand(uint8_t data);
        static void writeData(uint8_t data);
        static void setStartColumn(unsigned char d);
        static void setAddressingMode(unsigned char d);
        static void setColumnAddress(unsigned char a, unsigned char b);
        static void setPageAddress(unsigned char a, unsigned char b);
        static void setStartLine(unsigned char d);
        static void setContrastControl(unsigned char d);
        static void setChargePump(unsigned char d);
        static void setSegmentRemap(unsigned char d);
        static void setEntireDisplay(unsigned char d);
        static void setInverseDisplay(unsigned char d);
        static void setMultiplexRatio(unsigned char d);
        static void setDisplayOnOff(unsigned char d);
        static void setStartPage(unsigned char d);
        static void setCommonRemap(unsigned char d);
        static void setDisplayOffset(unsigned char d);
        static void setDisplayClock(unsigned char d);
        static void setPrechargePeriod(unsigned char d);
        static void setCommonConfig(unsigned char d);
        static void setVCOMH(unsigned char d);
};

#endif
