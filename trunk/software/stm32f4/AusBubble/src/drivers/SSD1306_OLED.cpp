/************************************************************************/
/* AusBubble:                                                           */
/* An open-source RF jammer designed to work in the 2.4 GHz Wi-Fi       */
/* frequency block.                                                     */
/*                                                                      */
/* SSD1306_OLED.cpp                                                     */
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

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//
//  UG-2864HSWEG01 (2864-46p) Reference Code
//
//    Dot Matrix: 128*64
//    Driver IC : SSD1306 (Solomon Systech)
//    Interface : 8-bit 68XX/80XX Parallel, 3-/4-wire SPI
//    Revision  :
//    Date      : 2009/06/05
//    Author    :
//    Editor    : Humphrey Lin
//
//  Copyright (c) Univision Technology Inc.
//
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#include "SSD1306_OLED.h"

void SSD1306_OLED::HWInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef     SPI_InitStructure;

    // RST
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin     = OLED_RST_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
    GPIO_Init(OLED_RST_PORT, &GPIO_InitStructure);
    // DC
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin     = OLED_DC_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
    GPIO_Init(OLED_DC_PORT, &GPIO_InitStructure);

    // SPI1
    /* Configure SPI1 pins: SCK and MOSI */
    // SCK=PA5, MOSI=PA7
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_5 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    /* Configure CS pin */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin     = OLED_CS_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;
    GPIO_Init(OLED_CS_PORT, &GPIO_InitStructure);

    /* SPI1 Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    /* SPI1 Config */
    // Note: OLED has maximum allowable clock speed = 10 MHz (i.e. minimum Tcycle=100ns)
    SPI_StructInit(&SPI_InitStructure);
    SPI_InitStructure.SPI_Direction         = SPI_Direction_1Line_Tx;
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL              = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;               // Read data on RISING edge of clock
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;     // SCK Freq = SystemFrequency/APB2 Prescaler/SPI_BaudRatePrescaler -> 168/2/16 = 5.25 MHz
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    /* SPI1 enable */
    SPI_Cmd(SPI1, ENABLE);
}

/* Initialization routine (call once before using OLED) */
void SSD1306_OLED::Init(void)
{
    // Toggle RST line (delay between pin SET/RESET)
    // Note: Logic low must have duration >3us
    GPIO_WriteBit(OLED_RST_PORT, OLED_RST_PIN, Bit_RESET);
    GPIO_WriteBit(OLED_RST_PORT, OLED_RST_PIN, Bit_SET);

    setDisplayOnOff(0x00);              // Display Off (0x00/0x01)
    setDisplayClock(0xF0);              // Max display clock frequency, divide ratio=1
    setMultiplexRatio(0x3F);            // 1/64 Duty (0x0F~0x3F)
    setDisplayOffset(0x00);             // Shift Mapping RAM Counter (0x00~0x3F)
    setStartLine(0x00);                 // Set Mapping RAM Display Start Line (0x00~0x3F)
    setChargePump(0x04);                // Enable Embedded DC/DC Converter (0x00/0x04)
    setAddressingMode(0x02);            // Set Page Addressing Mode (0x00/0x01/0x02)
    setSegmentRemap(0x01);              // Set SEG/Column Mapping (0x00/0x01)
    setCommonRemap(0x08);               // Set COM/Row Scan Direction (0x00/0x08)
    setCommonConfig(0x10);              // Set Sequential Configuration (0x00/0x10)
    setContrastControl(oledBrightness); // Set SEG Output Current
    setPrechargePeriod(0xF1);           // Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    setVCOMH(0x40);                     // Set VCOM Deselect Level
    setEntireDisplay(0x00);             // Disable Entire Display On (0x00/0x01)
    setInverseDisplay(0x00);            // Disable Inverse Display On (0x00/0x01)

    fillScreen(0x00);                   // Clear Screen

    setDisplayOnOff(0x01);              // Display On (0x00/0x01)
}

/* Write a command to the OLED */
void SSD1306_OLED::writeCommand(uint8_t data)
{
    // CS=0, DC=0
    GPIO_WriteBit(OLED_CS_PORT, OLED_CS_PIN, Bit_RESET);
    GPIO_WriteBit(OLED_DC_PORT, OLED_DC_PIN, Bit_RESET);

    // Write data to SPI
    SPI_I2S_SendData(SPI1, data);
    // Wait until SPI is ready
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == RESET);
    // CS=1
    GPIO_WriteBit(OLED_CS_PORT, OLED_CS_PIN, Bit_SET);
}

/* Write data to the OLED */
void SSD1306_OLED::writeData(uint8_t data)
{
    // CS=0, DC=1
    GPIO_WriteBit(OLED_CS_PORT, OLED_CS_PIN, Bit_RESET);
    GPIO_WriteBit(OLED_DC_PORT, OLED_DC_PIN, Bit_SET);

    // Write data to SPI
    SPI_I2S_SendData(SPI1, data);
    // Wait until SPI is ready
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == RESET);
    // CS=1
    GPIO_WriteBit(OLED_CS_PORT, OLED_CS_PIN, Bit_SET);
}

/* Below are functions used to configure the OLED */
void SSD1306_OLED::setStartColumn(unsigned char d)
{
    writeCommand(0x00+d%16);    // Set Lower Column Start Address for Page Addressing Mode
                                // Default => 0x00
    writeCommand(0x10+d/16);    // Set Higher Column Start Address for Page Addressing Mode
                                // Default => 0x10
}

void SSD1306_OLED::setAddressingMode(unsigned char d)
{
    writeCommand(0x20);         // Set Memory Addressing Mode
    writeCommand(d);            // Default => 0x02
                                // 0x00 => Horizontal Addressing Mode
                                // 0x01 => Vertical Addressing Mode
                                // 0x02 => Page Addressing Mode
}

void SSD1306_OLED::setColumnAddress(unsigned char a, unsigned char b)
{
    writeCommand(0x21);         // Set Column Address
    writeCommand(a);            // Default => 0x00 (Column Start Address)
    writeCommand(b);            // Default => 0x7F (Column End Address)
}

void SSD1306_OLED::setPageAddress(unsigned char a, unsigned char b)
{
    writeCommand(0x22);         // Set Page Address
    writeCommand(a);            // Default => 0x00 (Page Start Address)
    writeCommand(b);            // Default => 0x07 (Page End Address)
}

void SSD1306_OLED::setStartLine(unsigned char d)
{
    writeCommand(0x40|d);       // Set Display Start Line
                                // Default => 0x40 (0x00)
}

void SSD1306_OLED::setContrastControl(unsigned char d)
{
    writeCommand(0x81);         // Set Contrast Control
    writeCommand(d);            // Default => 0x7F
}

void SSD1306_OLED::setChargePump(unsigned char d)
{
    writeCommand(0x8D);         // Set Charge Pump
    writeCommand(0x10|d);       // Default => 0x10
                                // 0x10 (0x00) => Disable Charge Pump
                                // 0x14 (0x04) => Enable Charge Pump
}

void SSD1306_OLED::setSegmentRemap(unsigned char d)
{
    writeCommand(0xA0|d);       // Set Segment Re-Map
                                // Default => 0xA0
                                // 0xA0 (0x00) => Column Address 0 Mapped to SEG0
                                // 0xA1 (0x01) => Column Address 0 Mapped to SEG127
}

void SSD1306_OLED::setEntireDisplay(unsigned char d)
{
    writeCommand(0xA4|d);       // Set Entire Display On / Off
                                // Default => 0xA4
                                // 0xA4 (0x00) => Normal Display
                                // 0xA5 (0x01) => Entire Display On
}

void SSD1306_OLED::setInverseDisplay(unsigned char d)
{
    writeCommand(0xA6|d);       // Set Inverse Display On/Off
                                // Default => 0xA6
                                // 0xA6 (0x00) => Normal Display
                                // 0xA7 (0x01) => Inverse Display On
}

void SSD1306_OLED::setMultiplexRatio(unsigned char d)
{
    writeCommand(0xA8);         // Set Multiplex Ratio
    writeCommand(d);            // Default => 0x3F (1/64 Duty)
}

void SSD1306_OLED::setDisplayOnOff(unsigned char d)
{
    writeCommand(0xAE|d);       // Set Display On/Off
                                // Default => 0xAE
                                // 0xAE (0x00) => Display Off
                                // 0xAF (0x01) => Display On
}

void SSD1306_OLED::setStartPage(unsigned char d)
{
    writeCommand(0xB0|d);       // Set Page Start Address for Page Addressing Mode
                                // Default => 0xB0 (0x00)
}

void SSD1306_OLED::setCommonRemap(unsigned char d)
{
    writeCommand(0xC0|d);       // Set COM Output Scan Direction
                                // Default => 0xC0
                                // 0xC0 (0x00) => Scan from COM0 to 63
                                // 0xC8 (0x08) => Scan from COM63 to 0
}

void SSD1306_OLED::setDisplayOffset(unsigned char d)
{
    writeCommand(0xD3);         // Set Display Offset
    writeCommand(d);            // Default => 0x00
}

void SSD1306_OLED::setDisplayClock(unsigned char d)
{
    writeCommand(0xD5);         // Set Display Clock Divide Ratio / Oscillator Frequency
    writeCommand(d);            // Default => 0x80
                                // D[3:0] => Display Clock Divider
                                // D[7:4] => Oscillator Frequency
}

void SSD1306_OLED::setPrechargePeriod(unsigned char d)
{
    writeCommand(0xD9);         // Set Pre-Charge Period
    writeCommand(d);            // Default => 0x22 (2 Display Clocks [Phase 2] / 2 Display Clocks [Phase 1])
                                // D[3:0] => Phase 1 Period in 1~15 Display Clocks
                                // D[7:4] => Phase 2 Period in 1~15 Display Clocks
}

void SSD1306_OLED::setCommonConfig(unsigned char d)
{
    writeCommand(0xDA);         // Set COM Pins Hardware Configuration
    writeCommand(0x02|d);       // Default => 0x12 (0x10)
                                // Alternative COM Pin Configuration
                                // Disable COM Left/Right Re-Map
}

void SSD1306_OLED::setVCOMH(unsigned char d)
{
    writeCommand(0xDB);         // Set VCOMH Deselect Level
    writeCommand(d);            // Default => 0x20 (0.77*VCC)
}

void SSD1306_OLED::setNOP()
{
    writeCommand(0xE3);         // Command for No Operation
}

/* Fill the screen with a particular data pattern */
void SSD1306_OLED::fillScreen(unsigned char data)
{
    unsigned char i,j;

    for(i=0;i<8;i++)
    {
        setStartPage(i);
        setStartColumn(0x00);

        for(j=0;j<128;j++)
            writeData(data);
    }
}

/* Fill a specified block with a particular data pattern */
void SSD1306_OLED::fillBlock(unsigned char data, unsigned char startRow, unsigned char endRow, unsigned char startCol, unsigned char endCol)
{
    unsigned char i,j;

    for(i=startRow;i<(endRow+1);i++)
    {
        setStartPage(i);
        setStartColumn(startCol);

        for(j=0;j<endCol;j++)
            writeData(data);
    }
}

/* Draw a frame (border) around the OLED */
void SSD1306_OLED::drawFrame()
{
    unsigned char i,j;

    setStartPage(0x00);
    setStartColumn(xLevel);

    for(i=0;i<maxColumn;i++)
        writeData(0x01);

    setStartPage(0x07);
    setStartColumn(xLevel);

    for(i=0;i<maxColumn;i++)
        writeData(0x80);

    for(i=0;i<8;i++)
    {
        setStartPage(i);

        for(j=0;j<maxColumn;j+=(maxColumn-1))
        {
            setStartColumn(xLevel+j);
            writeData(0xFF);
        }
    }
}

/* Draw a 30x8 battery symbol with specified level */
void SSD1306_OLED::drawBatt(int level, int x, int row)
{
    if(level>100)
        level=100;
    else if(level<0)
        level=0;

    setStartPage(row);
    setStartColumn(x+1);

    for(int i=0;i<27;i++)
        writeData(0x41);

    setStartColumn(x);
    writeData(0x7F);

    setStartColumn(x+28);
    writeData(0x7F);

    setStartColumn(x+29);
    writeData(0x1C);

    for(int i=0;i<level/4;i++)
    {
        setStartColumn(i+x+2);
        writeData(0x5D);
    }
}

/* Print a single character from font.cpp */
void SSD1306_OLED::showFont57(char ascii, unsigned char row, unsigned char xPos)
{
    char *srcPointer = 0;
    unsigned char i;

    srcPointer = &fontData[(ascii-32)][0];

    setStartPage(row);
    setStartColumn(xPos);

    for(i=0;i<5;i++)
    {
        writeData(*srcPointer);
        srcPointer++;
    }
    writeData(0x00);
}

/* Display a null-terminated string on the OLED */
void SSD1306_OLED::showString(const char *dataPtr, unsigned char row, unsigned char xPos)
{
    char *srcPointer;

    srcPointer = (char*)dataPtr;
    showFont57(' ',row,xPos); // NBSP must be written first before the string start

    while(1)
    {
        showFont57(*srcPointer,row,xPos);
        srcPointer++;
        xPos+=6;
        if(*srcPointer == 0) break;
    }
}

/* Put the OLED to sleep to save power */
void SSD1306_OLED::sleep(unsigned char doSleep)
{
    switch(doSleep)
    {
        case 0:
            setDisplayOnOff(0x00);
            setEntireDisplay(0x01);
            break;
        case 1:
            setEntireDisplay(0x00);
            setDisplayOnOff(0x01);
            break;
    }
}
