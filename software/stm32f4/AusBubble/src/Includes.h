/************************************************************************/
/* AusBubble:                                                           */
/* An open-source RF jammer designed to operate in the 2.4 GHz Wi-Fi    */
/* frequency block.                                                     */
/*                                                                      */
/* Includes.h                                                           */
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

#ifndef INCLUDES_H
#define INCLUDES_H

/* Standard libraries */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
/* Specifies the MCU peripherals to use */
#include "stm32f4xx_conf.h"
/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Macro for crude FreeRTOS-safe printf() */
// Note: Using the %llu print-specifier (e.g. for uint64_t) gives unpredictable behaviour
#define vDebugPrintf(A,...)     taskENTER_CRITICAL(); printf(A,##__VA_ARGS__); taskEXIT_CRITICAL();

/* ADC Data Register Addresses */
#define ADC1_DR_ADDRESS     ((uint32_t)0x4001204C)
#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)
/* On-chip temperature sensor properties */
#define V25                 1.41
#define AVG_SLOPE           0.0043
/* ADC1 Buffer Length */
#define ADC1_BUFFER_LENGTH  2

/* UI settings */
#define FREQ_STEP_HZ        500000
#define RATE_STEP_HZ        1
#define DISP_MAX            2
#define UI_DRAW_COUNT_MAX   500
/* Button behaviour */
#define TICK_RATE_1         200     // Slowest
#define TICK_RATE_2         100
#define TICK_RATE_3         50
#define TICK_RATE_4         25
#define TICK_RATE_5         10      // Fastest
#define TICK_INITIALRATE    TICK_RATE_1
#define TICK_HOLDCOUNT      1000
#define DO_MENU_HOLD_COUNT  5
#define DEBOUNCE_COUNT      10
/* Valid step sizes */
#define STEP_1K_HZ          1000
#define STEP_10K_HZ         10000
#define STEP_25K_HZ         25000
#define STEP_50K_HZ         50000
#define STEP_100K_HZ        100000
#define STEP_250K_HZ        250000
#define STEP_500K_HZ        500000
#define STEP_1M_HZ          1000000
#define STEP_NONE           0

/* Scan settings defaults */
#define SCAN_SETTINGS_DEFAULT_START_FREQ_HZ     2400000000      /* DO NOT MODIFY */
#define SCAN_SETTINGS_DEFAULT_STOP_FREQ_HZ      2500000000      /* DO NOT MODIFY */
#define SCAN_SETTINGS_DEFAULT_ALGO              ScanTriangle
#define SCAN_SETTINGS_DEFAULT_STEPSIZE          STEP_1M_HZ
#define SCAN_SETTINGS_DEFAULT_RATE_HZ           500

/* Synth Frequency */
// Allowable range
#define MIN_FREQ_HZ                     2400000000  /* DO NOT MODIFY */
#define MAX_FREQ_HZ                     2500000000  /* DO NOT MODIFY */
/* Jamming Update Rate */
// Allowable range
#define MIN_RATE_HZ                     1           /* DO NOT MODIFY */
#define MAX_RATE_HZ                     1000        /* DO NOT MODIFY */

/* OLED */
#define OLED_CS_PORT                    GPIOA
#define OLED_CS_PIN                     GPIO_Pin_4
#define OLED_RST_PORT                   GPIOA
#define OLED_RST_PIN                    GPIO_Pin_3
#define OLED_DC_PORT                    GPIOA
#define OLED_DC_PIN                     GPIO_Pin_2
/* RTOS Heartbeat LED */
#define RTOS_LED_PORT                   GPIOD
#define RTOS_LED_PIN                    GPIO_Pin_12
/* Synth */
#define SYNTH_SCLK_PORT                 GPIOB
#define SYNTH_SCLK_PIN                  GPIO_Pin_13
#define SYNTH_SDATA_PORT                GPIOB
#define SYNTH_SDATA_PIN                 GPIO_Pin_15
#define SYNTH_SDATA_PIN_N               15
#define SYNTH_ENX_PORT                  GPIOB
#define SYNTH_ENX_PIN                   GPIO_Pin_12
#define SYNTH_RESETX_PORT               GPIOB
#define SYNTH_RESETX_PIN                GPIO_Pin_0
#define SYNTH_GPO1ADD1_PORT             GPIOB
#define SYNTH_GPO1ADD1_PIN              GPIO_Pin_1
#define SYNTH_GPO2ADD2_PORT             GPIOB
#define SYNTH_GPO2ADD2_PIN              GPIO_Pin_2
#define SYNTH_GPO3FM_PORT               GPIOB
#define SYNTH_GPO3FM_PIN                GPIO_Pin_5
#define SYNTH_GPO4LDDO_PORT             GPIOE
#define SYNTH_GPO4LDDO_PIN              GPIO_Pin_9
#define SYNTH_ENBLGPO5_PORT             GPIOB
#define SYNTH_ENBLGPO5_PIN              GPIO_Pin_7
#define SYNTH_MODEGPO6_PORT             GPIOB
#define SYNTH_MODEGPO6_PIN              GPIO_Pin_8
/* Joystick */
// Left
#define JOYSTICK_LEFT_PORT              GPIOE
#define JOYSTICK_LEFT_PIN               GPIO_Pin_10
#define JOYSTICK_LEFT_PORT_SOURCE       EXTI_PortSourceGPIOE
#define JOYSTICK_LEFT_PIN_SOURCE        EXTI_PinSource10
#define JOYSTICK_LEFT_EXTI_LINE         EXTI_Line10
// Right
#define JOYSTICK_RIGHT_PORT             GPIOE
#define JOYSTICK_RIGHT_PIN              GPIO_Pin_11
#define JOYSTICK_RIGHT_PORT_SOURCE      EXTI_PortSourceGPIOE
#define JOYSTICK_RIGHT_PIN_SOURCE       EXTI_PinSource11
#define JOYSTICK_RIGHT_EXTI_LINE        EXTI_Line11
// Up
#define JOYSTICK_UP_PORT                GPIOE
#define JOYSTICK_UP_PIN                 GPIO_Pin_12
#define JOYSTICK_UP_PORT_SOURCE         EXTI_PortSourceGPIOE
#define JOYSTICK_UP_PIN_SOURCE          EXTI_PinSource12
#define JOYSTICK_UP_EXTI_LINE           EXTI_Line12
// Down
#define JOYSTICK_DOWN_PORT              GPIOE
#define JOYSTICK_DOWN_PIN               GPIO_Pin_13
#define JOYSTICK_DOWN_PORT_SOURCE       EXTI_PortSourceGPIOE
#define JOYSTICK_DOWN_PIN_SOURCE        EXTI_PinSource13
#define JOYSTICK_DOWN_EXTI_LINE         EXTI_Line13
// Select
#define JOYSTICK_SELECT_PORT            GPIOE
#define JOYSTICK_SELECT_PIN             GPIO_Pin_14
#define JOYSTICK_SELECT_PORT_SOURCE     EXTI_PortSourceGPIOE
#define JOYSTICK_SELECT_PIN_SOURCE      EXTI_PinSource14
#define JOYSTICK_SELECT_EXTI_LINE       EXTI_Line14
/* RF Amplifier */
#define AMP_PENABLE_PORT                GPIOB
#define AMP_PENABLE_PIN                 GPIO_Pin_9
/* Variable Gain Amplifier */
#define VARGAINAMP_PUP_PORT             GPIOD
#define VARGAINAMP_PUP_PIN              GPIO_Pin_6
#define VARGAINAMP_CLK_PORT             GPIOD
#define VARGAINAMP_CLK_PIN              GPIO_Pin_7
#define VARGAINAMP_DATA_PORT            GPIOD
#define VARGAINAMP_DATA_PIN             GPIO_Pin_8
#define VARGAINAMP_LE_PORT              GPIOD
#define VARGAINAMP_LE_PIN               GPIO_Pin_9

/* RTOS millisecond delay function */
void DelayMS(uint32_t milliseconds);

/* Structure definitions */
struct Stats
{
    /* System */
    bool heartbeat;
    float Vbat;
    float onChipTemp_degC;
    /* Battery */
    int batteryLevel;
    bool isCharging;
    /* Jammer */
    bool isPLLLocked;
    bool isJamming;
    float Pout_dBm;
};

#endif
