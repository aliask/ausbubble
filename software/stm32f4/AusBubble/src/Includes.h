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

// Standard libraries
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
// Specifies the MCU peripherals to use
#include "stm32f4xx_conf.h"
// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

/* Synth Allowable Frequency Range (DO NOT MODIFY) */
#define MIN_FREQ_MHZ                    2400
#define MAX_FREQ_MHZ                    2500

/* OLED */
#define OLED_CS_PORT                    GPIOA
#define OLED_CS_PIN                     GPIO_Pin_4
#define OLED_RST_PORT                   GPIOB
#define OLED_RST_PIN                    GPIO_Pin_10
#define OLED_DC_PORT                    GPIOB
#define OLED_DC_PIN                     GPIO_Pin_11

/* RTOS Heartbeat LED */
#define RTOS_LED_PORT                   GPIOD
#define RTOS_LED_PIN                    GPIO_Pin_12

/* Synth */
#define SYNTH_SCLK_PORT                 GPIOB
#define SYNTH_SCLK_PIN                  GPIO_Pin_13
#define SYNTH_SDATA_PORT                GPIOB
#define SYNTH_SDATA_PIN                 GPIO_Pin_15
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
#define SYNTH_GPO4LDDO_PORT             GPIOB
#define SYNTH_GPO4LDDO_PIN              GPIO_Pin_6
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
