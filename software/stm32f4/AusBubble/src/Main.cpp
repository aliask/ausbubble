/************************************************************************/
/* AusBubble:                                                           */
/* An open-source RF jammer designed to operate in the 2.4 GHz Wi-Fi    */
/* frequency block.                                                     */
/*                                                                      */
/* Main.cpp                                                             */
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

#include "Includes.h"
#include "UI.h"
#include "Jammer.h"
/* Peripheral libraries */
#include "RDA1005L_VarGainAmp.h"
#include "SSD1306_OLED.h"

/* USB */
extern "C" {
    #include "usbd_cdc_core.h"
    #include "usbd_usr.h"
    #include "usbd_desc.h"
    #include "usbd_cdc_vcp.h"
}
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
    #if defined ( __ICCARM__ ) /*!< IAR Compiler */
        #pragma data_alignment = 4
    #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;

/* Volatiles */
__IO uint16_t gADC1ConvertedValue[ADC1_BUFFER_LENGTH];
__IO uint16_t gADC3ConvertedValue = 0;

/* Global Variables */
Stats stats;

/* Function Prototypes */
void prvSetupHardware(void);
void SetupJoystick(void);
void NVIC_Config(void);
void ADC1_DMA_Config(void);
void ADC3_DMA_Config(void);
void I2C2_Config(void);

/* FreeRTOS millisecond delay function */
void DelayMS(uint32_t milliseconds)
{
    vTaskDelay(milliseconds / portTICK_RATE_MS);
}

/* RTOS "heartbeat" LED task */
void vHeartbeatTask(void *pvParameters)
{
    /* Local variables */
    bool toggle = false;

    while(1)
    {
        /* Toggle LED */
        if(toggle ^= true)
            GPIO_WriteBit(RTOS_LED_PORT, RTOS_LED_PIN, Bit_RESET);
        else
            GPIO_WriteBit(RTOS_LED_PORT, RTOS_LED_PIN, Bit_SET);
        /* Update statistics structure */
        stats.heartbeat = toggle;
        UI::updateStatsData(stats);
        /* Sleep */
        DelayMS(500);
    }
}

/* User interface task */
void vUITask(void *pvParameters)
{
    /* Local variables */
    uint8_t buttonState = ButtonNone;
    uint8_t buttonState_prev = ButtonNone;
    uint32_t debounceCount = 0;
    uint32_t drawCount = 0;
    int tickRate = TICK_INITIALRATE;
    uint32_t holdCount = 0;
    bool buttonReleased = false;

    while(1)
    {
        /* Read button state (5-bits) */
        /* Note: [4:0] = {Select,Right,Left,Down,Up} */
        buttonState = ButtonNone;
        buttonState |= ((JOYSTICK_SELECT_PORT->IDR & JOYSTICK_SELECT_PIN) == 0) << 4;
        buttonState |= ((JOYSTICK_RIGHT_PORT->IDR & JOYSTICK_RIGHT_PIN) == 0) << 3;
        buttonState |= ((JOYSTICK_LEFT_PORT->IDR & JOYSTICK_LEFT_PIN) == 0) << 2;
        buttonState |= ((JOYSTICK_DOWN_PORT->IDR & JOYSTICK_DOWN_PIN) == 0) << 1;
        buttonState |= ((JOYSTICK_UP_PORT->IDR & JOYSTICK_UP_PIN) == 0) << 0;

        /* Has button state changed since last check? */
        if(buttonState_prev == buttonState)
            debounceCount++;
        else
            debounceCount=0;

        if(drawCount < UI_DRAW_COUNT_MAX)
            drawCount++;
        else
        {
            /* Re-draw everything if we're on the home screen */
            if(UI::currentState == HomeScreen)
                UI::draw(HomeScreen);
            /* Just draw the header otherwise */
            else
                UI::drawHeader();
            /* Reset count */
            drawCount = 0;
        }

        /* Has button state been stable long enough? (de-bouncing) */
        if(debounceCount >= DEBOUNCE_COUNT)
        {
            /* Button press(es) detected */
            if(buttonState)
            {
                /* Button is being held down */
                if(((holdCount % TICK_HOLDCOUNT) == 0) && (holdCount > 0))
                {
                    /* UP/DOWN: Increase tick rate */
                    if((buttonState == ButtonUp)||(buttonState == ButtonDown))
                    {
                        switch(tickRate)
                        {
                            case TICK_RATE_1:
                                tickRate = TICK_RATE_2;
                                break;
                            case TICK_RATE_2:
                                tickRate = TICK_RATE_3;
                                break;
                            case TICK_RATE_3:
                                tickRate = TICK_RATE_4;
                                break;
                            case TICK_RATE_4:
                                tickRate = TICK_RATE_5;
                                break;
                            case TICK_RATE_5:
                                break;
                        }
                    }
                    /* SELECT: Enable RF output */
                    else if(buttonState == ButtonSelect)
                    {
                        /* Enable jamming if disabled (and if not at Disclaimer screen) */
                        if(!Jammer::isEnabled() && (UI::currentState != DisclaimerScreen))
                        {
                            /* Set toggle off (de-selects setting if selected) */
                            UI::setToggle(false);

                            /* Enable hardware */
                            Jammer::SetEnabled(true);
                            /* Show splash text */
                            UI::splash("RF Output ENABLED", 1000);
                            /* Draw Home screen */
                            UI::draw(HomeScreen);
                            /* Set flag */
                            buttonReleased = false;
                        }
                    }
                }
                else
                {
                    /* Disable RF output if enabled */
                    if(Jammer::isEnabled() && buttonReleased)
                    {
                        /* Set hardware */
                        Jammer::SetEnabled(false);
                        /* Show splash text */
                        UI::splash("RF Output DISABLED", 1000);
                    }
                }

                /* Update UI */
                if(holdCount % tickRate == DO_MENU_HOLD_COUNT)
                    UI::doMenu(buttonState);

                holdCount++;
            }
            /* No button(s) held down */
            else
            {
                buttonReleased = true;
                holdCount = 0;
                tickRate = TICK_RATE_1;
            }
        }
        /* Save button state */
        buttonState_prev = buttonState;
        /* Sleep for 1 ms */
        DelayMS(1);
    }
}

/* Read/save various run-time statistics */
void vStatsTask(void *pvParameters)
{
    /* Local variables */
    uint32_t ADC1ConvertedVoltage0 = 0;
    uint32_t ADC1ConvertedVoltage1 = 0;
    float Vsense = 0.0;
    float Pout_dBm;
    float gPDETVoltage;
    float TCelsius;
    float VBATVoltage;

    while(1)
    {
        /* 1. RF Amplifier Power Detection (PDET) */
        gPDETVoltage = 3.0*((float)gADC3ConvertedValue/(float)0xFFF);
        // Get output power
        Pout_dBm = RFPA5201_Amp::GetOutputPower_dBm(gPDETVoltage);
        /* 2. VBAT */
        ADC1ConvertedVoltage0 = (uint32_t)(gADC1ConvertedValue[0] * 2) * 3000 / 0xFFF;
        VBATVoltage = (float)(ADC1ConvertedVoltage0 / 1000.0);
        /* 3. On-chip Temperature Sensor */
        ADC1ConvertedVoltage1 = (uint32_t)(gADC1ConvertedValue[1] * 3000 / 0xFFF);
        Vsense = (float)(ADC1ConvertedVoltage1 / 1000.0);
        TCelsius = ((V25-Vsense) / AVG_SLOPE) + 25.0 ;
        /* 4. TODO: RF Amplifier Thermistor */
        /* 5. TODO: Battery parameters (via I2C) */

        /* Update statistics structure */
        // System
        stats.Vbat = VBATVoltage;
        stats.onChipTemp_degC = TCelsius;
        // Battery
        stats.batteryLevel = 75;
        stats.isCharging = false;
        // Jammer
        stats.isJamming = Jammer::isEnabled();
        stats.isPLLLocked = RFFCx07x_Synth::isPLLLocked();
        stats.Pout_dBm = Pout_dBm;
        // Do update
        UI::updateStatsData(stats);

        /* Sleep */
        DelayMS(250);
    }
}

/* Jamming task */
void vJammingTask(void *pvParameters)
{
    while(1)
    {
        if(TIM_GetFlagStatus(TIM2, TIM_FLAG_Update) != RESET)
        {
            /* Update synthesizer frequency */
            Jammer::Advance();

            /* Clear timer flag */
            TIM_ClearFlag(TIM2, TIM_IT_Update);
        }
        taskYIELD();
    }
}

/* Main */
int main(void)
{
    /* At this stage the microcontroller clock setting is already configured,
    this is done through SystemInit() function which is called from startup
    file (startup_stm32f4xx.S) before to branch to application main.
    To reconfigure the default setting of SystemInit() function, refer to
    system_stm32f4xx.c file */

    /* Setup STM32 hardware */
    prvSetupHardware();

    /* Initialize jammer */
    Jammer::Init();

    /* Create FreeRTOS tasks */
    xTaskCreate(vHeartbeatTask,
                (signed portCHAR *)"vHeartbeatTask",
                512,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
    xTaskCreate(vUITask,
                (signed portCHAR *)"vUITask",
                512,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
    xTaskCreate(vStatsTask,
                (signed portCHAR *)"vADCTask",
                512,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
    xTaskCreate(vJammingTask,
                (signed portCHAR *)"vJammingTask",
                512,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
    /* Start FreeRTOS */
    vTaskStartScheduler();

    /* Main never returns */
    while(1);
    return 0;
}

/* Initialize necessary hardware */
void prvSetupHardware(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure HCLK clock as SysTick clock source */
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    /* GPIO Periph clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);
    /* SYSCFG APB clock must be enabled to get write access to SYSCFG_EXTICRx registers */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Enable FPU */
    #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
        SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));    // Set CP10 and CP11 Full Access
    #endif

    /* Random Number Generator */
    // Enable RNG clock source
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    // RNG Peripheral enable
    RNG_Cmd(ENABLE);

    /* RTOS "heartbeat" LED */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin     = RTOS_LED_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
    GPIO_Init(RTOS_LED_PORT, &GPIO_InitStructure);
    // Initially switch off LED
    GPIO_ResetBits(RTOS_LED_PORT, RTOS_LED_PIN);

    /* OLED */
    SSD1306_OLED::HWInit();
    SSD1306_OLED::Init();
    UI::draw(DisclaimerScreen);

    /* SYNTH */
    RFFCx07x_Synth::HWInit();
    RFFCx07x_Synth::Init();

    /* RF Amplifier */
    RFPA5201_Amp::HWInit();
    RFPA5201_Amp::SetEnabled(false);

    /* Variable Gain Amplifier */
    RDA1005L_VarGainAmp::HWInit();
    // Set gain to minimum
    RDA1005L_VarGainAmp::SetGain(VARGAINAMP_MIN_GAIN_LIMIT_DB);

    /* Joystick */
    SetupJoystick();

    /* ADC1 (VBAT and Temperature Sensor) */
    ADC1_DMA_Config();
    // Start ADC1 Software Conversion
    ADC_SoftwareStartConv(ADC1);

    /* ADC3 (PDET) */
    ADC3_DMA_Config();
    // Start ADC3 Software Conversion
    ADC_SoftwareStartConv(ADC3);

    /* I2C */
    I2C2_Config();

    /* Enable clock for timer used in vJammingTask() */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* Enable USB */
    USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
    /* Disable STDOUT buffering. Otherwise nothing will be printed
    before a newline character or when the buffer is flushed */
    setbuf(stdout, NULL);

    /* Nested Vectored Interrupt Controller */
    NVIC_Config();
}

void SetupJoystick(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Note: Internal pull-ups are used on each button input (i.e. external pull-up resistors not required) */

    /* Configure LEFT as input floating */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin     = JOYSTICK_LEFT_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;
    GPIO_Init(JOYSTICK_LEFT_PORT, &GPIO_InitStructure);
    /* Configure RIGHT as input floating */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin     = JOYSTICK_RIGHT_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;
    GPIO_Init(JOYSTICK_RIGHT_PORT, &GPIO_InitStructure);
    /* Configure UP as input floating */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin     = JOYSTICK_UP_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;
    GPIO_Init(JOYSTICK_UP_PORT, &GPIO_InitStructure);
    /* Configure DOWN as input floating */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin     = JOYSTICK_DOWN_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;
    GPIO_Init(JOYSTICK_DOWN_PORT, &GPIO_InitStructure);
    /* Configure SEL as input floating */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin     = JOYSTICK_SELECT_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;
    GPIO_Init(JOYSTICK_SELECT_PORT, &GPIO_InitStructure);
}

void NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Set the Vector Table base address at 0x08000000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00);

    /* Ensure all priority bits are assigned as preemption priority bits. */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* Enable the EXTI15_10 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                      = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                   = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void ADC1_DMA_Config(void)
{
    ADC_InitTypeDef       ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    DMA_InitTypeDef       DMA_InitStructure;

    /* Enable ADC1, DMA2 clocks */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    /* DMA2 Stream4 Channel0 configuration */
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_Channel               = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)ADC1_DR_ADDRESS;
    DMA_InitStructure.DMA_Memory0BaseAddr       = (uint32_t)&gADC1ConvertedValue;
    DMA_InitStructure.DMA_DIR                   = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize            = 2;
    DMA_InitStructure.DMA_PeripheralInc         = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc             = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize    = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize        = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode                  = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority              = DMA_Priority_Low;
    DMA_InitStructure.DMA_FIFOMode              = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold         = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst           = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst       = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream4, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream4, ENABLE);

    /* ADC Common Init */
    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode                = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler           = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode       = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay    = ADC_TwoSamplingDelay_20Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    /* ADC1 Init */
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode          = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode    = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge  = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv      = 0;
    ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion       = ADC1_BUFFER_LENGTH;
    ADC_Init(ADC1, &ADC_InitStructure);

    /* ADC1 regular channel18 (VBAT) & channel16 (TempSensor) configuration */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_Vbat, 1, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 2, ADC_SampleTime_480Cycles);

    /* Enable VBAT channel: channel18 */
    ADC_VBATCmd(ENABLE);

    /* Enable TempSensor and Vrefint channels: channel16 and channel17 */
    ADC_TempSensorVrefintCmd(ENABLE);

    /* Enable DMA request after last transfer (Single-ADC mode) */
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);

    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
}

void ADC3_DMA_Config(void)
{
    ADC_InitTypeDef       ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    DMA_InitTypeDef       DMA_InitStructure;
    GPIO_InitTypeDef      GPIO_InitStructure;

    /* Enable ADC3, DMA2 and GPIO clocks */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

    /* DMA2 Stream0 Channel2 configuration */
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_Channel               = DMA_Channel_2;
    DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)ADC3_DR_ADDRESS;
    DMA_InitStructure.DMA_Memory0BaseAddr       = (uint32_t)&gADC3ConvertedValue;
    DMA_InitStructure.DMA_DIR                   = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize            = 1;
    DMA_InitStructure.DMA_PeripheralInc         = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc             = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize    = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize        = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode                  = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority              = DMA_Priority_Low;
    DMA_InitStructure.DMA_FIFOMode              = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold         = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst           = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst       = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream0, ENABLE);

    /* Configure ADC3 Channel12 pin as analog input */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* ADC Common Init */
    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode                = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler           = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode       = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay    = ADC_TwoSamplingDelay_20Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    /* ADC3 Init */
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode          = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode    = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge  = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv      = 0;
    ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion       = 1;
    ADC_Init(ADC3, &ADC_InitStructure);

    /* ADC3 regular channel12 configuration */
    ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);

    /* Enable DMA request after last transfer (Single-ADC mode) */
    ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

    /* Enable ADC3 DMA */
    ADC_DMACmd(ADC3, ENABLE);

    /* Enable ADC3 */
    ADC_Cmd(ADC3, ENABLE);
}

void I2C2_Config(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    I2C_InitTypeDef     I2C_InitStructure;

    /* Enable APB1 peripheral clock for I2C2 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    /* Enable clock for SCL and SDA pins */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* SCL=PB10, SDA=PB11 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;      // Set output to open-drain (line has to only be pulled low, not driven high)
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        // Enable pull up resistors
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Connect I2C2 pins to AF */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

    /* Configure I2C2 */
    I2C_InitStructure.I2C_ClockSpeed = 400000;                                 // 400kHz
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;                                 // I2C mode
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;                         // 50% duty cycle (standard)
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;                                  // Own address (not relevant in master mode)
    I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;                               // Disable acknowledge when reading
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  // Set address length to 7 bit addresses
    I2C_Init(I2C2, &I2C_InitStructure);

    /* Enable I2C2 */
    I2C_Cmd(I2C2, ENABLE);
}

extern "C"
void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    taskDISABLE_INTERRUPTS();
    while(1);
}

extern "C"
void vApplicationIdleHook( void )
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}

extern "C"
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();
    while(1);
}
