/*******************************************************************************
*
*    Custom TX for RC vehicles
*
*    Copyright (C) 2015  Damian Thompson
*
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*******************************************************************************/

#include <stm32f10x.h>
#include <stm32f10x_adc.h>
#include <stm32f10x_dma.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>
#include <stm32f10x_tim.h>
#include <misc.h>

#define TIM2_CCR1	0x4000004C	// Not CCR but DMAR pointing to CCR
#define ADC1_DR		0x4001244C

#define PPM_ELEMENTS 18
#define ADC_ELEMENTS 9

#define MAX_TRIM 500
#define TRIM_STEP 50
#define MAX_SUB_TRIM 50

#define DEBOUNCE_REF 100

u16 PPM_Buffer[PPM_ELEMENTS] = {400, 1400, 1800, 3000, 3400, 4700, 5100, 6500, 6900, 8400, 8800, 10800, 11200, 13200, 13600, 16000, 16400, 21999};
u16 ADC_Buffer[ADC_ELEMENTS];
s16 Trim_Vals[8] = {0, 0, 0, 0, 0, 0, 0, 0};
u8 Reverse[8] = {1, 0, 1, 0, 1, 0, 1, 1};

#define CHILD  1        //Values for onEnter
#define HANDLER  2

#define NULL (void*)0

typedef struct menuitem menuitem;

struct menuitem
{
	struct menuitem *prev;
	struct menuitem *next;
	struct menuitem *child;
	struct menuitem *parent;
	void (*handler)(void);
	unsigned int onEnter;
    void (*generator)(void);
};

menuitem m_info_screen, m_monitor, m_reverse, m_timer, m_settings, m_model, m_calibration, m_trim, m_s_batt, m_s_sound, m_s_contrast, m_s_backlight;

menuitem m_info_screen = {&m_info_screen, &m_info_screen, &m_monitor, &m_info_screen, NULL, CHILD, &g_info_screen};
menuitem m_monitor = {&m_settings, &m_model, NULL, &m_info_screen, &h_monitor, HANDLER, &g_monitor};
menuitem m_model = {&m_monitor, &m_timer, NULL, &m_info_screen, &h_model, HANDLER, &g_model};
menuitem m_timer = {&m_model, &m_reverse, NULL, &m_info_screen, &h_timer, HANDLER, &g_timer};
menuitem m_reverse = {&m_timer, &m_trim, NULL, &m_info_screen, &h_reverses, HANDLER, &g_reverse};
menuitem m_trim = {&m_reverse, &m_calibration, NULL, &m_info_screen, &h_trim, HANDLER, &g_trim};
menuitem m_calibration = {&m_trim, &m_settings, NULL, &m_info_screen, &h_calibration, HANDLER, &g_calibration};
menuitem m_settings = {&m_calibration, &m_model, &m_s_batt, &m_info_screen, NULL, CHILD, &g_settings};

menuitem m_s_batt = {&m_s_backlight, &m_s_sound, NULL, &m_settings, &h_s_batt, HANDLER, &g_s_batt};
menuitem m_s_sound =  {&m_s_batt, &m_s_contrast, NULL, &m_settings, &h_s_sound, HANDLER, &g_s_sound};
menuitem m_s_contrast = {&m_s_sound, &m_s_backlight, NULL, &m_settings, &h_s_contrast, HANDLER, &g_s_contrast};
menuitem m_s_backlight = {&m_s_contrast, &m_s_batt, NULL, &m_settings, &h_s_backlight, HANDLER, &g_s_bbacklight};

menuitem current = m_info_screen;

void SysClkInit(void);
void ADCInit(void);
void DMAInit(void);
void TIM2Init(void);
void TrimInit(void);

int main(void)
{
	SysClkInit();
	ADCInit();
	TIM2Init();
	DMAInit();
	TrimInit();

	SysTick_Config(SystemCoreClock / 1000);

	// if (pressed trim at power on) calibrate mode

    while(1)
    {

    }
}

void SysClkInit()
{
	RCC_DeInit();    // Reset RCC to default

	RCC_HSEConfig(RCC_HSE_ON);    // Enable HSE

	ErrorStatus HSEStartUpStatus = RCC_WaitForHSEStartUp();	// Wait for HSE to stabilise

	if (HSEStartUpStatus == SUCCESS)	// If we are stable
	{
		RCC_HCLKConfig(RCC_SYSCLK_Div1);	// HCLK = SYSCLK/1
		RCC_PCLK2Config(RCC_HCLK_Div1);		// PCLK2 = HCLK/1
		RCC_PCLK1Config(RCC_HCLK_Div1);		// PCLK1 = HCLK/1

		// PLLCLK = (8MHzHSE/2) * 6 = 24 MHz
		RCC_PREDIV1Config(RCC_PREDIV1_Source_HSE, RCC_PREDIV1_Div2);
		RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_6);

		RCC_PLLCmd(ENABLE);		// Enable the PLL
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);	// Wait for it to be ready

		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);    // Use PLL as SYSCLK

		while (RCC_GetSYSCLKSource() != 0x08);    // Wait for switch over to happen
	}
}

void ADCInit(void)
{
    //Enable ADC1, GPIOA, and GPIOB
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1  |
    					   RCC_APB2Periph_GPIOA |
    					   RCC_APB2Periph_GPIOB |
    					   RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure; //Variable used to setup the GPIO pins

    //Configure ADC pins (PA0 -> Channel 1 to PA7 -> Channel 8) as analog inputs
    GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |
    							  GPIO_Pin_1 |
    							  GPIO_Pin_2 |
    							  GPIO_Pin_3 |
    							  GPIO_Pin_4 |
    							  GPIO_Pin_5 |
    							  GPIO_Pin_6 |
    							  GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &GPIO_InitStructure);		// init batt pin

    ADC_InitTypeDef ADC_InitStructure;
    //ADC1 configuration

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    //We will convert multiple channels
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    //select single conversion mode
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    //select no external triggering
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    //right 12-bit data alignment in ADC data register
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    //9 channels conversion
    ADC_InitStructure.ADC_NbrOfChannel = 9;
    //load structure values to control and status registers
    ADC_Init(ADC1, &ADC_InitStructure);

    //configure each channel
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 7, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 9, ADC_SampleTime_41Cycles5);

    //Enable ADC1
    ADC_Cmd(ADC1, ENABLE);

    //enable DMA for ADC
    ADC_DMACmd(ADC1, ENABLE);

    //Enable ADC1 reset calibration register
    ADC_ResetCalibration(ADC1);
    //Check the end of ADC1 reset calibration register
    while(ADC_GetResetCalibrationStatus(ADC1));
    //Start ADC1 calibration
    ADC_StartCalibration(ADC1);
    //Check the end of ADC1 calibration
    while(ADC_GetCalibrationStatus(ADC1));
}

void DMAInit(void)
{
    //enable DMA1 clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    //create DMA structure
    DMA_InitTypeDef  DMA_InitStructure;

    //reset DMA1 channe1 to default values;
    DMA_DeInit(DMA1_Channel1);
    //channel will be used for memory to memory transfer
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    //setting normal mode (non circular)
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    //medium priority
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    //source and destination data size word=32bit
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    //automatic memory destination increment enable.
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    //source address increment disable
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    //Location assigned to peripheral register will be source
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    //chunk of data to be transfered
    DMA_InitStructure.DMA_BufferSize = ADC_ELEMENTS;							// 8 channels + Batt sense
    //source and destination start addresses
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_Buffer;
    //send values to DMA registers
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel1, ENABLE); //Enable the DMA1 - Channel1

    DMA_DeInit(DMA1_Channel5);
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = PPM_ELEMENTS;							// 8 channels + 9 space + end
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)TIM2_CCR1;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)PPM_Buffer;
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);

	// turning DMA on
	DMA_Cmd(DMA1_Channel5, ENABLE);
}

void TIM2Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure ;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	TIM_TimeBaseStructure.TIM_Period = 22000 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 24 - 1;    // 24 MHz / 24 (1 MHz tick)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

	TIM_OC1Init(TIM2, &TIM_OCInitStructure);

	TIM_DMAConfig(TIM2, TIM_DMABase_CCR1, TIM_DMABurstLength_1Transfer);
	// "connecting" DMA and TIM2
	TIM_DMACmd(TIM2, TIM_DMA_CC1, ENABLE);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// turning on TIM3 and PWM outputs
	TIM_Cmd(TIM2, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	// Enable the TIM2 gloabal Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TrimInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; //Variable used to setup the GPIO pins

	//Configure ADC pins (PA0 -> Channel 1 to PA7 -> Channel 8) as analog inputs
	GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 |
								  GPIO_Pin_9 |
	   							  GPIO_Pin_10 |
	   							  GPIO_Pin_11 |
	   							  GPIO_Pin_12 |
	   							  GPIO_Pin_13 |
	   							  GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void TIM2_IRQHandler(void)
{
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	// move the rest to adc interrupt*********************************************************************************************
	int i;
	u16 temp_adc_buffer[8];

	for (i = 0; i < 8; i++)
	{
		if (Reverse[i] == 1)
			temp_adc_buffer[i] = 4095 - ADC_Buffer[i];
		else
			temp_adc_buffer[i] = ADC_Buffer[i];
	}

	for (i = 1; i < PPM_ELEMENTS - 1; ++i)	// Ignore first and last ppm value. Should always be 400 and 21999 respectively
	{
		if (i % 2 == 1)
		{
			PPM_Buffer[i] = PPM_Buffer[i - 1] + temp_adc_buffer[(i - 1) / 2]/4 + 600 + Trim_Vals[(i - 1) / 2];	//https://en.wikipedia.org/wiki/Gamma_correction
		}
		else
		{
			PPM_Buffer[i] = PPM_Buffer[i - 1] + 400;
		}
	}

	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
}

void SysTick_Handler(void)
{
	static uint8_t high_time[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	static uint8_t low_time[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	static uint8_t prev_state[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t state[12];
	int i;

	state[0] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8);	// Trims
	state[1] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9);
	state[2] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10);
	state[3] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11);
	state[4] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);
	state[5] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_13);
	state[6] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_14);
	state[7] = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15);

	state[8] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2);	// Buttons
	state[9] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
	state[10] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4);
	state[11] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);

	for (i = 0; i < 7; ++i)
	{
		if ((state[i] == 1) & (prev_state[i] == 0))
		{
			high_time[i]++;
			low_time[i] = 0;
			if (high_time[i] >= DEBOUNCE_REF)
			{
				high_time[i] = DEBOUNCE_REF + 1;
				prev_state[i] = 1;
				if (i % 2 == 1)
				{
					Trim_Vals[i / 2] = Trim_Vals[i / 2] + TRIM_STEP;
					if (Trim_Vals[i / 2] > MAX_TRIM)
						Trim_Vals[i / 2] = MAX_TRIM;
				}
				else
				{
					Trim_Vals[i / 2] = Trim_Vals[i / 2] - TRIM_STEP;
					if (Trim_Vals[i / 2] < -MAX_TRIM)
						Trim_Vals[i / 2] = -MAX_TRIM;
				}
			}
		}
		else
		{
			high_time[i] = 0;
			low_time[i]++;
			if (low_time[i] >= DEBOUNCE_REF)
			{
				low_time[i] = DEBOUNCE_REF + 1;
				prev_state[i] = 0;
			}
		}
	}

	for (i = 8; i < 11; ++i)
	{
		if ((state[i] == 1) & (prev_state[i] == 0))
		{
			high_time[i]++;
			low_time[i] = 0;
			if (high_time[i] >= DEBOUNCE_REF)
			{
				high_time[i] = DEBOUNCE_REF + 1;
				prev_state[i] = 1;
				switch (i)
				{
					case 8:	// Up

						break;
					case 9:	// Down

						break;
					case 10:	// Enter

						break;
					case 11:	// Back

						break;
					default:
						break;
				}
			}
		}
		else
		{
			high_time[i] = 0;
			low_time[i]++;
			if (low_time[i] >= DEBOUNCE_REF)
			{
				low_time[i] = DEBOUNCE_REF + 1;
				prev_state[i] = 0;
			}
		}
	}
}
