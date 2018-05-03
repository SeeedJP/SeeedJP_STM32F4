#include "Arduino.h"
#include <stm32f4xx_hal.h>
#include <DeviceSupportLibrary.h>

#define ADC_CORE			(0)
#define DAC_CORE			(0)
#define CONVERSION_TIMEOUT	(10)

static ADC_HandleTypeDef Adc0Handle;
static DAC_HandleTypeDef Dac0Handle;

int analogRead(int pin)
{
	static bool first = true;

	if (first) {
		DslAdcClockEnable(DslAdcRegs[ADC_CORE]);

		Adc0Handle.Instance = DslAdcRegs[ADC_CORE];
		Adc0Handle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
		Adc0Handle.Init.Resolution = ADC_RESOLUTION_12B;
		Adc0Handle.Init.ScanConvMode = DISABLE;
		Adc0Handle.Init.ContinuousConvMode = DISABLE;
		Adc0Handle.Init.DiscontinuousConvMode = DISABLE;
		Adc0Handle.Init.NbrOfDiscConversion = 0;
		Adc0Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
		Adc0Handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
		Adc0Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
		Adc0Handle.Init.NbrOfConversion = 1;
		Adc0Handle.Init.DMAContinuousRequests = DISABLE;
		Adc0Handle.Init.EOCSelection = DISABLE;
		if (HAL_ADC_Init(&Adc0Handle) != HAL_OK) return 0;

		first = false;
	}

	ADC_ChannelConfTypeDef chConfig;
	chConfig.Channel = DslAdcChannel(DslAdcRegs[ADC_CORE], pin);
	chConfig.Rank = 1;
	chConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	chConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&Adc0Handle, &chConfig) != HAL_OK) return 0;

	if (HAL_ADC_Start(&Adc0Handle) != HAL_OK) return 0;
	if (HAL_ADC_PollForConversion(&Adc0Handle, CONVERSION_TIMEOUT) != HAL_OK) return 0;	// TODO
	if ((HAL_ADC_GetState(&Adc0Handle) & HAL_ADC_STATE_EOC_REG) != HAL_ADC_STATE_EOC_REG) return 0;
	return HAL_ADC_GetValue(&Adc0Handle);
}

void analogWrite(int pin, int value)
{
	static bool first = true;

	if (first) {
		Dac0Handle.Instance = DslDacRegs[DAC_CORE];
		if (HAL_DAC_Init(&Dac0Handle) != HAL_OK) return;

		first = false;
	}

	const auto channel = DslDacChannel(DslDacRegs[DAC_CORE], pin); 
	DAC_ChannelConfTypeDef chConfig;
	chConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	chConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&Dac0Handle, &chConfig, channel) != HAL_OK) return;

	if (HAL_DAC_Start(&Dac0Handle, channel) != HAL_OK) return;
	HAL_DAC_SetValue(&Dac0Handle, channel, DAC_ALIGN_12B_R, value);
}
