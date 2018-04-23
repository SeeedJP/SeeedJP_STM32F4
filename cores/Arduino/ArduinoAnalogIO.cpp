#include "Arduino.h"
#include <stm32f4xx_hal.h>
#include <DeviceSupportLibrary.h>

#define ADC_CORE			(0)
#define CONVERSION_TIMEOUT	(10)

static ADC_HandleTypeDef Adc0Handle;

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
