#include "DeviceSupportLibrary.h"

#ifdef STM32F439xx

#define PINNAME_TO_PIN(port, pin) ((port - 'A') * 16 + pin)

////////////////////////////////////////
// GPIO

GPIO_TypeDef* const DslGpioRegs[] = {
	GPIOA,
	GPIOB,
	GPIOC,
	GPIOD,
	GPIOE,
	GPIOF,
	GPIOG,
	GPIOH,
	GPIOI,
	GPIOJ,
	GPIOK,
};

const uint16_t DslGpioPins[] = {
	GPIO_PIN_0,
	GPIO_PIN_1,
	GPIO_PIN_2,
	GPIO_PIN_3,
	GPIO_PIN_4,
	GPIO_PIN_5,
	GPIO_PIN_6,
	GPIO_PIN_7,
	GPIO_PIN_8,
	GPIO_PIN_9,
	GPIO_PIN_10,
	GPIO_PIN_11,
	GPIO_PIN_12,
	GPIO_PIN_13,
	GPIO_PIN_14,
	GPIO_PIN_15,
};

void DslGpioClockEnable(GPIO_TypeDef* reg)
{
	if (reg == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
	else if (reg == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
	else if (reg == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
	else if (reg == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
	else if (reg == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
	else if (reg == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
	else if (reg == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
	else if (reg == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();
	else if (reg == GPIOI) __HAL_RCC_GPIOI_CLK_ENABLE();
	else if (reg == GPIOJ) __HAL_RCC_GPIOJ_CLK_ENABLE();
	else if (reg == GPIOK) __HAL_RCC_GPIOK_CLK_ENABLE();
}

////////////////////////////////////////
// ADC

ADC_TypeDef* const DslAdcRegs[] = {
	ADC1,
	ADC2,
	ADC3,
};

const uint32_t DslAdcChannels[] = {
	ADC_CHANNEL_0,
	ADC_CHANNEL_1,
	ADC_CHANNEL_2,
	ADC_CHANNEL_3,
	ADC_CHANNEL_4,
	ADC_CHANNEL_5,
	ADC_CHANNEL_6,
	ADC_CHANNEL_7,
	ADC_CHANNEL_8,
	ADC_CHANNEL_9,
	ADC_CHANNEL_10,
	ADC_CHANNEL_11,
	ADC_CHANNEL_12,
	ADC_CHANNEL_13,
	ADC_CHANNEL_14,
	ADC_CHANNEL_15,
	ADC_CHANNEL_16,
	ADC_CHANNEL_17,
	ADC_CHANNEL_18,
};

void DslAdcClockEnable(ADC_TypeDef* reg)
{
	if (reg == ADC1) __HAL_RCC_ADC1_CLK_ENABLE();
	else if (reg == ADC2) __HAL_RCC_ADC2_CLK_ENABLE();
	else if (reg == ADC3) __HAL_RCC_ADC3_CLK_ENABLE();
}

uint32_t DslAdcChannel(ADC_TypeDef* reg, int pin)
{
	switch (pin) {
	case PINNAME_TO_PIN('A', 0):
		if (reg == ADC1 || reg == ADC2 || reg == ADC3) return 0;
	case PINNAME_TO_PIN('A', 1):
		if (reg == ADC1 || reg == ADC2 || reg == ADC3) return 1;
	case PINNAME_TO_PIN('A', 2):
		if (reg == ADC1 || reg == ADC2 || reg == ADC3) return 2;
	case PINNAME_TO_PIN('A', 3):
		if (reg == ADC1 || reg == ADC2 || reg == ADC3) return 3;
	case PINNAME_TO_PIN('A', 4):
		if (reg == ADC1 || reg == ADC2) return 4;
	case PINNAME_TO_PIN('A', 5):
		if (reg == ADC1 || reg == ADC2) return 5;
	case PINNAME_TO_PIN('A', 6):
		if (reg == ADC1 || reg == ADC2) return 6;
	case PINNAME_TO_PIN('A', 7):
		if (reg == ADC1 || reg == ADC2) return 7;
	case PINNAME_TO_PIN('B', 0):
		if (reg == ADC1 || reg == ADC2) return 8;
	case PINNAME_TO_PIN('B', 1):
		if (reg == ADC1 || reg == ADC2) return 9;
	case PINNAME_TO_PIN('C', 0):
		if (reg == ADC1 || reg == ADC2 || reg == ADC3) return 10;
	case PINNAME_TO_PIN('C', 1):
		if (reg == ADC1 || reg == ADC2 || reg == ADC3) return 11;
	case PINNAME_TO_PIN('C', 2):
		if (reg == ADC1 || reg == ADC2 || reg == ADC3) return 12;
	case PINNAME_TO_PIN('C', 3):
		if (reg == ADC1 || reg == ADC2 || reg == ADC3) return 13;
	case PINNAME_TO_PIN('C', 4):
		if (reg == ADC1 || reg == ADC2) return 14;
	case PINNAME_TO_PIN('C', 5):
		if (reg == ADC1 || reg == ADC2) return 15;
	}

	return 0;	// TODO Fail.
}

////////////////////////////////////////
// DAC

DAC_TypeDef* const DslDacRegs[] = {
	DAC1
};

const uint32_t DslDacChannels[] = {
	DAC_CHANNEL_1,
	DAC_CHANNEL_2
};

void DslDacClockEnable(DAC_TypeDef* reg)
{
	if (reg == DAC1) { __HAL_RCC_DAC_CLK_ENABLE(); }
}

uint32_t DslDacChannel(DAC_TypeDef* reg, int pin)
{
	switch (pin) {
	case PINNAME_TO_PIN('A', 4):
		if (reg == DAC1) return DAC_CHANNEL_1;
		break;
	case PINNAME_TO_PIN('A', 5):
		if (reg == DAC1) return DAC_CHANNEL_2;
		break;
	}

	return 0;	// TODO Fail.
}

////////////////////////////////////////
// UART

USART_TypeDef* const DslUartRegs[] = {
	USART1,
	USART2,
	USART3,
	UART4,
	UART5,
	USART6,
	UART7,
	UART8,
};

const IRQn_Type DslUartIRQs[] = {
	USART1_IRQn,
	USART2_IRQn,
	USART3_IRQn,
	UART4_IRQn,
	UART5_IRQn,
	USART6_IRQn,
	UART7_IRQn,
	UART8_IRQn,
};

void* DslUartInstances[] = {	// HardwareSerial* []
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
};

void DslUartClockEnable(USART_TypeDef* reg)
{
	if (reg == USART1) __HAL_RCC_USART1_CLK_ENABLE();
	else if (reg == USART2) __HAL_RCC_USART2_CLK_ENABLE();
	else if (reg == USART3) __HAL_RCC_USART3_CLK_ENABLE();
	else if (reg == UART4) __HAL_RCC_UART4_CLK_ENABLE();
	else if (reg == UART5) __HAL_RCC_UART5_CLK_ENABLE();
	else if (reg == USART6) __HAL_RCC_USART6_CLK_ENABLE();
	else if (reg == UART7) __HAL_RCC_UART7_CLK_ENABLE();
	else if (reg == UART8) __HAL_RCC_UART8_CLK_ENABLE();
}

uint32_t DslUartGpioAlternate(USART_TypeDef* reg, int pin)
{
	if (reg == USART1) {
		switch (pin) {
		case PINNAME_TO_PIN('A', 9):
		case PINNAME_TO_PIN('A', 10):
		case PINNAME_TO_PIN('B', 6):
		case PINNAME_TO_PIN('B', 7):
			return GPIO_AF7_USART1;
		}
	}
	else if (reg == USART2) {
		switch (pin) {
		case PINNAME_TO_PIN('A', 2):
		case PINNAME_TO_PIN('A', 3):
		case PINNAME_TO_PIN('D', 5):
		case PINNAME_TO_PIN('D', 6):
			return GPIO_AF7_USART2;
		}
	}
	else if (reg == USART3) {
		switch (pin) {
		case PINNAME_TO_PIN('B', 10):
		case PINNAME_TO_PIN('B', 11):
		case PINNAME_TO_PIN('C', 10):
		case PINNAME_TO_PIN('C', 11):
		case PINNAME_TO_PIN('D', 8):
		case PINNAME_TO_PIN('D', 9):
			return GPIO_AF7_USART3;
		}
	}
	else if (reg == UART4) {
		switch (pin) {
		case PINNAME_TO_PIN('A', 0):
		case PINNAME_TO_PIN('A', 1):
		case PINNAME_TO_PIN('C', 10):
		case PINNAME_TO_PIN('C', 11):
			return GPIO_AF8_UART4;
		}
	}
	else if (reg == UART5) {
		switch (pin) {
		case PINNAME_TO_PIN('C', 12):
		case PINNAME_TO_PIN('D', 2):
			return GPIO_AF8_UART5;
		}
	}
	else if (reg == USART6) {
		switch (pin) {
		case PINNAME_TO_PIN('C', 6):
		case PINNAME_TO_PIN('C', 7):
		case PINNAME_TO_PIN('G', 9):
		case PINNAME_TO_PIN('G', 14):
			return GPIO_AF8_USART6;
		}
	}
	else if (reg == UART7) {
		switch (pin) {
		case PINNAME_TO_PIN('E', 7):
		case PINNAME_TO_PIN('E', 8):
		case PINNAME_TO_PIN('F', 6):
		case PINNAME_TO_PIN('F', 7):
			return GPIO_AF8_UART7;
		}
	}
	else if (reg == UART8) {
		switch (pin) {
		case PINNAME_TO_PIN('E', 0):
		case PINNAME_TO_PIN('E', 1):
			return GPIO_AF8_UART8;
		}
	}

	return 0;	// TODO Fail.
}

int DslUartToPort(USART_TypeDef* reg)
{
	for (int i = 0; i < (int)(sizeof(DslUartRegs) / sizeof(DslUartRegs[0])); i++) {
		if (DslUartRegs[i] == reg) return i;
	}

	return -1;	// TODO Fail.
}

////////////////////////////////////////
// I2C

I2C_TypeDef* const DslI2cRegs[] = {
	I2C1,
	I2C2,
	I2C3,
};

void DslI2cClockEnable(I2C_TypeDef* reg)
{
	if (reg == I2C1) __HAL_RCC_I2C1_CLK_ENABLE();
	else if (reg == I2C2) __HAL_RCC_I2C2_CLK_ENABLE();
	else if (reg == I2C3) __HAL_RCC_I2C3_CLK_ENABLE();
}

uint32_t DslI2cGpioAlternate(I2C_TypeDef* reg, int pin)
{
	if (reg == I2C1) {
		switch (pin) {
		case PINNAME_TO_PIN('B', 6):
		case PINNAME_TO_PIN('B', 7):
		case PINNAME_TO_PIN('B', 8):
		case PINNAME_TO_PIN('B', 9):
			return GPIO_AF4_I2C1;
		}
	}
	else if (reg == I2C2) {
		switch (pin) {
		case PINNAME_TO_PIN('B', 10):
		case PINNAME_TO_PIN('B', 11):
		case PINNAME_TO_PIN('F', 0):
		case PINNAME_TO_PIN('F', 1):
		case PINNAME_TO_PIN('H', 4):
		case PINNAME_TO_PIN('H', 5):
			return GPIO_AF4_I2C2;
		}
	}
	else if (reg == I2C3) {
		switch (pin) {
		case PINNAME_TO_PIN('A', 8):
		case PINNAME_TO_PIN('C', 9):
		case PINNAME_TO_PIN('H', 7):
		case PINNAME_TO_PIN('H', 8):
			return GPIO_AF4_I2C3;
		}
	}

	return 0;	// TODO Fail.
}

////////////////////////////////////////
// Interrupt

void DslInterruptExtiEnable(int num)
{
	switch (num) {
	case 0:
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
		break;
	case 1:
		HAL_NVIC_EnableIRQ(EXTI1_IRQn);
		break;
	case 2:
		HAL_NVIC_EnableIRQ(EXTI2_IRQn);
		break;
	case 3:
		HAL_NVIC_EnableIRQ(EXTI3_IRQn);
		break;
	case 4:
		HAL_NVIC_EnableIRQ(EXTI4_IRQn);
		break;
	default:
		if (5 <= num && num <= 9) {
			HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
		}
		else if (10 <= num && num <= 15) {
			HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
		}
		break;
	}
}

#endif // STM32F439xx
