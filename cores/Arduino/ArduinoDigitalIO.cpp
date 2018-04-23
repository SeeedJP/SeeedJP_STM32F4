#include "Arduino.h"
#include <stm32f4xx_hal.h>
#include <DeviceSupportLibrary.h>

void pinMode(int pin, int mode)
{
	DslGpioClockEnable(DslGpioRegs[pin / 16]);

	GPIO_InitTypeDef gpioInit;
	gpioInit.Pin = DslGpioPins[pin % 16];
	gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	switch (mode) {
	case OUTPUT:
		gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
		gpioInit.Pull = GPIO_NOPULL;
		break;
	case INPUT:
		gpioInit.Mode = GPIO_MODE_INPUT;
		gpioInit.Pull = GPIO_NOPULL;
		break;
	case INPUT_PULLUP:
		gpioInit.Mode = GPIO_MODE_INPUT;
		gpioInit.Pull = GPIO_PULLUP;
		break;
	case INPUT_ANALOG:
		gpioInit.Mode = GPIO_MODE_ANALOG;
		gpioInit.Pull = GPIO_NOPULL;
		break;
	}
	HAL_GPIO_Init(DslGpioRegs[pin / 16], &gpioInit);
}

int digitalRead(int pin)
{
	return HAL_GPIO_ReadPin(DslGpioRegs[pin / 16], DslGpioPins[pin % 16]) == GPIO_PIN_SET ? HIGH : LOW;
}

void digitalWrite(int pin, int value)
{
	HAL_GPIO_WritePin(DslGpioRegs[pin / 16], DslGpioPins[pin % 16], value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
