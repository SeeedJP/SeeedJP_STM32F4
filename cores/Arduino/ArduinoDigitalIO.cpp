#include "Arduino.h"
#include <stm32f4xx_hal.h>
#include <DeviceSupportLibrary.h>

extern void internalInitializeDacPorts(int pin);

void pinMode(int pin, int mode)
{
	DslGpioClockEnable(DslGpioRegs[pin / 16]);

	GPIO_InitTypeDef gpioInit;
	gpioInit.Pin = DslGpioPins[pin % 16];
	gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	
	// TODO:
	//   1. Initialize sequence at each modes are mode-dependent.
	//     So we have to finish intializing at this point.
	//     (ex: currently analogRead() has ADC initializing code)
	//   2. We have to save current state at each pins,
	//     and do de-initializing if mode has changed.
	//   --> pinMode() maybe moves to new file location better than ArduinoDigitalIO.cpp.
	switch (mode) {
	case OUTPUT:
		gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
		gpioInit.Pull = GPIO_NOPULL;
		break;
	case OUTPUT_OPEN_DRAIN:
		gpioInit.Mode = GPIO_MODE_OUTPUT_OD;
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

	case OUTPUT_ANALOG:
		// Initialize DAC with related ports.
		internalInitializeDacPorts(pin);
		return;
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
