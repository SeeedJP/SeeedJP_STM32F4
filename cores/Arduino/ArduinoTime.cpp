#include "Arduino.h"
#include <stm32f4xx_hal.h>

void delay(unsigned long ms)
{
	HAL_Delay(ms);
}

void delayMicroseconds(int us)
{
	unsigned long end = micros() + us;
	long diff;
	do {
		diff = micros() - end;
	} while (diff < 0);
}

unsigned long micros()
{
	unsigned long ms;
	uint32_t sysTickCurrent;
	do {
		ms = millis();
		sysTickCurrent = SysTick->VAL;
	} while (millis() != ms || SysTick->VAL > sysTickCurrent);

	uint32_t sysTickReload = SysTick->LOAD;

	return ms * 1000 + (sysTickReload - sysTickCurrent) * 1000 / (sysTickReload + 1);
}

unsigned long millis()
{
	return HAL_GetTick();
}
