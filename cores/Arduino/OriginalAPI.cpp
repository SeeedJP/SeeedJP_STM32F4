#include "Arduino.h"
#include <stm32f4xx_hal.h>

void enterLowPowerMode(LowPowerModeType mode)
{
	switch (mode) {
	case LOW_POWER_MODE_SLEEP:
		HAL_SuspendTick();
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		HAL_ResumeTick();
		break;
	}
}
