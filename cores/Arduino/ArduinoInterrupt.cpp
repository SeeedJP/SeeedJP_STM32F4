#include "Arduino.h"
#include <stm32f4xx_hal.h>
#include <DeviceSupportLibrary.h>
#include <map>

////////////////////////////////////////
// Copy from HAL_GPIO_Init()(stm32f4xx_hal_gpio.c)
//

#define GPIO_NUMBER           16U

////////////////////////////////////////

static std::map<uint16_t, void(*)()> ExtiCallback;

void attachInterrupt(int pin, void(*userFunc)(void), int mode)
{
	auto it = ExtiCallback.find(DslGpioPins[pin % 16]);
	if (it != ExtiCallback.end()) {
		it->second = userFunc;
	}
	else {
		ExtiCallback.insert(std::make_pair(DslGpioPins[pin % 16], userFunc));
	}

	////////////////////////////////////////
	// Copy from HAL_GPIO_Init()(stm32f4xx_hal_gpio.c)
	//

	uint32_t position;
	uint32_t ioposition = 0x00U;
	uint32_t iocurrent = 0x00U;
	uint32_t temp = 0x00U;

	/* Configure the port pins */
	for (position = 0U; position < GPIO_NUMBER; position++)
	{
		/* Get the IO position */
		ioposition = 0x01U << position;
		/* Get the current IO position */
		iocurrent = (uint32_t)(DslGpioPins[pin % 16]) & ioposition;

		if (iocurrent == ioposition)
		{
			/*--------------------- EXTI Mode Configuration ------------------------*/
			/* Configure the External Interrupt or event for the current IO */

			/* Enable SYSCFG Clock */
			__HAL_RCC_SYSCFG_CLK_ENABLE();

			temp = SYSCFG->EXTICR[position >> 2U];
			temp &= ~(0x0FU << (4U * (position & 0x03U)));
			temp |= ((uint32_t)(pin / 16) << (4U * (position & 0x03U)));
			SYSCFG->EXTICR[position >> 2U] = temp;

			/* Clear EXTI line configuration */
			temp = EXTI->IMR;
			temp &= ~((uint32_t)iocurrent);
			//if ((GPIO_Init->Mode & GPIO_MODE_IT) == GPIO_MODE_IT)
			//{
				temp |= iocurrent;
			//}
			EXTI->IMR = temp;

			temp = EXTI->EMR;
			temp &= ~((uint32_t)iocurrent);
			//if ((GPIO_Init->Mode & GPIO_MODE_EVT) == GPIO_MODE_EVT)
			//{
			//	temp |= iocurrent;
			//}
			EXTI->EMR = temp;

			/* Clear Rising Falling edge configuration */
			temp = EXTI->RTSR;
			temp &= ~((uint32_t)iocurrent);
			if (mode == RISING || mode == CHANGE)
			{
				temp |= iocurrent;
			}
			EXTI->RTSR = temp;

			temp = EXTI->FTSR;
			temp &= ~((uint32_t)iocurrent);
			if (mode == FALLING || mode == CHANGE)
			{
				temp |= iocurrent;
			}
			EXTI->FTSR = temp;
		}
	}

	////////////////////////////////////////

	DslInterruptExtiEnable(pin % 16);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	auto it = ExtiCallback.find(GPIO_Pin);
	if (it != ExtiCallback.end()) {
		it->second();
	}
}
