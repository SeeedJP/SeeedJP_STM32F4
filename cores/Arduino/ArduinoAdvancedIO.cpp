#include "Arduino.h"
#include <stm32f4xx_hal.h>
#include <DeviceSupportLibrary.h>

static unsigned long MicrosDiff(unsigned long begin, unsigned long end)
{
	return end - begin;
}

unsigned long pulseIn(int pin, int value, unsigned long timeout)
{
	auto begin = micros();

	// wait for any previous pulse to end
	while (digitalRead(pin) == value) if (MicrosDiff(begin, micros()) >= timeout) return 0;

	// wait for the pulse to start
	while (digitalRead(pin) != value) if (MicrosDiff(begin, micros()) >= timeout) return 0;
	auto pulseBegin = micros();

	// wait for the pulse to stop
	while (digitalRead(pin) == value) if (MicrosDiff(begin, micros()) >= timeout) return 0;
	auto pulseEnd = micros();

	return MicrosDiff(pulseBegin, pulseEnd);
}
