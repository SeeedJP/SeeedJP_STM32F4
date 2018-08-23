#pragma once

#include "ArduinoSetupAndLoop.h"
#include <stddef.h>	// size_t
#include <stdint.h>	// uint8_t

typedef bool boolean;
typedef uint8_t byte;

// ArduinoDigitalIO.cpp

#define HIGH	(0x1)
#define LOW		(0x0)

#define OUTPUT			(0)
#define INPUT			(1)
#define INPUT_PULLUP	(2)
#define INPUT_ANALOG	(3)
#define OUTPUT_ANALOG	(4)

int digitalRead(int pin);
void digitalWrite(int pin, int value);
void pinMode(int pin, int mode);

// ArduinoAnalogIO.cpp

int analogRead(int pin);
void analogWrite(int pin, int value);

// ArduinoAdvancedIO.cpp

unsigned long pulseIn(int pin, int value, unsigned long timeout = 1000000);

// ArduinoTime.cpp

void delay(unsigned long ms);
void delayMicroseconds(int us);
unsigned long micros();
unsigned long millis();

// ArduinoMath.cpp

long map(long x, long in_min, long in_max, long out_min, long out_max);

// ArduinoInterrupt.cpp

#define CHANGE	(0)
#define FALLING	(1)
#define RISING	(2)

void attachInterrupt(int pin, void (*userFunc)(void), int mode);

// OriginalAPI.cpp

enum LowPowerModeType {
	LOW_POWER_MODE_SLEEP,
};

void suspendSysTick();
void resumeSysTick();
void enterLowPowerMode(LowPowerModeType mode);

// Default includes

#include "WString.h"
#include "HardwareSerial.h"
#include "Wire.h"	// TODO Remove in the future
