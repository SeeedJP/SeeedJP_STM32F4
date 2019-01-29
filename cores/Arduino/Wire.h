#pragma once

#include "Arduino.h"
#include <vector>	// std::vector

class TwoWire
{
public:
	TwoWire(int port, int sdaPin, int sclPin);
	~TwoWire();

	void begin();

	void beginTransmission(int address);
	void write(uint8_t data);
	uint8_t endTransmission();

	uint8_t requestFrom(int address, int quantity);
	int available() const;
	int read();

private:
	int _Port;
	int _SdaPin;
	int _SclPin;
	void* _Handle;	// I2C_HandleTypeDef*

	int _Address;
	std::vector<uint8_t> _Buffer;

};

#if defined ARDUINO_WIO_3G || defined ARDUINO_WIO_LTE_M1NB1_BG96
extern TwoWire Wire;
#endif
