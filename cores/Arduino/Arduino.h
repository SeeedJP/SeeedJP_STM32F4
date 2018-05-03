#pragma once

#include "ArduinoSetupAndLoop.h"
#include "Print.h"

#include <stddef.h>	// size_t
#include <stdint.h>	// uint8_t
#include <vector>	// std::vector
#include <queue>	// std::queue

typedef uint8_t byte;

// ArduinoDigitalIO.cpp

#define OUTPUT			(0)
#define INPUT			(1)
#define INPUT_PULLUP	(2)
#define INPUT_ANALOG	(3)
#define OUTPUT_ANALOG	(4)

#define LOW		(0x0)
#define HIGH	(0x1)

int digitalRead(int pin);
void digitalWrite(int pin, int value);
void pinMode(int pin, int mode);

// ArduinoAnalogIO.cpp

int analogRead(int pin);
void analogWrite(int pin, int value);

// ArduinoAdvancedIO.cpp
int pulseIn(int pin, int value, unsigned long timeout = 1000000);

// ArduinoTime.cpp

void delay(unsigned long ms);
void delayMicroseconds(int us);
unsigned long micros();
unsigned long millis();

// ArduinoCommunication.cpp

#define SERIAL_8N1	(0x06)
#define SERIAL_8E1	(0x26)
#define SERIAL_8O1	(0x36)

class HardwareSerial : public Print
{
public:
	HardwareSerial(int port, int txPin, int rxPin);
	~HardwareSerial();

	int available() const;
	void begin(long speed, int config = SERIAL_8N1);
	void end();
	void flush();
	//size_t print(const char* val);
	//size_t print(char val);
	//size_t println(const char* val);
	int read();

	void setReadBufferSize(int size);
	bool isReadOverflow() const;
	void clearReadOverflow();

public:
	virtual size_t write(uint8_t val);

private:
	int _Port;
	int _TxPin;
	int _RxPin;

	int _RxBufferCapacity;

	std::vector<uint8_t> _TxBuffers[2];
	int _TxBufferCurrent;

	uint8_t _RxByte;
	std::queue<uint8_t> _RxBuffer;	// TODO Poor performance. -> circular_buffer
	bool _RxBufferOverflow;

	void EnableIRQ() const;
	void DisableIRQ() const;
	bool TxIsReady() const;
	void TxWriteAsync(const uint8_t* data, int dataSize, const uint8_t* data2 = NULL, int data2Size = 0);
	void RxReadStart();
	int RxReadedSize() const;
	int RxReadByte();

public:
	void TxWriteCallback();	// For internal use only.
	void RxReadCallback();	// For internal use only.

};

// ArduinoWire.cpp

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
