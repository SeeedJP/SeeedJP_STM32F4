#pragma once

#include "Arduino.h"
#include "Print.h"
#include <vector>	// std::vector
#include <queue>	// std::queue

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
