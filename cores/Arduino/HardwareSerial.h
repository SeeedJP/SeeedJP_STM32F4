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
	HardwareSerial(int port, int txPin, int rxPin, int ctsPin = -1, int rtsPin = -1);
	~HardwareSerial();

	void begin(long speed, int config = SERIAL_8N1);
	void end();

	int available() const;
	int read();

	int getReadBufferSize() const;
	void setReadBufferSize(int size);
	bool isReadOverflow() const;
	void clearReadOverflow();

	unsigned long getWriteTimeout() const;
	void setWriteTimeout(unsigned long timeout);

public:
	virtual size_t write(uint8_t val);
	void flush();

private:
	int _Port;
	int _TxPin;		// out
	int _RxPin;		// in
	int _CtsPin;	// in
	int _RtsPin;	// out

	int _RxBufferCapacity;
	unsigned long _TxTimeout;

	uint8_t _RxByte;
	std::queue<uint8_t> _RxBuffer;	// TODO Poor performance. -> circular_buffer
	bool _RxBufferOverflow;

	void EnableIRQ() const;
	void DisableIRQ() const;
	void RxReadStart();
	int RxReadedSize() const;
	int RxReadByte();

public:
	void RxReadCallback();	// For internal use only.

private:
	void TxWrite(const uint8_t* data, int dataSize);

	int HAL_UART_Transmit(void *huart_vp, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
	int UART_WaitOnFlagUntilTimeout(void *huart_vp, uint32_t Flag, int Status, uint32_t Tickstart, uint32_t Timeout);

};
