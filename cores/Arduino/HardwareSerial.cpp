#include "Arduino.h"
#include "HardwareSerial.h"
#include <stm32f4xx_hal.h>
#include <DeviceSupportLibrary.h>
#include <stm32f4xx_it.h>
#include <string.h>

HardwareSerial::HardwareSerial(int port, int txPin, int rxPin, int ctsPin, int rtsPin)
{
	_Port = port;
	_TxPin = txPin;
	_RxPin = rxPin;
	_CtsPin = ctsPin;
	_RtsPin = rtsPin;

	_RxBufferCapacity = 0;

	DslUartInstances[_Port] = this;
}

HardwareSerial::~HardwareSerial()
{
	DslUartInstances[_Port] = NULL;
}

void HardwareSerial::begin(long speed, int config)
{
	auto handle = &ItUartHandles[_Port];

	_TxBuffers[0].clear();
	_TxBuffers[1].clear();
	_TxBufferCurrent = 0;

	while (!_RxBuffer.empty()) _RxBuffer.pop();
	_RxBufferOverflow = false;

	if (_CtsPin >= 0) {
		pinMode(_CtsPin, INPUT);
	}
	if (_RtsPin >= 0) {
		pinMode(_RtsPin, OUTPUT);
		digitalWrite(_RtsPin, LOW);
	}

	DslGpioClockEnable(DslGpioRegs[_TxPin / 16]);
	DslGpioClockEnable(DslGpioRegs[_RxPin / 16]);

	GPIO_InitTypeDef  gpioInit;
	gpioInit.Mode = GPIO_MODE_AF_PP;
	gpioInit.Pull = GPIO_PULLUP;
	gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	gpioInit.Pin = DslGpioPins[_TxPin % 16];
	gpioInit.Alternate = DslUartGpioAlternate(DslUartRegs[_Port], _TxPin);
	HAL_GPIO_Init(DslGpioRegs[_TxPin / 16], &gpioInit);

	gpioInit.Pin = DslGpioPins[_RxPin % 16];
	gpioInit.Alternate = DslUartGpioAlternate(DslUartRegs[_Port], _RxPin);;
	HAL_GPIO_Init(DslGpioRegs[_RxPin / 16], &gpioInit);

	HAL_NVIC_SetPriority(DslUartIRQs[_Port], 0, 1);
	EnableIRQ();

	DslUartClockEnable(DslUartRegs[_Port]);

	handle->Instance = DslUartRegs[_Port];
	handle->Init.BaudRate = speed;
	handle->Init.Mode = UART_MODE_TX_RX;
	handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	handle->Init.OverSampling = UART_OVERSAMPLING_16;

	switch (config) {
	case SERIAL_8N1:
		handle->Init.WordLength = UART_WORDLENGTH_8B;
		handle->Init.Parity = UART_PARITY_NONE;
		handle->Init.StopBits = UART_STOPBITS_1;
		break;
	case SERIAL_8E1:
		handle->Init.WordLength = UART_WORDLENGTH_9B;
		handle->Init.Parity = UART_PARITY_EVEN;
		handle->Init.StopBits = UART_STOPBITS_1;
		break;
	case SERIAL_8O1:
		handle->Init.WordLength = UART_WORDLENGTH_9B;
		handle->Init.Parity = UART_PARITY_ODD;
		handle->Init.StopBits = UART_STOPBITS_1;
		break;
	default:
		return;	// TODO Fail.
	}

	if (HAL_UART_Init(handle) != HAL_OK) return;

	RxReadStart();
}

void HardwareSerial::end()
{
	auto handle = &ItUartHandles[_Port];

	DisableIRQ();

	HAL_UART_DeInit(handle);

	_TxBuffers[0].clear();
	_TxBuffers[1].clear();

	while (!_RxBuffer.empty()) _RxBuffer.pop();
}

int HardwareSerial::read()
{
	return RxReadByte();
}

void HardwareSerial::flush()
{
	while (!TxIsReady()) {}
}

int HardwareSerial::available() const
{
	return RxReadedSize();
}

void HardwareSerial::setReadBufferSize(int size)
{
	_RxBufferCapacity = size;
}

bool HardwareSerial::isReadOverflow() const
{
	return _RxBufferOverflow;
}

void HardwareSerial::clearReadOverflow()
{
	_RxBufferOverflow = false;
}

size_t HardwareSerial::write(uint8_t val)
{
	TxWriteAsync((const uint8_t*)&val, 1);

	return 1;
}

void HardwareSerial::EnableIRQ() const
{
	HAL_NVIC_EnableIRQ(DslUartIRQs[_Port]);
}

void HardwareSerial::DisableIRQ() const
{
	HAL_NVIC_DisableIRQ(DslUartIRQs[_Port]);
}

bool HardwareSerial::TxIsReady() const
{
	auto handle = &ItUartHandles[_Port];

	return handle->gState == HAL_UART_STATE_READY;
}

void HardwareSerial::TxWriteAsync(const uint8_t* data, int dataSize, const uint8_t* data2, int data2Size)
{
	auto handle = &ItUartHandles[_Port];

	DisableIRQ();

	auto transmitting = !_TxBuffers[_TxBufferCurrent].empty();

	auto txBuffer = &_TxBuffers[(_TxBufferCurrent + 1) % 2];
	auto txBufferSize = txBuffer->size();
	txBuffer->resize(txBufferSize + dataSize + data2Size);
	memcpy(&(*txBuffer)[txBufferSize], data, dataSize);
	if (data2Size >= 1) memcpy(&(*txBuffer)[txBufferSize + dataSize], data2, data2Size);

	if (!transmitting) {
		_TxBufferCurrent = (_TxBufferCurrent + 1) % 2;
		HAL_UART_Transmit_IT(handle, &_TxBuffers[_TxBufferCurrent][0], _TxBuffers[_TxBufferCurrent].size());
	}

	EnableIRQ();
}

void HardwareSerial::TxWriteCallback()
{
	auto handle = &ItUartHandles[_Port];

	_TxBuffers[_TxBufferCurrent].clear();

	_TxBufferCurrent = (_TxBufferCurrent + 1) % 2;
	if (!_TxBuffers[_TxBufferCurrent].empty()) {
		HAL_UART_Transmit_IT(handle, &_TxBuffers[_TxBufferCurrent][0], _TxBuffers[_TxBufferCurrent].size());
	}
}

void HardwareSerial::RxReadStart()
{
	auto handle = &ItUartHandles[_Port];

	HAL_UART_Receive_IT(handle, &_RxByte, 1);
}

int HardwareSerial::RxReadedSize() const
{
	DisableIRQ();

	auto size = _RxBuffer.size();

	EnableIRQ();

	return size;
}

int HardwareSerial::RxReadByte()
{
	DisableIRQ();

	int data;
	if (_RxBuffer.size() <= 0) {
		data = -1;
	}
	else {
		data = _RxBuffer.front();
		_RxBuffer.pop();
	}

	EnableIRQ();

	return data;
}

void HardwareSerial::RxReadCallback()
{
	auto handle = &ItUartHandles[_Port];

	if (_RxBufferCapacity >= 1 && (int)_RxBuffer.size() >= _RxBufferCapacity) {
		_RxBufferOverflow = true;
	}
	else {
		_RxBuffer.push(_RxByte);
	}

	HAL_UART_Receive_IT(handle, &_RxByte, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* handle)
{
	auto port = DslUartToPort(handle->Instance);
	((HardwareSerial*)DslUartInstances[port])->TxWriteCallback();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* handle)
{
	auto port = DslUartToPort(handle->Instance);
	((HardwareSerial*)DslUartInstances[port])->RxReadCallback();
}
