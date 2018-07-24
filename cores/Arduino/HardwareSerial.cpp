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
	_TxTimeout = HAL_MAX_DELAY;

	DslUartInstances[_Port] = this;
}

HardwareSerial::~HardwareSerial()
{
	DslUartInstances[_Port] = NULL;
}

void HardwareSerial::begin(long speed, int config)
{
	auto handle = &ItUartHandles[_Port];

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

	while (!_RxBuffer.empty()) _RxBuffer.pop();
}

int HardwareSerial::available() const
{
	return RxReadedSize();
}

int HardwareSerial::read()
{
	return RxReadByte();
}

int HardwareSerial::getReadBufferSize() const
{
	return _RxBufferCapacity;
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

unsigned long HardwareSerial::getWriteTimeout() const
{
	return _TxTimeout;
}

void HardwareSerial::setWriteTimeout(unsigned long timeout)
{
	_TxTimeout = timeout;
}

void HardwareSerial::flush()
{
	return;
}

size_t HardwareSerial::write(uint8_t val)
{
	TxWrite((const uint8_t*)&val, 1);

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

	if (_RtsPin >= 0 && _RxBufferCapacity >= 1 && (int)_RxBuffer.size() <= _RxBufferCapacity * 1 / 3) {
		digitalWrite(_RtsPin, LOW);
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

	if (_RtsPin >= 0 && _RxBufferCapacity >= 1 && (int)_RxBuffer.size() >= _RxBufferCapacity * 2 / 3) {
		digitalWrite(_RtsPin, HIGH);
	}

	HAL_UART_Receive_IT(handle, &_RxByte, 1);
}

void HardwareSerial::TxWrite(const uint8_t* data, int dataSize)
{
	auto handle = &ItUartHandles[_Port];

	HardwareSerial::HAL_UART_Transmit(handle, data, dataSize, _TxTimeout);
}

int HardwareSerial::HAL_UART_Transmit(void *huart_vp, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	auto huart = (UART_HandleTypeDef*)huart_vp;
	const uint16_t* tmp;
	uint32_t tickstart = 0U;

	/* Check that a Tx process is not already ongoing */
	if (huart->gState == HAL_UART_STATE_READY)
	{
		if ((pData == NULL) || (Size == 0))
		{
			return  HAL_ERROR;
		}

		/* Process Locked */
		//__HAL_LOCK(huart);

		huart->ErrorCode = HAL_UART_ERROR_NONE;
		huart->gState = HAL_UART_STATE_BUSY_TX;

		/* Init tickstart for timeout managment */
		tickstart = HAL_GetTick();

		huart->TxXferSize = Size;
		huart->TxXferCount = Size;
		while (huart->TxXferCount > 0U)
		{
			huart->TxXferCount--;

			while (_CtsPin >= 0 && digitalRead(_CtsPin) == HIGH)
			{
				if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
				{
					huart->gState = HAL_UART_STATE_READY;
					//__HAL_UNLOCK(huart);
					return HAL_TIMEOUT;
				}
			}

			if (huart->Init.WordLength == UART_WORDLENGTH_9B)
			{
				if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TXE, RESET, tickstart, Timeout) != HAL_OK)
				{
					return HAL_TIMEOUT;
				}
				tmp = (const uint16_t*)pData;
				huart->Instance->DR = (*tmp & (uint16_t)0x01FF);
				if (huart->Init.Parity == UART_PARITY_NONE)
				{
					pData += 2U;
				}
				else
				{
					pData += 1U;
				}
			}
			else
			{
				if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TXE, RESET, tickstart, Timeout) != HAL_OK)
				{
					return HAL_TIMEOUT;
				}
				huart->Instance->DR = (*pData++ & (uint8_t)0xFF);
			}
		}

		if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TC, RESET, tickstart, Timeout) != HAL_OK)
		{
			return HAL_TIMEOUT;
		}

		/* At end of Tx process, restore huart->gState to Ready */
		huart->gState = HAL_UART_STATE_READY;

		/* Process Unlocked */
		//__HAL_UNLOCK(huart);

		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

int HardwareSerial::UART_WaitOnFlagUntilTimeout(void *huart_vp, uint32_t Flag, int Status, uint32_t Tickstart, uint32_t Timeout)
{
	auto huart = (UART_HandleTypeDef*)huart_vp;
	/* Wait until flag is set */
	while ((__HAL_UART_GET_FLAG(huart, Flag) ? SET : RESET) == Status)
	{
		/* Check for the Timeout */
		if (Timeout != HAL_MAX_DELAY)
		{
			if ((Timeout == 0U) || ((HAL_GetTick() - Tickstart) > Timeout))
			{
				/* Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts for the interrupt process */
				CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_TXEIE));
				CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

				huart->gState = HAL_UART_STATE_READY;
				huart->RxState = HAL_UART_STATE_READY;

				/* Process Unlocked */
				//__HAL_UNLOCK(huart);

				return HAL_TIMEOUT;
			}
		}
	}

	return HAL_OK;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* handle)
{
	auto port = DslUartToPort(handle->Instance);
	((HardwareSerial*)DslUartInstances[port])->RxReadCallback();
}
