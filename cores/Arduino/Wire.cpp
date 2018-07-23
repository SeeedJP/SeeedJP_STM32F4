#include "Arduino.h"
#include "Wire.h"
#include <stm32f4xx_hal.h>
#include <DeviceSupportLibrary.h>

#define TRANSMIT_TIMEOUT	(1000)
#define RECEIVE_TIMEOUT		(10000)

TwoWire::TwoWire(int port, int sdaPin, int sclPin)
{
	_Port = port;
	_SdaPin = sdaPin;
	_SclPin = sclPin;

	_Handle = new I2C_HandleTypeDef;
}

TwoWire::~TwoWire()
{
	delete (I2C_HandleTypeDef*)_Handle;
}

void TwoWire::begin()
{
	I2C_HandleTypeDef* handle = (I2C_HandleTypeDef*)_Handle;

	DslGpioClockEnable(DslGpioRegs[_SdaPin / 16]);
	DslGpioClockEnable(DslGpioRegs[_SclPin / 16]);

	GPIO_InitTypeDef  gpioInit;
	gpioInit.Mode = GPIO_MODE_AF_OD;
	gpioInit.Pull = GPIO_PULLUP;
	gpioInit.Speed = GPIO_SPEED_FAST;

	gpioInit.Pin = DslGpioPins[_SdaPin % 16];
	gpioInit.Alternate = DslI2cGpioAlternate(DslI2cRegs[_Port], _SdaPin);
	HAL_GPIO_Init(DslGpioRegs[_SdaPin / 16], &gpioInit);

	gpioInit.Pin = DslGpioPins[_SclPin % 16];
	gpioInit.Alternate = DslI2cGpioAlternate(DslI2cRegs[_Port], _SclPin);
	HAL_GPIO_Init(DslGpioRegs[_SclPin / 16], &gpioInit);

	DslI2cClockEnable(DslI2cRegs[_Port]);

	handle->Instance = DslI2cRegs[_Port];
	handle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	handle->Init.ClockSpeed = 400000;
	handle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	handle->Init.DutyCycle = I2C_DUTYCYCLE_2;
	handle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	handle->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	handle->Init.OwnAddress1 = 0;
	handle->Init.OwnAddress2 = 0;
	if (HAL_I2C_Init(handle) != HAL_OK) return;
}

void TwoWire::beginTransmission(int address)
{
	_Address = address << 1;
	_Buffer.clear();
}

void TwoWire::write(uint8_t data)
{
	_Buffer.push_back(data);
}

uint8_t TwoWire::endTransmission()
{
	I2C_HandleTypeDef* handle = (I2C_HandleTypeDef*)_Handle;

	if (HAL_I2C_Master_Transmit(handle, (uint16_t)_Address, &_Buffer[0], _Buffer.size(), TRANSMIT_TIMEOUT) != HAL_OK) {
		if (HAL_I2C_GetError(handle) == HAL_I2C_ERROR_AF) {
			return 2;	// received NACK on transmit of address
		}
		else {
			return 4;	// other error
		}
	}

	return 0;	// success
}

uint8_t TwoWire::requestFrom(int address, int quantity)
{
	I2C_HandleTypeDef* handle = (I2C_HandleTypeDef*)_Handle;

	_Buffer.resize(quantity);
	if (HAL_I2C_Master_Receive(handle, (uint16_t)_Address, &_Buffer[0], _Buffer.size(), RECEIVE_TIMEOUT) != HAL_OK) {
		_Buffer.clear();
		return 0;
	}

	return _Buffer.size();
}

int TwoWire::available() const
{
	return _Buffer.size();
}

int TwoWire::read()
{
	uint8_t val = _Buffer[0];
	_Buffer.erase(_Buffer.begin());

	return val;
}
