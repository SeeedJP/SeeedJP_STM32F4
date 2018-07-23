#pragma once

#include <stm32f4xx_hal.h>

////////////////////////////////////////
// GPIO

extern GPIO_TypeDef* const DslGpioRegs[];
extern const uint16_t DslGpioPins[];

void DslGpioClockEnable(GPIO_TypeDef* reg);

////////////////////////////////////////
// ADC

extern ADC_TypeDef* const DslAdcRegs[];
extern const uint32_t DslAdcChannels[];

void DslAdcClockEnable(ADC_TypeDef* reg);
uint32_t DslAdcChannel(ADC_TypeDef* reg, int pin);

////////////////////////////////////////
// DAC

extern DAC_TypeDef* const DslDacRegs[];
extern const uint32_t DslDacChannels[];

void DslDacClockEnable(DAC_TypeDef* reg);
uint32_t DslDacChannel(DAC_TypeDef* reg, int pin);

////////////////////////////////////////
// UART

extern USART_TypeDef* const DslUartRegs[];
extern const IRQn_Type DslUartIRQs[];
extern void* DslUartInstances[];	// HardwareSerial* []

int DslUartToPort(USART_TypeDef* reg);
void DslUartClockEnable(USART_TypeDef* reg);
uint32_t DslUartGpioAlternate(USART_TypeDef* reg, int pin);

////////////////////////////////////////
// I2C

extern I2C_TypeDef* const DslI2cRegs[];

void DslI2cClockEnable(I2C_TypeDef* reg);
uint32_t DslI2cGpioAlternate(I2C_TypeDef* reg, int pin);

////////////////////////////////////////
// Interrupt

void DslInterruptExtiEnable(int num);

////////////////////////////////////////
// SPI

extern SPI_TypeDef* const DslSpiRegs[];

void DslSpiClockEnable(SPI_TypeDef* reg);
bool DslSpiNssGpio(SPI_TypeDef* reg, const int pin);
uint32_t DslSpiGpioAlternate(SPI_TypeDef* reg, const int pin);
