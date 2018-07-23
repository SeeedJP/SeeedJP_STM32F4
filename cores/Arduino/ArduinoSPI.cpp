////////////////////////////////////////////////////////////////////////// 
// 
// An SPI driver implementation for Arduino IDE platform using SeeedJP Wio 3G.
// Copyright (c) 2018 Kouji Matsui (@kekyo2, @kozy_kekyo)
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files
// (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge,
// publish, distribute, sublicense, and/or sell copies of the Software,
// and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// 
////////////////////////////////////////////////////////////////////////// 

#include "Arduino.h"
#include <stm32f4xx_hal.h>
#include <DeviceSupportLibrary.h>

#include <string.h>
#include <alloca.h>

////////////////////////////////////////////////////////////////////////////////////////

#define PINNAME_TO_PIN(port, pin) ((port - 'A') * 16 + pin)

#define NSS_SPI1 PINNAME_TO_PIN('A', 4)
#define SCK_SPI1 PINNAME_TO_PIN('A', 5)
#define MISO_SPI1 PINNAME_TO_PIN('A', 6)
#define MOSI_SPI1 PINNAME_TO_PIN('A', 7)

#define NSS_SPI3 PINNAME_TO_PIN('D', 0)     // (Not hardware NSS)
#define SCK_SPI3 PINNAME_TO_PIN('C', 10)
#define MISO_SPI3 PINNAME_TO_PIN('C', 11)
#define MOSI_SPI3 PINNAME_TO_PIN('C', 12)

// Arduino compatibility - SPI3 (TF card interface)
const uint8_t WIO_TF_SS_PIN = NSS_SPI3;
const uint8_t WIO_TF_MOSI_PIN = MOSI_SPI3;
const uint8_t WIO_TF_MISO_PIN = MISO_SPI3;
const uint8_t WIO_TF_SCK_PIN = SCK_SPI3;

////////////////////////////////////////////////////////////////////////////////////////

// Wio 3G default SPI interface (bound SPI1)
WioSPIClass WioSPI(WioSPIClass::WioSPI1, SCK_SPI1, MOSI_SPI1, MISO_SPI1);

// Wio 3G default SPI interface (bound SPI3)
WioSPIClass WioTFSPI(WioSPIClass::WioSPI3, SCK_SPI3, MOSI_SPI3, MISO_SPI3);

// Legacy Arduino implementation (It's static facade)
const SPIClass SPI;

// Default SPI setting instance.
static WioSPISettings g_DefaultSPISettings;

////////////////////////////////////////////////////////////////////////////////////////

#define getSelectPin(t) ((t).selectPin_)
#define getParameter(t) (static_cast<SPI_InitTypeDef*>(const_cast<void*>((t).parameter_)))

WioSPISettings::WioSPISettings()
    : parameter_(new SPI_InitTypeDef)
{
    // Defaulted: SPI1, 4MHz, MSB, MODE0
    init(NSS_SPI1, 4000000, MSBFIRST, SPI_MODE0);
}

WioSPISettings::WioSPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
    : parameter_(new SPI_InitTypeDef)
{
    init(-1, clock, bitOrder, dataMode);
}

WioSPISettings::WioSPISettings(int selectPin, uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
    : parameter_(new SPI_InitTypeDef)
{
    init(selectPin, clock, bitOrder, dataMode);
}

WioSPISettings::~WioSPISettings()
{
    delete getParameter(*this);
}

void WioSPISettings::init(uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
{
    init(-1, clock, bitOrder, dataMode);
}

void WioSPISettings::init(int selectPin, uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
{
    selectPin_ = selectPin;

    auto parameter = getParameter(*this);

    parameter->Mode = SPI_MODE_MASTER;

    parameter->Direction = SPI_DIRECTION_2LINES;
    parameter->DataSize = SPI_DATASIZE_8BIT;
    parameter->NSS = SPI_NSS_SOFT;

    parameter->TIMode = SPI_TIMODE_DISABLE;
    parameter->CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    parameter->CRCPolynomial = 7;

    parameter->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;   // TODO:
    parameter->FirstBit = (bitOrder == LSBFIRST) ? SPI_FIRSTBIT_LSB : SPI_FIRSTBIT_MSB;
    parameter->CLKPolarity = ((dataMode == SPI_MODE2) || (dataMode == SPI_MODE3)) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
    parameter->CLKPhase = ((dataMode == SPI_MODE1) || (dataMode == SPI_MODE3)) ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
}

////////////////////////////////////////////////////////////////////////////////////////

#define getRegs(t) (static_cast<SPI_TypeDef*>((t).regs_))
#define getHandle(t) (static_cast<SPI_HandleTypeDef*>((t).handle_))

#define getSelectPinOrDefaulted(t) ((((t).selectPin_) == -1) ? ((t).selectPin_) : NSS_SPI1)

WioSPIClass::WioSPIClass(const enum WioSPIClass::WioSPIDevice device, const int clockPin, const int mosiPin, const int misoPin)
    : regs_(DslSpiRegs[static_cast<int>(device)])
    , clockPin_(clockPin)
    , mosiPin_(mosiPin)
    , misoPin_(misoPin)
    , beginCount_(0)
    , parameterInitialized_(false)
    , handle_(nullptr)
    , selectPin_(-1)
{
}

WioSPIClass::~WioSPIClass()
{
    end();
}

static void InitializeSPIRelatedGpioPort(SPI_TypeDef* regs, const int pin)
{
    auto gpioRegs = DslGpioRegs[pin / 16];
    const auto rawGpioPin = DslGpioPins[pin % 16];

	DslGpioClockEnable(gpioRegs);
    
    GPIO_InitTypeDef gpioInit;

    gpioInit.Pin = rawGpioPin;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Alternate = DslSpiGpioAlternate(regs, pin);

    HAL_GPIO_Init(gpioRegs, &gpioInit);
}

void WioSPIClass::begin()
{
    beginCount_++;
    if (beginCount_ == 1)
    {
        handle_ = new SPI_HandleTypeDef;
        memset(handle_, 0, sizeof(SPI_HandleTypeDef));
        selectPin_ = -1;

        auto handle = getHandle(*this);
        handle->Init = *getParameter(g_DefaultSPISettings);

        auto regs = getRegs(*this);
        InitializeSPIRelatedGpioPort(regs, clockPin_);
        InitializeSPIRelatedGpioPort(regs, mosiPin_);
        InitializeSPIRelatedGpioPort(regs, misoPin_);

        DslSpiClockEnable(regs);

        parameterInitialized_ = false;
    }
}

void WioSPIClass::end()
{
    if (beginCount_ >= 1)
    {
        beginCount_--;
        if (beginCount_ == 0)
        {
            // TODO: DslSpiClockDisable(getRegs(*this));
            // TODO: DslGpioClockDisable(DslGpioRegs[pin / 16]);

            delete getHandle(*this);

            parameterInitialized_ = false;
        }
    }
}

void WioSPIClass::InitializeSelectSlaveGpioPort()
{
    auto regs = getRegs(*this);
    auto handle = getHandle(*this);

    const auto selectPin = getSelectPinOrDefaulted(*this);
    const auto autoNss = DslSpiNssGpio(regs, selectPin);

    handle->Init.NSS = autoNss ? SPI_NSS_HARD_OUTPUT : SPI_NSS_SOFT;
    if (handle->Init.NSS == SPI_NSS_SOFT)
    {
        InitializeSPIRelatedGpioPort(regs, selectPin);
    }
}

void WioSPIClass::beginTransaction(const WioSPISettings& settings)
{
    auto regs = getRegs(*this);
    auto handle = getHandle(*this);

    handle->Instance = regs;
    handle->Init = *getParameter(settings);

    selectPin_ = getSelectPin(settings);

    InitializeSelectSlaveGpioPort();

    parameterInitialized_ = false;
}

void WioSPIClass::endTransaction(void)
{
    auto handle = getHandle(*this);

    HAL_SPI_DeInit(handle);

    if (handle->Init.NSS == SPI_NSS_SOFT)
    {
        // TODO: UninitializeSPIRelatedGpioPort(regs, getSelectPinOrDefaulted(*this));
    }

    parameterInitialized_ = false;
}

bool WioSPIClass::TryInitializeParameter(const uint32_t dataSize)
{
    auto handle = getHandle(*this);

    if ((parameterInitialized_ == false) || (handle->Init.DataSize != dataSize))
    {
        if (parameterInitialized_ == true)
        {
            if (HAL_SPI_DeInit(handle) != HAL_OK)
            {
                return false;
            }

            parameterInitialized_ = false;
        }

        handle->Init.DataSize = dataSize;

        if (HAL_SPI_Init(handle) != HAL_OK)
        {
            return false;
        }

        parameterInitialized_ = true;
    }

    return true;
}

void WioSPIClass::SelectSlaveIfRequired(const bool select)
{
    auto handle = getHandle(*this);

    if (handle->Init.NSS == SPI_NSS_SOFT)
    {
        auto selectPin = getSelectPinOrDefaulted(*this);
        auto selectRegs = DslGpioRegs[selectPin / 16];
        const auto rawSelectPin = DslGpioPins[selectPin % 16];
        HAL_GPIO_WritePin(selectRegs, rawSelectPin, select ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

uint8_t WioSPIClass::transfer(const uint8_t data)
{
    if (TryInitializeParameter(SPI_DATASIZE_8BIT) == false)
    {
        return 0;
    }

    SelectSlaveIfRequired(true);

    auto handle = getHandle(*this);
    uint8_t receivedData = 0;
    if (HAL_SPI_TransmitReceive(
        handle,
        const_cast<uint8_t*>(&data),
        &receivedData,
        sizeof receivedData,
        10000) != HAL_OK)
    {
        receivedData = 0;
    }

    SelectSlaveIfRequired(false);

    return receivedData;
}

uint16_t WioSPIClass::transfer16(const uint16_t data)
{
    if (TryInitializeParameter(SPI_DATASIZE_16BIT) == false)
    {
        return 0;
    }

    SelectSlaveIfRequired(true);

    auto handle = getHandle(*this);
    uint16_t receivedData = 0;
    if (HAL_SPI_TransmitReceive(
        handle,
        reinterpret_cast<uint8_t*>(const_cast<uint16_t*>(&data)),
        reinterpret_cast<uint8_t*>(&receivedData),
        sizeof receivedData,
        10000) != HAL_OK)
    {
        receivedData = 0;
    }

    SelectSlaveIfRequired(false);

    return receivedData;
}

void WioSPIClass::transfer(void *buf, const size_t count)
{
    // Use temporary buffer, we have to avoid race condition at full duplex transfer.
    auto receivedData = static_cast<uint8_t*>(alloca(count));
    if (transferBytes(static_cast<const uint8_t*>(buf), receivedData, count) == count)
    {
        memcpy(buf, receivedData, count);
    }
    else
    {
        memset(buf, 0, count);
    }
}

size_t WioSPIClass::transferBytes(const uint8_t* sendData, uint8_t* receivedData, const size_t count)
{
    if (TryInitializeParameter(SPI_DATASIZE_8BIT) == false)
    {
        return 0;
    }

    SelectSlaveIfRequired(true);

    auto handle = getHandle(*this);
    size_t received;
    if (HAL_SPI_TransmitReceive(
        handle,
        const_cast<uint8_t*>(sendData),
        receivedData,
        count,
        10000) != HAL_OK)
    {
        received = 0;
    }
    else
    {
        received = count;
    }

    SelectSlaveIfRequired(false);

    return received;
}

size_t WioSPIClass::transferWords(const uint16_t* sendData, uint16_t* receivedData, const size_t count)
{
    if (TryInitializeParameter(SPI_DATASIZE_16BIT) == false)
    {
        return 0;
    }

    SelectSlaveIfRequired(true);

    auto handle = getHandle(*this);
    size_t received;
    if (HAL_SPI_TransmitReceive(
        handle,
        reinterpret_cast<uint8_t*>(const_cast<uint16_t*>(sendData)),
        reinterpret_cast<uint8_t*>(receivedData),
        count,
        10000) != HAL_OK)
    {
        received = 0;
    }
    else
    {
        received = count;
    }

    SelectSlaveIfRequired(false);

    return received;
}

void WioSPIClass::setBitOrder(const uint8_t bitOrder)
{
    auto handle = getHandle(*this);

    handle->Init.FirstBit = (bitOrder == LSBFIRST) ? SPI_FIRSTBIT_LSB : SPI_FIRSTBIT_MSB;
    parameterInitialized_ = false;
}

void WioSPIClass::setDataMode(const uint8_t dataMode)
{
    auto handle = getHandle(*this);

    handle->Init.CLKPolarity = ((dataMode == SPI_MODE2) || (dataMode == SPI_MODE3)) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
    handle->Init.CLKPhase = ((dataMode == SPI_MODE1) || (dataMode == SPI_MODE3)) ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
    parameterInitialized_ = false;
}

void WioSPIClass::setClockDivider(const uint8_t clockDiv)
{
    auto handle = getHandle(*this);

    switch (clockDiv)
    {
        case SPI_CLOCK_DIV2:
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
            break;
        case SPI_CLOCK_DIV4:
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
            break;
        case SPI_CLOCK_DIV8:
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
            break;
        case SPI_CLOCK_DIV16:
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
            break;
        case SPI_CLOCK_DIV32:
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
            break;
        case SPI_CLOCK_DIV64:
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
            break;
        case SPI_CLOCK_DIV128:
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
            break;
        default:
            return;
    }

    parameterInitialized_ = false;
}
