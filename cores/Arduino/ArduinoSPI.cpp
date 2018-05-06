////////////////////////////////////////////////////////////////////////// 
// 
// An SPI driver implementation for Arduino IDE platform using SeeedJP Wio 3G.
// Copyright (c) 2018 Kouji Matsui (@kozy_kekyo) 
//
// TBD: LICENSE
// 
////////////////////////////////////////////////////////////////////////// 

#include "Arduino.h"
#include <stm32f4xx_hal.h>
#include <DeviceSupportLibrary.h>

////////////////////////////////////////////////////////////////////////////////////////

// Wio 3G default SPI interface (bound SPI3 - TF card interface)
WioSPIClass WioSPI3(WioSPIClass::SPI3, );

// Legacy Arduino implementation (It's statical facade)
SPIClass SPI;

////////////////////////////////////////////////////////////////////////////////////////

#define getSelectPin(t) ((t).selectPin_)
#define getParameter(t) (static_cast<SPI_InitTypeDef*>(const_cast<void*>((t).parameter_)))

WioSPISettings::WioSPISettings()
    : parameter_(new SPI_InitTypeDef)
{
    init(4000000, MSBFIRST, SPI_MODE0);
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
    parameter->NSSPMode = SPI_NSS_PULSE_DISABLE;

    parameter->TIMode = SPI_TIMODE_DISABLE;
    parameter->CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    parameter->CRCLength = SPI_CRC_LENGTH_DATASIZE;
    parameter->CRCPolynomial = 7;

    parameter->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;   // TODO:
    parameter->FirstBit = (bitOrder == LSBFIRST) ? SPI_FIRSTBIT_LSB : SPI_FIRSTBIT_MSB;
    parameter->CLKPolarity = ((dataMode == SPI_MODE2) || (dataMode == SPI_MODE3)) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
    parameter->CLKPhase = ((dataMode == SPI_MODE1) || (dataMode == SPI_MODE3)) ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
}

////////////////////////////////////////////////////////////////////////////////////////

static WioSPISettings g_DefaultSPISettings;

#define getRegs(t) (static_cast<SPI_TypeDef*>((t).regs_))
#define getHandle(t) (static_cast<SPI_HandleTypeDef*>((t).handle_))

#define getSelectPinOrDefaulted(t) ((((t).selectPin_) == -1) ? ((t).selectPin_) : getSelectPin(g_DefaultSPISettings))

WioSPIClass::WioSPIClass(const enum WioSPIClass::WioSPIDevice device, const int clockPin, const int mosiPin, const int misoPin)
    : regs_(DslSpiRegs[static_cast<int>(device)])
    , clockPin_(clockPin)
    , mosiPin_(mosiPin)
    , misoPin_(misoPin)
    , beginCount_(0)
    , initialized_(false)
    , handle_(nullptr)
    , selectPin_(0)
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
        selectPin_ = selectPin;

        auto handle = getHandle(*this);
        handle->Init = *getParameter(g_DefaultSPISettings);

        auto regs = getRegs(*this);
        InitializeSPIRelatedGpioPort(regs, clockPin_);
        InitializeSPIRelatedGpioPort(regs, mosiPin_);
        InitializeSPIRelatedGpioPort(regs, misoPin_);

        DslSpiClockEnable(regs);

        initialized_ = false;
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

            initialized_ = false;
        }
    }
}

void WioSPIClass::InitializeSelectSlaveGpioPort()
{
    auto regs = getRegs(*this);
    const auto selectPin = getSelectPinOrDefaulted(*this);
    const auto autoNss = DslSpiNssGpio(regs, selectPin);

    handle->Init.NSS = autoNss ? SPI_NSS_HARD_OUTPUT : SPI_NSS_SOFT;
    handle->Init.NSSPMode = autoNss ? SPI_NSS_PULSE_ENABLED : SPI_NSS_PULSE_DISABLED;

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

    initialized_ = false;
}

void WioSPIClass::endTransaction(void)
{
    auto handle = getHandle(*this);

    HAL_SPI_DeInit(handle);

    if (handle->Init.NSS == SPI_NSS_SOFT)
    {
        // TODO: UninitializeSPIRelatedGpioPort(regs, getSelectPinOrDefaulted(*this));
    }

    initialized_ = false;
}

bool WioSPIClass::InitializeSPIHalIfRequired(const uint32_t dataSize)
{
    if ((initialized_ == false) || (handle->Init.DataSize != dataSize))
    {
        if (initialized_ == true)
        {
            if (HAL_SPI_DeInit(handle) != HAL_OK)
            {
                return false;
            }

            initialized_ = false;
        }

        handle->Init.DataSize = dataSize;

        if (HAL_SPI_Init(handle) != HAL_OK)
        {
            return false;
        }

        initialized_ = true;
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
        HAL_GPIO_WritePin(selectRegs, rawSelectPin, GPIO_PIN_RESET);
    }
}

uint8_t WioSPIClass::transfer(const uint8_t data)
{
    if (InitializeSPIHalIfRequired(SPI_DATASIZE_8BIT) == false)
    {
        return 0;
    }

    SelectSlaveIfRequired(true);

    auto handle = getHandle(*this);
    uint8_t received = 0;
    if (HAL_SPI_TransmitReceive(handle, &data, &received, sizeof received, 10000) != HAL_OK)
    {
        received = 0;
    }

    SelectSlaveIfRequired(false);

    return received;
}

uint16_t WioSPIClass::transfer16(const uint16_t data)
{
    if (InitializeSPIHalIfRequired(SPI_DATASIZE_16BIT) == false)
    {
        return 0;
    }

    SelectSlaveIfRequired(true);

    auto handle = getHandle(*this);
    uint16_t received = 0;
    if (HAL_SPI_TransmitReceive(handle, &data, &received, sizeof received, 10000) != HAL_OK)
    {
        received = 0;
    }

    SelectSlaveIfRequired(false);

    return received;
}

void WioSPIClass::transfer(void *buf, const size_t count)
{
    if (count == 0) return;
    uint8_t *p = (uint8_t *)buf;
    SPDR = *p;
    while (--count > 0) {
      uint8_t out = *(p + 1);
      while (!(SPSR & _BV(SPIF))) ;
      uint8_t in = SPDR;
      SPDR = out;
      *p++ = in;
    }
    while (!(SPSR & _BV(SPIF))) ;
    *p = SPDR;
}

// This function is deprecated.  New applications should use
// beginTransaction() to configure SPI settings.
void WioSPIClass::setBitOrder(const uint8_t bitOrder)
{
    auto handle = getHandle(*this);

    handle->Init.FirstBit = (bitOrder == LSBFIRST) ? SPI_FIRSTBIT_LSB : SPI_FIRSTBIT_MSB;
    requireInitialize_ = true;
}

// This function is deprecated.  New applications should use
// beginTransaction() to configure SPI settings.
void WioSPIClass::setDataMode(const uint8_t dataMode)
{
    SPCR = (SPCR & ~SPI_MODE_MASK) | dataMode;
}

// This function is deprecated.  New applications should use
// beginTransaction() to configure SPI settings.
void WioSPIClass::setClockDivider(const uint8_t clockDiv)
{
    SPCR = (SPCR & ~SPI_CLOCK_MASK) | (clockDiv & SPI_CLOCK_MASK);
    SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((clockDiv >> 2) & SPI_2XCLOCK_MASK);
}

// These undocumented functions should not be used.  SPI.transfer()
// polls the hardware flag which is automatically cleared as the
// AVR responds to SPI's interrupt
void WioSPIClass::attachInterrupt()
{
    SPCR |= _BV(SPIE);
}

void WioSPIClass::detachInterrupt()
{
    SPCR &= ~_BV(SPIE);
}
