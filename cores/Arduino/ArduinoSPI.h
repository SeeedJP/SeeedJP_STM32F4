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

#pragma once

#include <stddef.h>
#include <stdint.h>

#define SPI_HAS_TRANSACTION 1
#define SPI_HAS_NOTUSINGINTERRUPT 0
#define SPI_ATOMIC_VERSION 0

#ifndef LSBFIRST
#define LSBFIRST 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif

#define SPI_CLOCK_DIV2 0x00
#define SPI_CLOCK_DIV4 0x08
#define SPI_CLOCK_DIV8 0x10
#define SPI_CLOCK_DIV16 0x18
#define SPI_CLOCK_DIV32 0x20
#define SPI_CLOCK_DIV64 0x28
#define SPI_CLOCK_DIV128 0x30
#define SPI_CLOCK_DIV256 0x38

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

class WioSPISettings
{
  friend class WioSPIClass;

private:
  int selectPin_;
  void* parameter_;  // SPI_HandleTypeDef*

public:
  WioSPISettings();
  WioSPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode);
  WioSPISettings(int selectPin, uint32_t clock, uint8_t bitOrder, uint8_t dataMode);
  ~WioSPISettings();

  void init(uint32_t clock, uint8_t bitOrder, uint8_t dataMode);
  void init(int selectPin, uint32_t clock, uint8_t bitOrder, uint8_t dataMode);
};

class WioSPIClass
{
private:
  void* regs_;    // SPI_TypeDef*
  const int clockPin_;
  const int mosiPin_;
  const int misoPin_;
  int beginCount_;
  bool parameterInitialized_;
  void* handle_;  // SPI_HandleTypeDef*
  int selectPin_;

  void InitializeSelectSlaveGpioPort();
  bool TryInitializeParameter(const uint32_t dataSize);
  void SelectSlaveIfRequired(const bool select);

public:
  enum WioSPIDevice
  {
    WioSPI1 = 0,
    WioSPI2,
    WioSPI3,
    WioSPI4
  };

  WioSPIClass(const enum WioSPIDevice device, const int clockPin, const int mosiPin, const int misoPin);
  ~WioSPIClass();

  void begin();
  void end();

  void beginTransaction(const WioSPISettings& settings);
  void endTransaction(void);

  uint8_t transfer(const uint8_t data);
  uint16_t transfer16(const uint16_t data);
  void transfer(void* buf, const size_t count);

  size_t transferBytes(const uint8_t* sendData, uint8_t* receivedData, const size_t count);
  size_t transferWords(const uint16_t* sendData, uint16_t* receivedData, const size_t count);

  void setBitOrder(const uint8_t bitOrder);
  void setDataMode(const uint8_t dataMode);
  void setClockDivider(const uint8_t clockDiv);
};

// Wio 3G default SPI interface (bound SPI3 - TF card interface)
extern WioSPIClass WioTFSPI;

typedef WioSPISettings SPISettings;

class SPIClass
{
public:
  static inline void begin() { WioTFSPI.begin(); }
  static inline void end() { WioTFSPI.end(); }

  static inline void usingInterrupt(uint8_t interruptNumber) { }
  static inline void notUsingInterrupt(uint8_t interruptNumber) { }

  // It signature bit changed because save from copy construct.
  static inline void beginTransaction(const SPISettings& settings) { WioTFSPI.beginTransaction(settings); }

  static inline void endTransaction(void) { WioTFSPI.endTransaction(); }

  static inline uint8_t transfer(uint8_t data) { return WioTFSPI.transfer(data); }
  static inline uint16_t transfer16(uint16_t data) { return WioTFSPI.transfer16(data); }
  static inline void transfer(void *buf, size_t count) { WioTFSPI.transfer(buf, count); }

  static inline void setBitOrder(uint8_t bitOrder) { WioTFSPI.setBitOrder(bitOrder); }
  static inline void setDataMode(uint8_t dataMode) { WioTFSPI.setDataMode(dataMode); }
  static inline void setClockDivider(uint8_t clockDiv) { WioTFSPI.setClockDivider(clockDiv); }
};

// Legacy Arduino implementation (It's statical facade)
extern SPIClass SPI;

///////////////////////////////////////
// Arduino SD library compatibility

// It's dirty hack, but hide implements behind Arduino :(

#ifndef WIO3G_EXCLUDE_ARDUINO_SD_LIBRARY_COMPATIBILITY

#define Sd2PinMap_h

extern uint8_t const SS_PIN;
extern uint8_t const MOSI_PIN;
extern uint8_t const MISO_PIN;
extern uint8_t const SCK_PIN;

#define SDCARD_SS_PIN SS_PIN
#define SDCARD_MOSI_PIN MOSI_PIN
#define SDCARD_MISO_PIN MISO_PIN
#define SDCARD_SCK_PIN SCK_PIN

#define boolean bool

#endif
