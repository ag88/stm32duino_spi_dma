/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@arduino.cc>
 * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _SPICLASS_H_INCLUDED
#define _SPICLASS_H_INCLUDED

#include "Arduino.h"
#include <stdio.h>
extern "C" {
#include "stm32_def.h"
#include "PeripheralPins.h"
}

// SPI_HAS_TRANSACTION means SPI has
//   - beginTransaction()
//   - endTransaction()
//   - usingInterrupt()
//   - SPISetting(clock, bitOrder, dataMode)
#define SPI_HAS_TRANSACTION 1

// Compatibility with sketches designed for AVR @ 16 MHz could not
// be ensured as SPI frequency depends of system clock configuration.
// user have to use appropriate divider for the SPI clock
// This function should not be used in new project.
// Use SPISettings with SPI.beginTransaction() to configure SPI parameters.
#define SPI_CLOCK_DIV2   2
#define SPI_CLOCK_DIV4   4
#define SPI_CLOCK_DIV8   8
#define SPI_CLOCK_DIV16  16
#define SPI_CLOCK_DIV32  32
#define SPI_CLOCK_DIV64  64
#define SPI_CLOCK_DIV128 128

#define SPI_TRANSMITRECEIVE false
#define SPI_TRANSMITONLY true

///@brief specifies the SPI speed bus in HZ.
#define SPI_SPEED_CLOCK_DEFAULT     4000000

/* this is defined in wiring_constants.h
enum BitOrder {
  LSBFIRST = 0,
  MSBFIRST = 1
};
*/

///@brief specifies the SPI mode to use
//Mode          Clock Polarity (CPOL)       Clock Phase (CPHA)
//SPI_MODE0             0                         0
//SPI_MODE1             0                         1
//SPI_MODE2             1                         0
//SPI_MODE3             1                         1
//enum definitions coming from SPI.h of SAM
// SPI mode parameters for SPISettings
typedef enum {
  SPI_MODE0 = 0,
  SPI_MODE1 = 1,
  SPI_MODE2 = 2,
  SPI_MODE3 = 3,
} SPIMode;

class SPISettings {
  public:
    constexpr SPISettings(uint32_t clock, BitOrder bitOrder, uint8_t dataMode)
      : clockFreq(clock),
        bitOrder(bitOrder),
        dataMode((SPIMode)dataMode)
    { }
    constexpr SPISettings(uint32_t clock, BitOrder bitOrder, SPIMode dataMode)
      : clockFreq(clock),
        bitOrder(bitOrder),
        dataMode(dataMode)
    { }
    constexpr SPISettings()
      : clockFreq(SPI_SPEED_CLOCK_DEFAULT),
        bitOrder(MSBFIRST),
        dataMode(SPI_MODE0)
    { }

    bool operator==(const SPISettings &rhs) const
    {
      if ((this->clockFreq == rhs.clockFreq) &&
          (this->bitOrder == rhs.bitOrder) &&
          (this->dataMode == rhs.dataMode)) {
        return true;
      }
      return false;
    }

    bool operator!=(const SPISettings &rhs) const
    {
      return !(*this == rhs);
    }

    inline uint32_t getClockFreq() {
    	return clockFreq;
    }

    inline BitOrder getBitOrder() {
    	return bitOrder;
    }

    inline SPIMode getDataMode() {
    	return dataMode;
    }

  private:
    uint32_t clockFreq; //specifies the spi bus maximum clock speed
    BitOrder bitOrder;  //bit order (MSBFirst or LSBFirst)
    SPIMode  dataMode;  //one of the data mode

    friend class SPIClass;
};

class SPIClass {
  public:
    SPIClass(uint32_t mosi = MOSI, uint32_t miso = MISO, uint32_t sclk = SCK, uint32_t ssel = PNUM_NOT_DEFINED);

    // setMISO/MOSI/SCLK/SSEL have to be called before begin()
    void setMISO(uint32_t miso)
    {
      pin_miso = digitalPinToPinName(miso);
    };
    void setMOSI(uint32_t mosi)
    {
      pin_mosi = digitalPinToPinName(mosi);
    };
    void setSCLK(uint32_t sclk)
    {
      pin_sclk = digitalPinToPinName(sclk);
    };
    void setSSEL(uint32_t ssel)
    {
      pin_ssel = digitalPinToPinName(ssel);
    };

    void setMISO(PinName miso)
    {
      pin_miso = (miso);
    };
    void setMOSI(PinName mosi)
    {
      pin_mosi = (mosi);
    };
    void setSCLK(PinName sclk)
    {
      pin_sclk = (sclk);
    };
    void setSSEL(PinName ssel)
    {
      pin_ssel = (ssel);
    };

    virtual void begin(void);
    virtual void end(void);

    /* This function should be used to configure the SPI instance in case you
     * don't use default parameters.
     */
    virtual void beginTransaction(SPISettings settings);
    virtual void endTransaction(void);

    /* Transfer functions: must be called after initialization of the SPI
     * instance with begin() or beginTransaction().
     */
    /**
      * @brief  Transfer one byte on the SPI bus.
      *         begin() or beginTransaction() must be called at least once before.
      * @param  data: byte to send.
      * @param  skipReceive: skip receiving data after transmit or not.
      *         SPI_TRANSMITRECEIVE or SPI_TRANSMITONLY.
      *         Optional, default: SPI_TRANSMITRECEIVE.
      * @return byte received from the slave.
      *
      * note this is abstract, derived class needs to implement it
      */
    virtual uint8_t transfer(uint8_t data, bool skipReceive = SPI_TRANSMITRECEIVE) = 0;

    /**
      * @brief  Transfer two bytes on the SPI bus in 16 bits format.
      *         begin() or beginTransaction() must be called at least once before.
      * @param  data: bytes to send.
      * @param  skipReceive: skip receiving data after transmit or not.
      *         SPI_TRANSMITRECEIVE or SPI_TRANSMITONLY.
      *         Optional, default: SPI_TRANSMITRECEIVE.
      * @return bytes received from the slave in 16 bits format.
      *
      * default implementation calls transfer(uint8_t data, bool skipReceive)
      */
    virtual uint16_t transfer16(uint16_t data, bool skipReceive = SPI_TRANSMITRECEIVE);


    /**
      * @brief  Transfer several bytes. Only one buffer used to send and receive data.
      *         begin() or beginTransaction() must be called at least once before.
      * @param  buf: pointer to the bytes to send. The bytes received are copy in
      *         this buffer.
      * @param  count: number of bytes to send/receive.
      * @param  skipReceive: skip receiving data after transmit or not.
      *         SPI_TRANSMITRECEIVE or SPI_TRANSMITONLY.
      *         Optional, default: SPI_TRANSMITRECEIVE.
      *
      * note this is abstract, derived class needs to implement it
      */
    virtual void transfer(void *buf, size_t count, bool skipReceive = SPI_TRANSMITRECEIVE) = 0;

    /* Expand SPI API
     * https://github.com/arduino/ArduinoCore-API/discussions/189
     *
     * @brief  Transfer several bytes. 2 buffers used to send and receive data.
     *         begin() or beginTransaction() must be called at least once before.
     * @param  tx_buf: pointer to the bytes to send.
     * @param  rx_buf: pointer to the bytes to receive.
     * @param  count: number of bytes to send/receive.
     * @param  skipReceive: skip receiving data after transmit or not.
     *         SPI_TRANSMITRECEIVE or SPI_TRANSMITONLY.
     *         Optional, default: SPI_TRANSMITRECEIVE.
     *
     * note this is abstract, derived class needs to implement it
     */
    virtual void transfer(const void *tx_buf, void *rx_buf, size_t count) = 0;

    /* These methods are deprecated and kept for compatibility.
     * Use SPISettings with SPI.beginTransaction() to configure SPI parameters.
     */
    void setBitOrder(BitOrder);
    void setDataMode(uint8_t);
    void setDataMode(SPIMode);
    void setClockDivider(uint8_t);

    // Not implemented functions. Kept for compatibility.
    void usingInterrupt(int interruptNumber);
    void notUsingInterrupt(int interruptNumber);
    void attachInterrupt(void);
    void detachInterrupt(void);

    // Could be used to mix Arduino API and STM32Cube HAL API (ex: DMA). Use at your own risk.
    SPI_HandleTypeDef *getHandle(void)
    {
      return &spihandle;
    }

  protected:
    // spi instance
    //spi_t         _spi;

    SPI_HandleTypeDef spihandle; //for HAL
    SPI_TypeDef *spi_reg; //SPI register base
    PinName pin_miso;
    PinName pin_mosi;
    PinName pin_sclk;
    PinName pin_ssel;

    /* Current SPISettings */
    SPISettings   _spiSettings = SPISettings();

    virtual void init();

    /* initSPI and getClkFreq are abstract
     * and needs to be implemented by the derived class
     */
    virtual void initSPI() = 0;

    /*
     * note that get ClkFreq() has dependency on initialization
     * the SPI instance should be initialised before calling
     */
    virtual uint32_t getClkFreq() = 0;

  private:
};

//extern SPIClass SPI;

#if defined(SUBGHZSPI_BASE)
class SUBGHZSPIClass : public SPIClass {
  public:
    SUBGHZSPIClass(): SPIClass{NC, NC, NC, NC}
    {
      _spi.spi = SUBGHZSPI;
    }

    void enableDebugPins(uint32_t mosi = DEBUG_SUBGHZSPI_MOSI, uint32_t miso = DEBUG_SUBGHZSPI_MISO, uint32_t sclk = DEBUG_SUBGHZSPI_SCLK, uint32_t ssel = DEBUG_SUBGHZSPI_SS);
};

#endif

#endif /* _SPI_H_INCLUDED */


