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

#include "SPIClass.h"

//SPIClass SPI;

/**
 * @brief  Default Constructor. Uses pin configuration of default SPI
 *         defined in the variant*.h.
 *         To create another SPI instance attached to another SPI
 *         peripheral gave the pins as parameters to the constructor.
 * @note   All pins must be attached to the same SPI peripheral.
 *         See datasheet of the microcontroller.
 * @param  mosi: SPI mosi pin. Accepted format: number or Arduino format (Dx)
 *         or ST format (Pxy). Default is MOSI pin of the default SPI peripheral.
 * @param  miso: SPI miso pin. Accepted format: number or Arduino format (Dx)
 *         or ST format (Pxy). Default is MISO pin of the default SPI peripheral.
 * @param  sclk: SPI clock pin. Accepted format: number or Arduino format (Dx)
 *         or ST format (Pxy). Default is SCK pin of the default SPI peripheral.
 * @param  ssel: SPI ssel pin (optional). Accepted format: number or
 *         Arduino format (Dx) or ST format (Pxy). By default is set to NC.
 *         This pin must correspond to a hardware CS pin which can be managed
 *         by the SPI peripheral itself. See the datasheet of the microcontroller
 *         or look at PinMap_SPI_SSEL[] inside the file PeripheralPins.c
 *         corresponding to the board. If you configure this pin you can't use
 *         another CS pin and don't pass a CS pin as parameter to any functions
 *         of the class.
 */
SPIClass::SPIClass(uint32_t mosi, uint32_t miso, uint32_t sclk, uint32_t ssel) {
	//memset((void *)&_spi, 0, sizeof(_spi));
	memset((void*) &spihandle, 0, sizeof(spihandle));
	spi_reg = NULL;

	pin_miso = digitalPinToPinName(miso);
	pin_mosi = digitalPinToPinName(mosi);
	pin_sclk = digitalPinToPinName(sclk);
	pin_ssel = digitalPinToPinName(ssel);
}

/**
 * @brief  Initialize the SPI instance.
 */
void SPIClass::begin(void) {
	spihandle.State = HAL_SPI_STATE_RESET;
	_spiSettings = SPISettings();
	init();
}

/* spi initializations */
void SPIClass::init() {

	initSPI();
	/*
	 spi_init(&_spi, _spiSettings.clockFreq,
	 _spiSettings.dataMode,
	 _spiSettings.bitOrder);
	 */
}

/**
 * @brief  Deinitialize the SPI instance and stop it.
 */
void SPIClass::end(void) {
	//spi_deinit(&_spi);
}

/**
 * @brief  This function should be used to configure the SPI instance in case you
 *         don't use the default parameters set by the begin() function.
 * @param  settings: SPI settings(clock speed, bit order, data mode).
 */
void SPIClass::beginTransaction(SPISettings settings) {
	if (_spiSettings != settings) {
		_spiSettings = settings;

		initSPI();
		/*
		 spi_init(&_spi, _spiSettings.clockFreq,
		 _spiSettings.dataMode,
		 _spiSettings.bitOrder);
		 */
	}
}

/**
 * @brief  End the transaction after beginTransaction usage
 */
void SPIClass::endTransaction(void) {

}

/**
 * @brief  Deprecated function.
 *         Configure the bit order: MSB first or LSB first.
 * @param  bitOrder: MSBFIRST or LSBFIRST
 */
void SPIClass::setBitOrder(BitOrder bitOrder) {
	_spiSettings.bitOrder = bitOrder;

	initSPI();
	/*
	 spi_init(&_spi, _spiSettings.clockFreq,
	 _spiSettings.dataMode,
	 _spiSettings.bitOrder);
	 */
}

/**
 * @brief  Deprecated function.
 *         Configure the data mode (clock polarity and clock phase)
 * @param  mode: SPI_MODE0, SPI_MODE1, SPI_MODE2 or SPI_MODE3
 * @note
 *         Mode          Clock Polarity (CPOL)   Clock Phase (CPHA)
 *         SPI_MODE0             0                     0
 *         SPI_MODE1             0                     1
 *         SPI_MODE2             1                     0
 *         SPI_MODE3             1                     1
 */
void SPIClass::setDataMode(uint8_t mode) {
	setDataMode((SPIMode) mode);
}

void SPIClass::setDataMode(SPIMode mode) {
	_spiSettings.dataMode = mode;
	initSPI();
	/*
	 spi_init(&_spi, _spiSettings.clockFreq,
	 _spiSettings.dataMode,
	 _spiSettings.bitOrder);
	 */
}

/**
 * @brief  Deprecated function.
 *         Configure the clock speed
 * @param  divider: the SPI clock can be divided by values from 1 to 255.
 *         If 0, default SPI speed is used.
 */
void SPIClass::setClockDivider(uint8_t divider) {
	if (divider == 0) {
		_spiSettings.clockFreq = SPI_SPEED_CLOCK_DEFAULT;
	} else {
		/* Get clk freq of the SPI instance and compute it */
		//_spiSettings.clockFreq = getClkFreq(spi_reg) / divider;
		_spiSettings.clockFreq = getClkFreq() / divider;
	}
	initSPI();
	/*
	 spi_init(&_spi, _spiSettings.clockFreq,
	 _spiSettings.dataMode,
	 _spiSettings.bitOrder);
	 */
}

/**
 * @brief  Transfer two bytes on the SPI bus in 16 bits format.
 *         begin() or beginTransaction() must be called at least once before.
 * @param  data: bytes to send.
 * @param  skipReceive: skip receiving data after transmit or not.
 *         SPI_TRANSMITRECEIVE or SPI_TRANSMITONLY.
 *         Optional, default: SPI_TRANSMITRECEIVE.
 * @return bytes received from the slave in 16 bits format.
 */
uint16_t SPIClass::transfer16(uint16_t data, bool skipReceive) {
	uint16_t tmp, r = 0;

	if (_spiSettings.bitOrder) {
		tmp = ((data & 0xff00) >> 8) | ((data & 0xff) << 8);
		data = tmp;
	}

	r = transfer(data & 0xffUL, skipReceive);
	r |= transfer((data >> 8) & 0xffUL, skipReceive) << 8;

	if (_spiSettings.bitOrder) {
		tmp = ((data & 0xff00) >> 8) | ((data & 0xff) << 8);
		data = tmp;
	}

	return data;
}

/**
 * @brief  Not implemented.
 */
void SPIClass::usingInterrupt(int interruptNumber) {
	UNUSED(interruptNumber);
}

/**
 * @brief  Not implemented.
 */
void SPIClass::notUsingInterrupt(int interruptNumber) {
	UNUSED(interruptNumber);
}

/**
 * @brief  Not implemented.
 */
void SPIClass::attachInterrupt(void) {
	// Should be enableInterrupt()
}

/**
 * @brief  Not implemented.
 */
void SPIClass::detachInterrupt(void) {
	// Should be disableInterrupt()
}

#if defined(SUBGHZSPI_BASE)
void SUBGHZSPIClass::enableDebugPins(uint32_t mosi, uint32_t miso, uint32_t sclk, uint32_t ssel)
{
  /* Configure SPI GPIO pins */
  pinmap_pinout(digitalPinToPinName(mosi), PinMap_SPI_MOSI);
  pinmap_pinout(digitalPinToPinName(miso), PinMap_SPI_MISO);
  pinmap_pinout(digitalPinToPinName(sclk), PinMap_SPI_SCLK);
  pinmap_pinout(digitalPinToPinName(ssel), PinMap_SPI_SSEL);
}
#endif
