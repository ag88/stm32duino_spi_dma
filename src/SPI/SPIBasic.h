/*
 * SPIBasic.h
 *
 *  Created on: Jan 9, 2025
 *      Author: andrew
 *
 *  Credit for various codes here, and the original SPIClass implementation goes to
 *  Frederic Pillon (fpiSTM) and others who created the Arduino_Core_STM32 stm32duino core
 *
 *  https://github.com/stm32duino/Arduino_Core_STM32
 *
 *  Some of the SPIClass methods are overridden here.
 *  e.g. those init() and transfer() methods
 *
 */

#ifndef SPI_SPIBasic_H_
#define SPI_SPIBasic_H_

#include "SPIClass.h"


class SPIBasic: public SPIClass {
public:
	SPIBasic(uint32_t mosi = MOSI, uint32_t miso = MISO, uint32_t sclk = SCK, uint32_t ssel = PNUM_NOT_DEFINED)
		: SPIClass(mosi, miso, sclk, ssel) {};

	virtual void begin() override;

	/* defined in SPIClass
	virtual void end();

	virtual void beginTransaction(SPISettings settings);
	virtual void endTransaction();
	*/

	virtual uint8_t transfer(uint8_t data, bool skipReceive = SPI_TRANSMITRECEIVE) override;

	/* defined and implemented in SPIClass
	virtual uint16_t transfer16(uint16_t data, bool skipReceive = SPI_TRANSMITRECEIVE);
	*/

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
      * implemented in this class
      */
	virtual void transfer(void *buf, size_t count, bool skipReceive = SPI_TRANSMITRECEIVE) override;

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
     * implemented in this class
     */
	virtual void transfer(const void *tx_buf, void *rx_buf, size_t count) override;

	/* this is the actual implementation of the prior with a skipreceive flag
	 * in that mode rx_buf is left untouched */
	virtual void transfer(const uint8_t *tx_buf, uint8_t *rx_buf, size_t count, bool skipReceive);

	virtual ~SPIBasic();

protected:
	//SPI handle from base class
    //SPI_HandleTypeDef* hspi;

    DMA_HandleTypeDef hdma_tx;
    DMA_HandleTypeDef hdma_rx;

    /* Current SPISettings */
    SPISettings   _spiSettings = SPISettings();

	/*
	 *  this is called by begin() which in turns call
	 * 		initPins();
	 *  	initSPI();
	 */
	virtual void init() override;


	/* initialize SPI Pins called by init()*/
	virtual void initPins();

	/* this implementation is generic, enable SPI clock
	 * and call SPIBasic::initSPIDefault(SPI_TypeDef *spi_reg) with the correct
	 * SPI_REGISTER_BASE (e.g. SPI1) to initialize SPI
	 */
	virtual void initSPI() override;
	virtual void initSPIDefault(SPI_TypeDef *spireg);

	/*
	 * returns SPI_BASE address from pins
	 */
	virtual SPI_TypeDef* getSPIBASE();

	/*
	 * enable SPI clocks, has dependency on spi_reg
	 */
	virtual void enableSPICLK(SPI_TypeDef *spireg);


	virtual uint32_t getClkFreq() override;


private:

};

extern SPIBasic SPI;

#endif /* SPI_SPIBasic_H_ */
