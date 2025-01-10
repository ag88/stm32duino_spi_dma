/*
 * DMASPI.h
 *
 *  Created on: Jan 9, 2025
 *      Author: andrew
 */

#ifndef SPI_SPIDMA_H_
#define SPI_SPIDMA_H_

#include "SPI.h"

class SPI_DMA: public SPIClass {
public:
	SPI_DMA(uint32_t mosi = MOSI, uint32_t miso = MISO, uint32_t sclk = SCK, uint32_t ssel = PNUM_NOT_DEFINED)
	  : SPIClass(mosi, miso, sclk, ssel) {};

	virtual void begin();
	virtual void end();

	virtual void beginTransaction(SPISettings settings);
	virtual void endTransaction();

	virtual uint8_t transfer(uint8_t data, bool skipReceive);
	virtual uint16_t transfer16(uint16_t data, bool skipReceive);

	/* note if skipReceive is true, this is async, it returns while DMA is sending */
	virtual void transfer(void *buf, size_t count, bool skipReceive);

	/* bulk transfer, this waits for transfer to complete */
	virtual void transfer(const void *tx_buf, void *rx_buf, size_t count);

	/* note this is async, it returns while DMA is sending/receiving */
	virtual void transfer_async(const void *tx_buf, void *rx_buf, size_t count);

	/* check if transfer is complete */
	virtual boolean isTransferComplete();

	virtual ~SPI_DMA();

protected:
	virtual uint32_t getClkFreq(spi_t *obj);
	virtual void init();

	virtual void initSPI();
	virtual void initDMA();
	virtual void initNVIC();
	virtual void initPINS();

	//SPI handle from base class
    //SPI_HandleTypeDef* hspi;

    DMA_HandleTypeDef hdma_tx;
    DMA_HandleTypeDef hdma_rx;

private:
    /* Current SPISettings */
    SPISettings   _spiSettings = SPISettings();


};

#endif /* SPI_SPIDMA_H_ */
