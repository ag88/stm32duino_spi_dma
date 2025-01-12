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
	virtual bool isTransferComplete();

	virtual ~SPI_DMA();

protected:
	//SPI handle from base class
    //SPI_HandleTypeDef* hspi;

    DMA_HandleTypeDef hdma_tx;
    DMA_HandleTypeDef hdma_rx;
    /* Current SPISettings */
    SPISettings   _spiSettings = SPISettings();

    /*
     * note this function returns SystemCoreClock by default
     * derived class should override this getClkFreq and specify the base clock
     * frequency (e.g. PCLK) used to derive the SPI pre-scalers for baud rates
     *
     * SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2
     * SPI_2 and SPI_3. Source CLK is PCKL1
     */
	virtual uint32_t getClkFreq(spi_t *obj);

	// derived class should override this init() method and specify _spi.spi see definition
	virtual void init();

	virtual void initSPI();
	virtual void initSPIDefault();

	/* DMA initialization are pure virtual functions because DMA hardware and definition
	 * is different between series
	 */
	virtual void initDMA() = 0;
	virtual void initDMADefault() = 0;

	// initNVIC is pure virtual, derived class from
	virtual void initNVIC() = 0;
	virtual void initNVIC(IRQn_Type DMA_IRQn_TX, IRQn_Type DMA_IRQn_RX);
	virtual void initPins();

private:

};

#endif /* SPI_SPIDMA_H_ */
