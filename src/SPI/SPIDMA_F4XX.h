/*
 * DMASPI.h
 *
 *  Created on: Jan 9, 2025
 *      Author: andrew
 */

#ifndef SPI_SPIDMAF4XX_H_
#define SPI_SPIDMAF4XX_H_

#include "SPIDMA.h"

class SPI_DMAF4 : public SPI_DMA {
public:
	SPI_DMAF4(uint32_t mosi = MOSI, uint32_t miso = MISO, uint32_t sclk = SCK, uint32_t ssel = PNUM_NOT_DEFINED)
	  : SPI_DMA(mosi, miso, sclk, ssel) {};

	virtual void begin() override;

	virtual void beginTransaction(SPISettings settings) override;
	virtual void endTransaction();

	/* these are defined in base class SPI_DMA

	// virtual uint8_t transfer(uint8_t data, bool skipReceive);
	// virtual uint16_t transfer16(uint16_t data, bool skipReceive);

	// note if skipReceive is true, this is async, it returns while DMA is sending
	// virtual void transfer(void *buf, size_t count, bool skipReceive);

	// bulk transfer, this waits for transfer to complete
	// virtual void transfer(const void *tx_buf, void *rx_buf, size_t count);

	// note this is async, it returns while DMA is sending/receiving
	// virtual void transfer_async(const void *tx_buf, void *rx_buf, size_t count);

	// check if transfer is complete
	// virtual bool isTransferComplete();

	*/

	virtual ~SPI_DMAF4();

protected:
	//SPI handle from base class
    //SPI_HandleTypeDef* hspi;

    DMA_HandleTypeDef hdma_tx;
    DMA_HandleTypeDef hdma_rx;

    /* Current SPISettings */
    SPISettings   _spiSettings = SPISettings();

    /*
     * note this function should override SPI_DMA::getClkFreq (returns SystemCoreClock by default)
     * and should return the base clock frequency (e.g. PCLK)
     * used to derive the SPI pre-scalers for baud rates
     *
     * SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2
     * SPI_2 and SPI_3. Source CLK is PCKL1
     *
     */
	virtual uint32_t getClkFreq(spi_t *obj) override;

	virtual void init() override;

	/*
	 * this class should enable SPI clock and initialize SPI
	 * e.g. call initSPIDefault();
	 */
	virtual void initSPI() override;

	/*
	 * DMA needs to be initialized for the common parts for each series
	 * which is in this class for F4
	 */
	virtual void initDMA() override;
	virtual void initDMADefault() override;

	/*
	 * override this and initialize NVIC interrupts for DMA
	 */
	virtual void initNVIC() override;

	// this is defined in base class SPI_DMA
	// virtual void initPins();

};

typedef class SPI_DMAF4 SPI_DMAF4_SPI1;

/* SPI2 */
class SPI_DMAF4_SPI2 : public SPI_DMAF4 {
public:
	SPI_DMAF4_SPI2(uint32_t mosi = MOSI, uint32_t miso = MISO, uint32_t sclk = SCK, uint32_t ssel = PNUM_NOT_DEFINED)
	  : SPI_DMAF4(mosi, miso, sclk, ssel) {};

	// in base class SPI_DMA4
	// virtual void begin();

	virtual ~SPI_DMAF4_SPI2();

protected:
	virtual void init() override;

	virtual void initSPI() override;

	virtual void initDMA() override;

	virtual void initNVIC() override;

	virtual uint32_t getClkFreq(spi_t *obj) override;
private:
};

/* SPI3 */
class SPI_DMAF4_SPI3 : public SPI_DMAF4 {
public:
	SPI_DMAF4_SPI3(uint32_t mosi = MOSI, uint32_t miso = MISO, uint32_t sclk = SCK, uint32_t ssel = PNUM_NOT_DEFINED)
	  : SPI_DMAF4(mosi, miso, sclk, ssel) {};

	// in base class SPI_DMA4
	// virtual void begin();

	virtual ~SPI_DMAF4_SPI3();

protected:
	virtual void init() override;

	virtual void initSPI() override;

	virtual void initDMA() override;

	virtual void initNVIC() override;

	virtual uint32_t getClkFreq(spi_t *obj) override;

private:
};


#endif /* SPI_SPIDMAF4XX_H_ */
