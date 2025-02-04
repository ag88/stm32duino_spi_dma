/*
 * DMASPI.h
 *
 *  Created on: Jan 9, 2025
 *      Author: andrew
 */

#ifndef SPI_SPIDMAF7XX_H_
#define SPI_SPIDMAF7XX_H_

#if defined(STM32F7xx)
#include "SPIDMA.h"


class SPI_DMAF7 : public SPI_DMA {
public:
	SPI_DMAF7(uint32_t mosi = MOSI, uint32_t miso = MISO, uint32_t sclk = SCK, uint32_t ssel = PNUM_NOT_DEFINED)
	  : SPI_DMA(mosi, miso, sclk, ssel) {};

	/* these are defined in base class SPI_DMA

	// virtual void begin() override;

	// virtual void beginTransaction(SPISettings settings) override;
	// virtual void endTransaction();

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

	virtual ~SPI_DMAF7();

protected:
	//SPI handle from base class
    //SPI_HandleTypeDef* hspi;

	// defined in SPI_DMA
    // DMA_HandleTypeDef hdma_tx;
    // DMA_HandleTypeDef hdma_rx;

	// defined in SPI_DMA
    /* Current SPISettings */
    // SPISettings   _spiSettings = SPISettings();

	// this is defined in SPI_DMA
	// virtual void init();

	/*
	 * this class should override this initSPI() method, enable SPI clock
	 * and call SPI_DMA::initSPIDefault(SPI_TypeDef *spi_reg) with the correct
	 * SPI_REGISTER_BASE (e.g. SPI1) to initialize SPI
	 */
	virtual void initSPI() override;

	/*
	 * DMA needs to be initialized for the common parts for each series
	 * which is in this class for F7
	 */
	virtual void initDMA() override;
	virtual void initDMADefault() override;

	/*
	 * override this and initialize NVIC interrupts for DMA
	 */
	virtual void initNVIC() override;

	// this is defined in base class SPI_DMA
	// virtual void initPins();

    /* implemented in SPIDMA
	virtual uint32_t getClkFreq() override;
	*/

};

typedef class SPI_DMAF7 SPI_DMAF7_SPI1;

/* SPI2 */
class SPI_DMAF7_SPI2 : public SPI_DMAF7 {
public:
	SPI_DMAF7_SPI2(uint32_t mosi = PB15, uint32_t miso = PB14, uint32_t sclk = PB13, uint32_t ssel = PNUM_NOT_DEFINED)
	  : SPI_DMAF7(mosi, miso, sclk, ssel) {};

	// in base class SPI_DMA4
	// virtual void begin();

	virtual ~SPI_DMAF7_SPI2();

protected:

	virtual void initSPI() override;

	virtual void initDMA() override;

	virtual void initNVIC() override;

	/* implemented in SPIDMA
	virtual uint32_t getClkFreq() override;
	*/
private:
};

/* SPI3 */
class SPI_DMAF7_SPI3 : public SPI_DMAF7 {
public:
	SPI_DMAF7_SPI3(uint32_t mosi = PB5, uint32_t miso = PB4, uint32_t sclk = PB3, uint32_t ssel = PNUM_NOT_DEFINED)
	  : SPI_DMAF7(mosi, miso, sclk, ssel) {};

	// in base class SPI_DMA4
	// virtual void begin();

	virtual ~SPI_DMAF7_SPI3();

protected:

	virtual void initSPI() override;

	virtual void initDMA() override;

	virtual void initNVIC() override;

	/* implemented in SPIDMA
	virtual uint32_t getClkFreq() override;
	*/

private:
};


#endif /* STM32F7xx */
#endif /* SPI_SPIDMAF7XX_H_ */
