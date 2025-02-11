/*
 * DMASPI.h
 *
 *  Created on: Jan 9, 2025
 *      Author: andrew
 *
 *  Credit for various codes here, and the original SPIClass implementation goes to
 *  Frederic Pillon (fpiSTM) and others who created the Arduino_Core_STM32 stm32duino core
 *
 *  https://github.com/stm32duino/Arduino_Core_STM32
 *
 *  Some of the SPIClass methods are overridden here with calls to HAL DMA transfer functions.
 *  e.g. those transfer() methods that use buffers
 *
 */

#ifndef SPI_SPIDMA_H_
#define SPI_SPIDMA_H_

#include "SPIClass.h"
#include "SingleBufTransferCopy.h"
//#include "SingleBufTransferInplace.h"

class SPI_DMA: public SPIClass {
public:
	SPI_DMA(uint32_t mosi = MOSI, uint32_t miso = MISO, uint32_t sclk = SCK, uint32_t ssel = PNUM_NOT_DEFINED,
		const SingleBufferTransfer &singleBufferTransfer = singleBufTransferImpl )
		: SPIClass(mosi, miso, sclk, ssel), singleBufTransfer_(singleBufferTransfer) {};

	virtual void begin() override;

	/* defined and implemented in SPIClass
	virtual void end();

	virtual void beginTransaction(SPISettings settings);
	virtual void endTransaction();
	*/

	virtual uint8_t transfer(uint8_t data, bool skipReceive = SPI_TRANSMITRECEIVE) override;

	/* defined and implemmented in SPIClass
	virtual uint16_t transfer16(uint16_t data, bool skipReceive = SPI_TRANSMITRECEIVE);
	*/

	/* note if skipReceive is true, this is async, it returns while DMA is sending */
	virtual void transfer(void *buf, size_t count, bool skipReceive = SPI_TRANSMITRECEIVE) override;

	/* bulk transfer, this waits for transfer to complete */
	virtual void transfer(const void *tx_buf, void *rx_buf, size_t count) override;

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
     * edit:
     * this is reimplemented using the SPIBasic generic getClkFreq codes.
     *
     * old notes:
     * note this function returns SystemCoreClock by default
     * derived class should override this getClkFreq and specify the base clock
     * frequency (e.g. PCLK) used to derive the SPI pre-scalers for baud rates
     *
     * SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2
     * SPI_2 and SPI_3. Source CLK is PCKL1
     */
	virtual uint32_t getClkFreq() override;

	/*
	 *  this is called by begin() which in turns call
	 *  	initSPI();
	 *		initDMA();
	 * 		initNVIC();
	 * 		initPins();
	 */
	virtual void init() override;

	/* derived class should override this initSPI() method, enable SPI clock
	 * and call SPI_DMA::initSPIDefault(SPI_TypeDef *spi_reg) with the correct
	 * SPI_REGISTER_BASE (e.g. SPI1) to initialize SPI
	 */
	virtual void initSPI() = 0;
	virtual void initSPIDefault(SPI_TypeDef *spi_reg);

	/* DMA initialization are pure virtual functions because DMA hardware and definition
	 * is different between series
	 */
	virtual void initDMA() = 0;
	virtual void initDMADefault() = 0;

	/* initNVIC is pure virtual, derived class should override/implement initNVIC()
	 * and call SPI_DMA::initNVIC(IRQn_Type DMA_IRQn_TX, IRQn_Type DMA_IRQn_RX)
	 * to initialise the DMA NVIC hooks
	 */
	virtual void initNVIC() = 0;
	virtual void initNVIC(IRQn_Type DMA_IRQn_TX, IRQn_Type DMA_IRQn_RX);
	virtual void initPins();

private:
	const SingleBufferTransfer& singleBufTransfer_;

};

#endif /* SPI_SPIDMA_H_ */
