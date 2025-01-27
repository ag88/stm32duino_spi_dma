/*
 * DMASPI.cpp
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

#include "SPIDMA.h"
#include "core_debug.h"
#include "stm32yyxx_ll_spi.h"
/*
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_spi.h"
*/

void SPI_DMA::begin() {
	spihandle.State = HAL_SPI_STATE_RESET;
	_spiSettings = SPISettings();
	init();
}

void SPI_DMA::init() {

	//SPIClass::init();
	initPins();
	initSPI();
	initDMA();
	initNVIC();

}

/* derived class should override this initSPI() method, enable SPI clock
 * and call SPI_DMA::initSPIDefault(SPI_TypeDef *spi_reg) with the correct
 * SPI_REGISTER_BASE (e.g. SPI1) to initialize SPI
 *
void SPI_DMA::initSPI() {
#if defined SPI1_BASE
	// Enable SPI clock
	if (spi_reg == SPI1 && ! __HAL_RCC_SPI1_IS_CLK_ENABLED() ) {
		__HAL_RCC_SPI1_CLK_ENABLE();
		__HAL_RCC_SPI1_FORCE_RESET();
		__HAL_RCC_SPI1_RELEASE_RESET();
	}
#endif

	initSPIDefault(NULL);
}
*/


#define SPI_SPEED_CLOCK_DIV2_MHZ    ((uint32_t)2)
#define SPI_SPEED_CLOCK_DIV4_MHZ    ((uint32_t)4)
#define SPI_SPEED_CLOCK_DIV8_MHZ    ((uint32_t)8)
#define SPI_SPEED_CLOCK_DIV16_MHZ   ((uint32_t)16)
#define SPI_SPEED_CLOCK_DIV32_MHZ   ((uint32_t)32)
#define SPI_SPEED_CLOCK_DIV64_MHZ   ((uint32_t)64)
#define SPI_SPEED_CLOCK_DIV128_MHZ  ((uint32_t)128)
#define SPI_SPEED_CLOCK_DIV256_MHZ  ((uint32_t)256)


void SPI_DMA::initSPIDefault(SPI_TypeDef *spireg) {

	if (spi_reg != NULL) {
		spi_reg = spireg;
	} else {
		core_debug("spi_reg must not be NULL");
		return;
	}

	uint32_t speed = _spiSettings.getClockFreq();
	SPIMode spimode = _spiSettings.getDataMode();
	BitOrder bitorder = _spiSettings.getBitOrder();

	if (speed == 0)
		speed = SPI_SPEED_CLOCK_DEFAULT;

	/*
    spihandle.Instance = SPI1;
    spihandle.Init.Mode = SPI_MODE_MASTER;
    spihandle.Init.Direction = SPI_DIRECTION_2LINES;
    spihandle.Init.DataSize = SPI_DATASIZE_8BIT;
    spihandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    spihandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    spihandle.Init.NSS = SPI_NSS_SOFT;
    spihandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    spihandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spihandle.Init.TIMode = SPI_TIMODE_DISABLE;
    spihandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spihandle.Init.CRCPolynomial = 10;
    */


	  // Configure the SPI pins
	  if (pin_ssel != NC) {
	    spihandle.Init.NSS = SPI_NSS_HARD_OUTPUT;
	  } else {
	    spihandle.Init.NSS = SPI_NSS_SOFT;
	  }

	  /* Fill default value */
	  spihandle.Instance			= spi_reg;
	  spihandle.Init.Mode			= SPI_MODE_MASTER;

	  //uint32_t spi_freq = spi_getClkFreqInst(spi_reg);
	  uint32_t spi_freq = getClkFreq();
	  /* For SUBGHZSPI,  'SPI_BAUDRATEPRESCALER_*' == 'SUBGHZSPI_BAUDRATEPRESCALER_*' */
	  if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV2_MHZ)) {
	    spihandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV4_MHZ)) {
	    spihandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV8_MHZ)) {
	    spihandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV16_MHZ)) {
	    spihandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV32_MHZ)) {
	    spihandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV64_MHZ)) {
	    spihandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV128_MHZ)) {
	    spihandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	  } else {
	    /*
	     * As it is not possible to go below (spi_freq / SPI_SPEED_CLOCK_DIV256_MHZ).
	     * Set prescaler at max value so get the lowest frequency possible.
	     */
	    spihandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	  }

	  spihandle.Init.Direction         = SPI_DIRECTION_2LINES;

	  if ((spimode == SPI_MODE0) || (spimode == SPI_MODE2)) {
	    spihandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
	  } else {
	    spihandle.Init.CLKPhase          = SPI_PHASE_2EDGE;
	  }

	  if ((spimode == SPI_MODE0) || (spimode == SPI_MODE1)) {
	    spihandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
	  } else {
	    spihandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
	  }

	  spihandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	  spihandle.Init.CRCPolynomial     = 7;
	  spihandle.Init.DataSize          = SPI_DATASIZE_8BIT;

	  if (bitorder == 0) {
	    spihandle.Init.FirstBit          = SPI_FIRSTBIT_LSB;
	  } else {
	    spihandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	  }

	  spihandle.Init.TIMode            = SPI_TIMODE_DISABLE;


    if (HAL_SPI_Init(&spihandle) != HAL_OK) {
        // Initialization Error
        Error_Handler();
    }

    __HAL_SPI_ENABLE(&spihandle);

}


void SPI_DMA::initNVIC(IRQn_Type DMA_IRQn_TX, IRQn_Type DMA_IRQn_RX) {
    // Configure NVIC for DMA
    HAL_NVIC_SetPriority(DMA_IRQn_TX, 0, 0);
    HAL_NVIC_EnableIRQ(DMA_IRQn_TX);

    HAL_NVIC_SetPriority(DMA_IRQn_RX, 0, 0);
    HAL_NVIC_EnableIRQ(DMA_IRQn_RX);
}

void SPI_DMA::initPins() {

    SPI_TypeDef *spi_mosi = (SPI_TypeDef *) pinmap_peripheral(pin_mosi, PinMap_SPI_MOSI);
    SPI_TypeDef *spi_miso = (SPI_TypeDef *) pinmap_peripheral(pin_miso, PinMap_SPI_MISO);
    SPI_TypeDef *spi_sclk = (SPI_TypeDef *) pinmap_peripheral(pin_sclk, PinMap_SPI_SCLK);
    SPI_TypeDef *spi_ssel = (SPI_TypeDef *) pinmap_peripheral(pin_ssel, PinMap_SPI_SSEL);

    if (spi_mosi == NULL || spi_miso == NULL || spi_sclk == NULL) {
      core_debug("ERROR: at least one SPI pin has no peripheral\n");
      return;
    }

    if (spi_mosi != spi_reg || spi_miso != spi_reg || spi_sclk != spi_reg) {
    	uint8_t spinum = 0;
    	if( spi_reg == SPI1 )
    		spinum = 1;
#if defined(SPI2)
    	else if( spi_reg == SPI2 )
    		spinum = 2;
#endif
#if defined(SPI3)
    	else if( spi_reg == SPI3 )
    		spinum = 3;
#endif
        core_debug("ERROR: assigned gpio pin does not match SPI %d\n", spinum);
        return;
    }

    /* Configure SPI GPIO pins */
    pinmap_pinout(pin_mosi, PinMap_SPI_MOSI);
    pinmap_pinout(pin_miso, PinMap_SPI_MISO);
    pinmap_pinout(pin_sclk, PinMap_SPI_SCLK);

    if (pin_ssel != NC)
    	pinmap_pinout(pin_ssel, PinMap_SPI_SSEL);

}


void SPI_DMA::end() {
}

void SPI_DMA::beginTransaction(SPISettings settings) {
	if (_spiSettings != settings) {
		_spiSettings = settings;
		initSPI();
	}

}

void SPI_DMA::endTransaction() {
}


uint8_t SPI_DMA::transfer(uint8_t data, bool skipReceive) {
	uint8_t r = 0;
	/* can DMA co-exist with single byte transfers? if not the DMA pause / resume are required */
	// HAL_SPI_DMAPause(&spihandle);
	// this is necessarily ugly due to different SPI hardware
	// many mcus use:
	//   TXE (transmit buf empty) flag
	//   RXNE (receive buf no empty) flag
	// and a single data register DR
	// while mcus like stm32 H5/H7 uses TXP / RXP flags and 2 data registers TXDR, RXDR

#if defined(SPI_SR_TXP)
	while( ! (spi_reg->SR & SPI_SR_TXP_Msk) == 0 ); //spinlock
#else
	while( ! (spi_reg->SR & SPI_SR_TXE_Msk) == 0 ); //spinlock
#endif
	//LL_SPI_TransmitData8(spi_reg, data);
#if defined(SPI_TXDR_TXDR)
	spi_reg->TXDR = data;
#else
	spi_reg->DR = data;
#endif

	// do we need to timeout? in theory timeout should not happen
#if defined(SPI_SR_RXP)
	while( ! (spi_reg->SR & SPI_SR_RXP_Msk) == 0 ); //spinlock
#else
	while( ! (spi_reg->SR & SPI_SR_RXNE_Msk) == 0 ); //spinlock
#endif

	//r = LL_SPI_ReceiveData8(spi_reg);
	// read the byte regardless of skipReceive to clear the RXNE or RXP flag
#if defined(SPI_TXDR_RXDR)
	r = spi_reg->RXDR & 0xffU;
#else
	r = spi_reg->DR & 0xffU;
#endif
	if (skipReceive) r = 0;

	// HAL_SPI_DMAResume(&spihandle);
	return r;
}


void SPI_DMA::transfer(void *buf, size_t count, bool skipReceive) {

	singleBufTransfer_.transfer(&spihandle, buf, count, skipReceive);

}


void SPI_DMA::transfer(const void *tx_buf, void *rx_buf, size_t count) {
	transfer_async(tx_buf, rx_buf, count);
	// wait for transfer to complete
	while(! isTransferComplete());
}


inline void SPI_DMA::transfer_async(const void *tx_buf, void *rx_buf, size_t count) {
	// assuming that DMA is 'pre-enabled'
	// HAL_SPI_DMAResume(&spihandle);
	HAL_SPI_TransmitReceive_DMA(&spihandle, (uint8_t*) tx_buf, (uint8_t*) rx_buf, count);
}

inline bool SPI_DMA::isTransferComplete() {
	return HAL_SPI_GetState(&spihandle) == HAL_SPI_STATE_READY;
}


/*
 * note this function returns SystemCoreClock by default
 * derived class should override this getClkFreq and specify the base clock
 * frequency (e.g. PCLK) used to derive the SPI pre-scalers for baud rates
 *
 * SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2
 * SPI_2 and SPI_3. Source CLK is PCKL1
 */
uint32_t SPI_DMA::getClkFreq() {
	return SystemCoreClock;
	//return SPIClass::getClkFreq(obj);
}

SPI_DMA::~SPI_DMA() {
	// TODO Auto-generated destructor stub
}

