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
	_spi.handle.State = HAL_SPI_STATE_RESET;
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
	if (_spi.spi == SPI1 && ! __HAL_RCC_SPI1_IS_CLK_ENABLED() ) {
		__HAL_RCC_SPI1_CLK_ENABLE();
		__HAL_RCC_SPI1_FORCE_RESET();
		__HAL_RCC_SPI1_RELEASE_RESET();
	}
#endif

	initSPIDefault(NULL);
}
*/

void SPI_DMA::initSPIDefault(SPI_TypeDef *spi_reg) {

	if (spi_reg != NULL) {
		_spi.spi = spi_reg;
	} else {
		core_debug("_spi.spi must not be NULL");
		return;
	}

	uint32_t speed = _spiSettings.getClockFreq();
	SPIMode spimode = _spiSettings.getDataMode();
	BitOrder bitorder = _spiSettings.getBitOrder();

	/*
    _spi.handle.Instance = SPI1;
    _spi.handle.Init.Mode = SPI_MODE_MASTER;
    _spi.handle.Init.Direction = SPI_DIRECTION_2LINES;
    _spi.handle.Init.DataSize = SPI_DATASIZE_8BIT;
    _spi.handle.Init.CLKPolarity = SPI_POLARITY_LOW;
    _spi.handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    _spi.handle.Init.NSS = SPI_NSS_SOFT;
    _spi.handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    _spi.handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    _spi.handle.Init.TIMode = SPI_TIMODE_DISABLE;
    _spi.handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    _spi.handle.Init.CRCPolynomial = 10;
    */


	  // Configure the SPI pins
	  if (_spi.pin_ssel != NC) {
	    _spi.handle.Init.NSS = SPI_NSS_HARD_OUTPUT;
	  } else {
	    _spi.handle.Init.NSS = SPI_NSS_SOFT;
	  }

	  /* Fill default value */
	  _spi.handle.Instance			= _spi.spi;
	  _spi.handle.Init.Mode			= SPI_MODE_MASTER;

	  //uint32_t spi_freq = spi_getClkFreqInst(_spi.spi);
	  uint32_t spi_freq = getClkFreq(&_spi);
	  /* For SUBGHZSPI,  'SPI_BAUDRATEPRESCALER_*' == 'SUBGHZSPI_BAUDRATEPRESCALER_*' */
	  if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV2_MHZ)) {
	    _spi.handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV4_MHZ)) {
	    _spi.handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV8_MHZ)) {
	    _spi.handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV16_MHZ)) {
	    _spi.handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV32_MHZ)) {
	    _spi.handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV64_MHZ)) {
	    _spi.handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV128_MHZ)) {
	    _spi.handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	  } else {
	    /*
	     * As it is not possible to go below (spi_freq / SPI_SPEED_CLOCK_DIV256_MHZ).
	     * Set prescaler at max value so get the lowest frequency possible.
	     */
	    _spi.handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	  }

	  _spi.handle.Init.Direction         = SPI_DIRECTION_2LINES;

	  if ((spimode == SPI_MODE0) || (spimode == SPI_MODE2)) {
	    _spi.handle.Init.CLKPhase          = SPI_PHASE_1EDGE;
	  } else {
	    _spi.handle.Init.CLKPhase          = SPI_PHASE_2EDGE;
	  }

	  if ((spimode == SPI_MODE0) || (spimode == SPI_MODE1)) {
	    _spi.handle.Init.CLKPolarity       = SPI_POLARITY_LOW;
	  } else {
	    _spi.handle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
	  }

	  _spi.handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	  _spi.handle.Init.CRCPolynomial     = 7;
	  _spi.handle.Init.DataSize          = SPI_DATASIZE_8BIT;

	  if (bitorder == 0) {
	    _spi.handle.Init.FirstBit          = SPI_FIRSTBIT_LSB;
	  } else {
	    _spi.handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	  }

	  _spi.handle.Init.TIMode            = SPI_TIMODE_DISABLE;


    if (HAL_SPI_Init(&_spi.handle) != HAL_OK) {
        // Initialization Error
        Error_Handler();
    }

    __HAL_SPI_ENABLE(&_spi.handle);

}


void SPI_DMA::initNVIC(IRQn_Type DMA_IRQn_TX, IRQn_Type DMA_IRQn_RX) {
    // Configure NVIC for DMA
    HAL_NVIC_SetPriority(DMA_IRQn_TX, 0, 0);
    HAL_NVIC_EnableIRQ(DMA_IRQn_TX);

    HAL_NVIC_SetPriority(DMA_IRQn_RX, 0, 0);
    HAL_NVIC_EnableIRQ(DMA_IRQn_RX);
}

void SPI_DMA::initPins() {

    SPI_TypeDef *spi_mosi = (SPI_TypeDef *) pinmap_peripheral(_spi.pin_mosi, PinMap_SPI_MOSI);
    SPI_TypeDef *spi_miso = (SPI_TypeDef *) pinmap_peripheral(_spi.pin_miso, PinMap_SPI_MISO);
    SPI_TypeDef *spi_sclk = (SPI_TypeDef *) pinmap_peripheral(_spi.pin_sclk, PinMap_SPI_SCLK);
    SPI_TypeDef *spi_ssel = (SPI_TypeDef *) pinmap_peripheral(_spi.pin_ssel, PinMap_SPI_SSEL);

    if (spi_mosi == NULL || spi_miso == NULL || spi_sclk == NULL) {
      core_debug("ERROR: at least one SPI pin has no peripheral\n");
      return;
    }

    if (spi_mosi != _spi.spi || spi_miso != _spi.spi || spi_sclk != _spi.spi) {
    	uint8_t spinum = 0;
    	if( _spi.spi == SPI1 )
    		spinum = 1;
#if defined(SPI2)
    	else if( _spi.spi == SPI2 )
    		spinum = 2;
#endif
#if defined(SPI3)
    	else if( _spi.spi == SPI3 )
    		spinum = 3;
#endif
        core_debug("ERROR: assigned gpio pin does not match SPI %d\n", spinum);
        return;
    }

    /* Configure SPI GPIO pins */
    pinmap_pinout(_spi.pin_mosi, PinMap_SPI_MOSI);
    pinmap_pinout(_spi.pin_miso, PinMap_SPI_MISO);
    pinmap_pinout(_spi.pin_sclk, PinMap_SPI_SCLK);

    if (_spi.pin_ssel != NC)
    	pinmap_pinout(_spi.pin_ssel, PinMap_SPI_SSEL);

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
	// HAL_SPI_DMAPause(&_spi.handle);
	/* HAL LL based codes:
	while(!LL_SPI_IsActiveFlag_TXE(_spi.spi)); //spinlock
	LL_SPI_TransmitData8(_spi.spi, data);
	if (!skipReceive) {
		// do we need to timeout? in theory timeout should not happen
		while(!LL_SPI_IsActiveFlag_RXNE(_spi.spi)); //spinlock
		r = LL_SPI_ReceiveData8(_spi.spi);
	}
	*/

#if defined(SPI_SR_TXP)
	while( ! (_spi.spi->SR & SPI_SR_TXP_Msk) == 0 ); //spinlock
#else
	while( ! (_spi.spi->SR & SPI_SR_TXE_Msk) == 0 ); //spinlock
#endif
	LL_SPI_TransmitData8(_spi.spi, data);

	// do we need to timeout? in theory timeout should not happen
#if defined(SPI_SR_RXP)
	while( ! (_spi.spi->SR & SPI_SR_RXP_Msk) == 0 ); //spinlock
#else
	while( ! (_spi.spi->SR & SPI_SR_RXNE_Msk) == 0 ); //spinlock
#endif
	if (!skipReceive) {
		r = LL_SPI_ReceiveData8(_spi.spi);
	} else { // read the byte to clear the RXNE or RXP flag
		LL_SPI_ReceiveData8(_spi.spi);
	}

	// HAL_SPI_DMAResume(&_spi.handle);
	return r;
}

uint16_t SPI_DMA::transfer16(uint16_t data, bool skipReceive) {
	uint16_t r = 0;
	r = transfer(data & 0xffUL, skipReceive);
	r |= transfer((data >> 8) & 0xffUL, skipReceive) << 8;
	return r;
}

void SPI_DMA::transfer(void *buf, size_t count, bool skipReceive) {

	singleBufTransfer_.transfer(&_spi.handle, buf, count, skipReceive);

}


void SPI_DMA::transfer(const void *tx_buf, void *rx_buf, size_t count) {
	transfer_async(tx_buf, rx_buf, count);
	// wait for transfer to complete
	while(! isTransferComplete());
}


inline void SPI_DMA::transfer_async(const void *tx_buf, void *rx_buf, size_t count) {
	// assuming that DMA is 'pre-enabled'
	// HAL_SPI_DMAResume(&_spi.handle);
	HAL_SPI_TransmitReceive_DMA(&_spi.handle, (uint8_t*) tx_buf, (uint8_t*) rx_buf, count);
}

inline bool SPI_DMA::isTransferComplete() {
	return HAL_SPI_GetState(&_spi.handle) == HAL_SPI_STATE_READY;
}


/*
 * note this function returns SystemCoreClock by default
 * derived class should override this getClkFreq and specify the base clock
 * frequency (e.g. PCLK) used to derive the SPI pre-scalers for baud rates
 *
 * SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2
 * SPI_2 and SPI_3. Source CLK is PCKL1
 */
uint32_t SPI_DMA::getClkFreq(spi_t *obj) {
	return SystemCoreClock;
	//return SPIClass::getClkFreq(obj);
}

SPI_DMA::~SPI_DMA() {
	// TODO Auto-generated destructor stub
}

