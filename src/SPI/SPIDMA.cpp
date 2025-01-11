/*
 * DMASPI.cpp
 *
 *  Created on: Jan 9, 2025
 *      Author: andrew
 */

#include "SPIDMA.h"
#include "core_debug.h"
#include "stm32yyxx_ll_spi.h"

void SPI_DMA::begin() {
	_spi.handle.State = HAL_SPI_STATE_RESET;
	_spiSettings = SPISettings();
	init();
}

void SPI_DMA::init() {

	// class that override this init() method should specify _spi.spi
	// _spi.spi = SPI1;

	//SPIClass::init();
	initSPI();
	initDMA();
	initNVIC();
	initPins();
}

void SPI_DMA::initSPI() {
/* class that derives SPI_DMA should enable SPI clock and
 * initialize SPI e.g. call initSPIDefault();
#if defined SPI1_BASE
	// Enable SPI clock
	if (_spi.spi == SPI1 && ! __HAL_RCC_SPI1_IS_CLK_ENABLED() ) {
		__HAL_RCC_SPI1_CLK_ENABLE();
		__HAL_RCC_SPI1_FORCE_RESET();
		__HAL_RCC_SPI1_RELEASE_RESET();
	}
#endif
*/
	initSPIDefault();
}

void SPI_DMA::initSPIDefault() {

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

    /* Configure SPI GPIO pins */
    pinmap_pinout(_spi.pin_mosi, PinMap_SPI_MOSI);
    pinmap_pinout(_spi.pin_miso, PinMap_SPI_MISO);
    pinmap_pinout(_spi.pin_sclk, PinMap_SPI_SCLK);

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
	HAL_SPI_DMAPause(&_spi.handle);
	while(!LL_SPI_IsActiveFlag_TXE(_spi.spi)); //spinlock
	LL_SPI_TransmitData8(_spi.spi, data);
	if (!skipReceive) {
		/* do we need to timeout? in theory timeout should not happen */
		while(!LL_SPI_IsActiveFlag_RXNE(_spi.spi)); //spinlock
		r = LL_SPI_ReceiveData8(_spi.spi);
	}
	HAL_SPI_DMAResume(&_spi.handle);
	return r;
}

uint16_t SPI_DMA::transfer16(uint16_t data, bool skipReceive) {
	uint16_t r = 0;
	r = transfer(data & 0xffUL, skipReceive);
	r |= transfer((data >> 8) & 0xffUL, skipReceive) << 8;
	return r;
}

void SPI_DMA::transfer(const void *tx_buf, void *rx_buf, size_t count) {
	transfer_async(tx_buf, rx_buf, count);
	// wait for transfer to complete
	while(! isTransferComplete());
}


inline void SPI_DMA::transfer_async(const void *tx_buf, void *rx_buf, size_t count) {
	HAL_SPI_DMAResume(&_spi.handle);
	HAL_SPI_TransmitReceive_DMA(&_spi.handle, (uint8_t*) tx_buf, (uint8_t*) rx_buf, count);
}

inline bool SPI_DMA::isTransferComplete() {
	return HAL_SPI_GetState(&_spi.handle) == HAL_SPI_STATE_READY;
}


void SPI_DMA::transfer(void *buf, size_t count, bool skipReceive) {
	uint8_t *rx_buf;
	HAL_SPI_DMAResume(&_spi.handle);
	if (skipReceive) {
		HAL_SPI_Transmit_DMA(&_spi.handle, (uint8_t*) buf, count);
	} else {
		rx_buf = (uint8_t *) alloca(count);
		HAL_SPI_TransmitReceive_DMA(&_spi.handle, (uint8_t*) buf, (uint8_t*) rx_buf, count);
		//wait for transfer to complete
		while(HAL_SPI_GetState(&_spi.handle) != HAL_SPI_STATE_READY);
		memcpy(buf, rx_buf, count);
	}
}

uint32_t SPI_DMA::getClkFreq(spi_t *obj) {
	return SPIClass::getClkFreq(obj);
}

SPI_DMA::~SPI_DMA() {
	// TODO Auto-generated destructor stub
}

