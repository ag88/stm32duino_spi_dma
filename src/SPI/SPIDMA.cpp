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
}

void SPI_DMA::init() {
	SPIClass::init();
	initSPI();
	initDMA();
	initNVIC();
	initPINS();
}

void SPI_DMA::initDMA() {
	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

    // Initialize DMA for TX
    hdma_tx.Instance = DMA2_Stream3;
    hdma_tx.Init.Channel = DMA_CHANNEL_3;
    hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode = DMA_NORMAL;
    hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&hdma_tx) != HAL_OK) {
        // Initialization Error
        Error_Handler();
    }

    __HAL_LINKDMA(&_spi.handle, hdmatx, hdma_tx);

    // Initialize DMA for RX
    hdma_rx.Instance = DMA2_Stream0;
    hdma_rx.Init.Channel = DMA_CHANNEL_3;
    hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode = DMA_NORMAL;
    hdma_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&hdma_rx) != HAL_OK) {
        // Initialization Error
        Error_Handler();
    }

    __HAL_LINKDMA(&_spi.handle, hdmarx, hdma_rx);
}

void SPI_DMA::initSPI() {

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

#if defined SPI1_BASE
	// Enable SPI clock
	if (_spi.spi == SPI1 && ! __HAL_RCC_SPI1_IS_CLK_ENABLED() ) {
		__HAL_RCC_SPI1_CLK_ENABLE();
		__HAL_RCC_SPI1_FORCE_RESET();
		__HAL_RCC_SPI1_RELEASE_RESET();
	}
#endif

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
	  uint32_t spi_freq = SPIClass::getClkFreq(&_spi);
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

void SPI_DMA::initNVIC() {
    // Configure NVIC for DMA
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void SPI_DMA::initPINS() {

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
	HAL_SPI_DMAResume(&_spi.handle);
	HAL_SPI_TransmitReceive_DMA(&_spi.handle, (uint8_t*) tx_buf, (uint8_t*) rx_buf, count);
}


void SPI_DMA::transfer(void *buf, size_t count, bool skipReceive) {
	uint8_t *rx_buf = (uint8_t *) alloca(count);
	HAL_SPI_DMAResume(&_spi.handle);
	HAL_SPI_TransmitReceive_DMA(&_spi.handle, (uint8_t*) buf, (uint8_t*) rx_buf, count);
	memcpy(buf, rx_buf, count);
}

uint32_t SPI_DMA::getClkFreq(spi_t *obj) {
	return SPIClass::getClkFreq(obj);
}

SPI_DMA::~SPI_DMA() {
	// TODO Auto-generated destructor stub
}

