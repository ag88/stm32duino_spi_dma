/*
 * DMASPI.cpp
 *
 *  Created on: Jan 9, 2025
 *      Author: andrew
 */

#include "SPIDMA_F4XX.h"
#include "core_debug.h"
#include "stm32yyxx_ll_spi.h"

void SPI_DMAF4::begin() {
	_spi.handle.State = HAL_SPI_STATE_RESET;
	_spiSettings = SPISettings();
	init();
}

void SPI_DMAF4::init() {
	// class that override init() method from SPI_DMA should specify _spi.spi
	_spi.spi = SPI1;

	SPI_DMA::init();
	//SPIClass::init();
	//initSPI();
	//initDMA();
	//initNVIC();
	//initPins();
}

/*
 * in initSPI() this class should enable SPI clock and initialize SPI
 * e.g. call initSPIDefault();
 */
void SPI_DMAF4::initSPI() {
#if defined SPI1_BASE
	// Enable SPI clock
	if ( ! __HAL_RCC_SPI1_IS_CLK_ENABLED() ) {
		__HAL_RCC_SPI1_CLK_ENABLE();
		__HAL_RCC_SPI1_FORCE_RESET();
		__HAL_RCC_SPI1_RELEASE_RESET();
	}
#endif
	SPI_DMA::initSPIDefault();
}

void SPI_DMAF4::initDMA() {
	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

    // Initialize DMA for TX
    hdma_tx.Instance = DMA2_Stream3;
    hdma_tx.Init.Channel = DMA_CHANNEL_3;

    // Initialize DMA for RX
    hdma_rx.Instance = DMA2_Stream0;
    hdma_rx.Init.Channel = DMA_CHANNEL_3;

    initDMADefault();
}

void SPI_DMAF4::initDMADefault() {
	/* DMA controller clock enable */
	//__HAL_RCC_DMA2_CLK_ENABLE();

    // Initialize DMA for TX
    //hdma_tx.Instance = DMA2_Stream3;
    //hdma_tx.Init.Channel = DMA_CHANNEL_3;
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
    //hdma_rx.Instance = DMA2_Stream0;
    //hdma_rx.Init.Channel = DMA_CHANNEL_3;
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


void SPI_DMAF4::initNVIC() {
	// Configure NVIC for DMA (TX, RX)
	SPI_DMA::initNVIC(DMA2_Stream3_IRQn, DMA2_Stream0_IRQn);
}


void SPI_DMAF4::beginTransaction(SPISettings settings) {
	if (_spiSettings != settings) {
		_spiSettings = settings;
		initSPI();
	}

}

void SPI_DMAF4::endTransaction() {
}


uint32_t SPI_DMAF4::getClkFreq(spi_t *obj) {
	return SPIClass::getClkFreq(obj);
}

SPI_DMAF4::~SPI_DMAF4() {
	// TODO Auto-generated destructor stub
}

/* SPI 2 */
void SPI_DMAF4_SPI2::init() {
	// class that override init() method from SPI_DMA should specify _spi.spi
	_spi.spi = SPI2;

	SPI_DMA::init();
}

void SPI_DMAF4_SPI2::initSPI() {
#if defined SPI2_BASE
	// Enable SPI clock
	if ( ! __HAL_RCC_SPI2_IS_CLK_ENABLED() ) {
		__HAL_RCC_SPI2_CLK_ENABLE();
		__HAL_RCC_SPI2_FORCE_RESET();
		__HAL_RCC_SPI2_RELEASE_RESET();
	}
#endif
	SPI_DMA::initSPIDefault();

}

void SPI_DMAF4_SPI2::initDMA() {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

    // Initialize DMA for TX
    hdma_tx.Instance = DMA1_Stream4;
    hdma_tx.Init.Channel = DMA_CHANNEL_0;

    // Initialize DMA for RX
    hdma_rx.Instance = DMA1_Stream3;
    hdma_rx.Init.Channel = DMA_CHANNEL_0;

    SPI_DMAF4::initDMADefault();

}

void SPI_DMAF4_SPI2::initNVIC() {
	// Configure NVIC for DMA (TX, RX)
	SPI_DMA::initNVIC(DMA1_Stream4_IRQn, DMA1_Stream3_IRQn);

}

/* SPI 3 */
void SPI_DMAF4_SPI3::init() {
	// class that override init() method from SPI_DMA should specify _spi.spi
	_spi.spi = SPI3;

	SPI_DMA::init();
}

void SPI_DMAF4_SPI3::initSPI() {
#if defined SPI3_BASE
	// Enable SPI clock
	if ( ! __HAL_RCC_SPI3_IS_CLK_ENABLED() ) {
		__HAL_RCC_SPI3_CLK_ENABLE();
		__HAL_RCC_SPI3_FORCE_RESET();
		__HAL_RCC_SPI3_RELEASE_RESET();
	}
#endif
	SPI_DMA::initSPIDefault();

}

void SPI_DMAF4_SPI3::initDMA() {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

    // Initialize DMA for TX
    hdma_tx.Instance = DMA1_Stream2;
    hdma_tx.Init.Channel = DMA_CHANNEL_0;

    // Initialize DMA for RX
    hdma_rx.Instance = DMA1_Stream5;
    hdma_rx.Init.Channel = DMA_CHANNEL_0;

    SPI_DMAF4::initDMADefault();

}

void SPI_DMAF4_SPI3::initNVIC() {
	// Configure NVIC for DMA (TX, RX)
	SPI_DMA::initNVIC(DMA1_Stream2_IRQn, DMA1_Stream5_IRQn);

}


