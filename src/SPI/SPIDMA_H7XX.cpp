/*
 * DMASPI.cpp
 *
 *  Created on: Jan 9, 2025
 *      Author: andrew
 */

#if defined(STM32H7xx)
#include "SPIDMA_H7XX.h"
#include "core_debug.h"
//#include "stm32yyxx_ll_spi.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_spi.h"

/*
 * this class should override this initSPI() method, enable SPI clock
 * and call SPI_DMA::initSPIDefault(SPI_TypeDef *spi_reg) with the correct
 * SPI_REGISTER_BASE (e.g. SPI1) to initialize SPI
 */
void SPI_DMAH7::initSPI() {
#if defined SPI1_BASE
	// Enable SPI clock
	if ( ! __HAL_RCC_SPI1_IS_CLK_ENABLED() ) {
		__HAL_RCC_SPI1_CLK_ENABLE();
		__HAL_RCC_SPI1_FORCE_RESET();
		__HAL_RCC_SPI1_RELEASE_RESET();
	}
#endif

	/* this call sets the SPI instance in _spi.spi*/
	SPI_DMA::initSPIDefault( SPI1 );
}

void SPI_DMAH7::initDMA() {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

    // Initialize DMA for TX
    hdma_tx.Instance = DMA1_Stream1;
    hdma_tx.Init.Request = DMA_REQUEST_SPI1_TX;

    // Initialize DMA for RX
    hdma_rx.Instance = DMA1_Stream0;
    hdma_rx.Init.Request = DMA_REQUEST_SPI1_RX;

    initDMADefault();
}

void SPI_DMAH7::initDMADefault() {
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
    hdma_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&hdma_rx) != HAL_OK) {
        // Initialization Error
        Error_Handler();
    }

    __HAL_LINKDMA(&_spi.handle, hdmarx, hdma_rx);
}


void SPI_DMAH7::initNVIC() {
	// Configure NVIC for DMA (TX, RX)
	SPI_DMA::initNVIC(DMA1_Stream1_IRQn, DMA1_Stream0_IRQn);
}



/*
 * note this function should override SPI_DMA::getClkFreq (returns SystemCoreClock by default)
 * and should return the base clock frequency (e.g. PCLK)
 * used to derive the SPI pre-scalers for baud rates
 *
 * SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2
 * SPI_2 and SPI_3. Source CLK is PCKL1
 *
 */
uint32_t SPI_DMAH7::getClkFreq(spi_t *obj) {
	return HAL_RCC_GetPCLK2Freq();
}

SPI_DMAH7::~SPI_DMAH7() {
	// TODO Auto-generated destructor stub
}

/* SPI 2 */

/*
 * this class should override this initSPI() method, enable SPI clock
 * and call SPI_DMA::initSPIDefault(SPI_TypeDef *spi_reg) with the correct
 * SPI_REGISTER_BASE (e.g. SPI1) to initialize SPI
 */
void SPI_DMAH7_SPI2::initSPI() {
#if defined SPI2_BASE
	// Enable SPI clock
	if ( ! __HAL_RCC_SPI2_IS_CLK_ENABLED() ) {
		__HAL_RCC_SPI2_CLK_ENABLE();
		__HAL_RCC_SPI2_FORCE_RESET();
		__HAL_RCC_SPI2_RELEASE_RESET();
	}
#endif

	/* this call sets the SPI instance in _spi.spi*/
	SPI_DMA::initSPIDefault( SPI2 );

}

void SPI_DMAH7_SPI2::initDMA() {
	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

    // Initialize DMA for TX
    hdma_tx.Instance = DMA2_Stream1;
    hdma_tx.Init.Request = DMA_REQUEST_SPI2_TX;

    // Initialize DMA for RX
    hdma_rx.Instance = DMA2_Stream0;
    hdma_rx.Init.Request = DMA_REQUEST_SPI2_RX;

    SPI_DMAH7::initDMADefault();

}

void SPI_DMAH7_SPI2::initNVIC() {
	// Configure NVIC for DMA (TX, RX)
	SPI_DMA::initNVIC(DMA2_Stream1_IRQn, DMA2_Stream0_IRQn);

}

uint32_t SPI_DMAH7_SPI2::getClkFreq(spi_t *obj) {
	/* SPI_2 and SPI_3. Source CLK is PCKL1 */
	return HAL_RCC_GetPCLK1Freq();
}


/* SPI 3 */

/*
 * this class should override this initSPI() method, enable SPI clock
 * and call SPI_DMA::initSPIDefault(SPI_TypeDef *spi_reg) with the correct
 * SPI_REGISTER_BASE (e.g. SPI1) to initialize SPI
 */
void SPI_DMAH7_SPI3::initSPI() {
#if defined SPI3_BASE
	// Enable SPI clock
	if ( ! __HAL_RCC_SPI3_IS_CLK_ENABLED() ) {
		__HAL_RCC_SPI3_CLK_ENABLE();
		__HAL_RCC_SPI3_FORCE_RESET();
		__HAL_RCC_SPI3_RELEASE_RESET();
	}
#endif

	/* this call sets the SPI instance in _spi.spi*/
	SPI_DMA::initSPIDefault( SPI3 );

}

void SPI_DMAH7_SPI3::initDMA() {
	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

    // Initialize DMA for TX
    hdma_tx.Instance = DMA2_Stream3;
    hdma_tx.Init.Request = DMA_REQUEST_SPI3_TX;

    // Initialize DMA for RX
    hdma_rx.Instance = DMA2_Stream2;
    hdma_rx.Init.Request = DMA_REQUEST_SPI3_RX;

    SPI_DMAH7::initDMADefault();

}

void SPI_DMAH7_SPI3::initNVIC() {
	// Configure NVIC for DMA (TX, RX)
	SPI_DMA::initNVIC(DMA2_Stream3_IRQn, DMA2_Stream2_IRQn);

}

uint32_t SPI_DMAH7_SPI3::getClkFreq(spi_t *obj) {
	/* SPI_2 and SPI_3. Source CLK is PCKL1 */
	return HAL_RCC_GetPCLK1Freq();
}

#endif //STM32H7xx
