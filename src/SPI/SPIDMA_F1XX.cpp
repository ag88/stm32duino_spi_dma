/*
 * DMASPI.cpp
 *
 *  Created on: Jan 9, 2025
 *      Author: andrew
 */

#if defined(STM32F1xx)
#include "SPIDMA_F1XX.h"
#include "core_debug.h"
//#include "stm32yyxx_ll_spi.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_spi.h"

SPI_DMAF1 SPI;

/*
 * this class should override this initSPI() method, enable SPI clock
 * and call SPI_DMA::initSPIDefault(SPI_TypeDef *spi_reg) with the correct
 * SPI_REGISTER_BASE (e.g. SPI1) to initialize SPI
 */
void SPI_DMAF1::initSPI() {
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

void SPI_DMAF1::initDMA() {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

    // Initialize DMA for TX
    hdma_tx.Instance = DMA1_Channel3;

    // Initialize DMA for RX
    hdma_rx.Instance = DMA1_Channel2;

    initDMADefault();
}

void SPI_DMAF1::initDMADefault() {
	/* DMA controller clock enable */
	//__HAL_RCC_DMA2_CLK_ENABLE();

    /* SPI1 DMA Init */
    /* SPI1_RX Init */
    hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode = DMA_NORMAL;
    hdma_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&spihandle,hdmarx,hdma_rx);

    /* SPI1_TX Init */
    hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode = DMA_NORMAL;
    hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&spihandle,hdmatx,hdma_tx);

}


void SPI_DMAF1::initNVIC() {
	// Configure NVIC for DMA (TX, RX)
	SPI_DMA::initNVIC(DMA1_Channel3_IRQn, DMA1_Channel2_IRQn);
}


SPI_DMAF1::~SPI_DMAF1() {
	// TODO Auto-generated destructor stub
}

/* SPI 2 */

/*
 * this class should override this initSPI() method, enable SPI clock
 * and call SPI_DMA::initSPIDefault(SPI_TypeDef *spi_reg) with the correct
 * SPI_REGISTER_BASE (e.g. SPI1) to initialize SPI
 */
void SPI_DMAF1_SPI2::initSPI() {
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

void SPI_DMAF1_SPI2::initDMA() {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

    // Initialize DMA for TX
    hdma_tx.Instance = DMA1_Channel5;

    // Initialize DMA for RX
    hdma_rx.Instance = DMA1_Channel4;

    SPI_DMAF1::initDMADefault();

}

void SPI_DMAF1_SPI2::initNVIC() {
	// Configure NVIC for DMA (TX, RX)
	SPI_DMA::initNVIC(DMA1_Channel5_IRQn, DMA1_Channel4_IRQn);

}


/* SPI 3 */
#if defined SPI3_BASE
/*
 * this class should override this initSPI() method, enable SPI clock
 * and call SPI_DMA::initSPIDefault(SPI_TypeDef *spi_reg) with the correct
 * SPI_REGISTER_BASE (e.g. SPI1) to initialize SPI
 */
void SPI_DMAF1_SPI3::initSPI() {
	// Enable SPI clock
	if ( ! __HAL_RCC_SPI3_IS_CLK_ENABLED() ) {
		__HAL_RCC_SPI3_CLK_ENABLE();
		__HAL_RCC_SPI3_FORCE_RESET();
		__HAL_RCC_SPI3_RELEASE_RESET();
	}


	/* this call sets the SPI instance in _spi.spi*/
	SPI_DMA::initSPIDefault( SPI3 );

}

void SPI_DMAF1_SPI3::initDMA() {
	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();


    // Initialize DMA for TX
    hdma_tx.Instance = DMA2_Channel2;

    // Initialize DMA for RX
    hdma_rx.Instance = DMA2_Channel1;

    SPI_DMAF1::initDMADefault();

}

void SPI_DMAF1_SPI3::initNVIC() {
	// Configure NVIC for DMA (TX, RX)
	SPI_DMA::initNVIC(DMA2_Channel2_IRQn, DMA2_Channel1_IRQn);

}

#endif //SPI3_BASE

#endif //STM32F1xx
