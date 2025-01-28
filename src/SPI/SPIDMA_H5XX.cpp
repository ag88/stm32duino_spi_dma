/*
 * DMASPI.cpp
 *
 *  Created on: Jan 9, 2025
 *      Author: andrew
 */

#if defined(STM32H5xx)
#include "SPIDMA_H5XX.h"
#include "core_debug.h"
//#include "stm32yyxx_ll_spi.h"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_ll_spi.h"

/*
 * this class should override this initSPI() method, enable SPI clock
 * and call SPI_DMA::initSPIDefault(SPI_TypeDef *spi_reg) with the correct
 * SPI_REGISTER_BASE (e.g. SPI1) to initialize SPI
 */
void SPI_DMAH5::initSPI() {
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

void SPI_DMAH5::initDMA() {
	/* DMA controller clock enable */
	__HAL_RCC_GPDMA1_CLK_ENABLE();

    // Initialize DMA for TX
    hdma_tx.Instance = GPDMA1_Channel1;
    hdma_tx.Init.Request = GPDMA1_REQUEST_SPI1_TX;

    // Initialize DMA for RX
    hdma_rx.Instance = GPDMA1_Channel0;
    hdma_rx.Init.Request = GPDMA1_REQUEST_SPI1_RX;

    initDMADefault();
}

void SPI_DMAH5::initDMADefault() {
	/* DMA controller clock enable */
	//__HAL_RCC_DMA2_CLK_ENABLE();

    // Initialize DMA for TX
    hdma_tx.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.SrcInc = DMA_SINC_INCREMENTED;
    hdma_tx.Init.DestInc = DMA_DINC_FIXED;
    hdma_tx.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    hdma_tx.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    hdma_tx.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    hdma_tx.Init.SrcBurstLength = 1;
    hdma_tx.Init.DestBurstLength = 1;
    hdma_tx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
    hdma_tx.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    hdma_tx.Init.Mode = DMA_NORMAL;

    if (HAL_DMA_Init(&hdma_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&_spi.handle, hdmatx, hdma_tx);

    if (HAL_DMA_ConfigChannelAttributes(&hdma_tx, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }


    // Initialize DMA for RX
    /* GPDMA1_REQUEST_SPI1_RX Init */
    hdma_rx.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.SrcInc = DMA_SINC_FIXED;
    hdma_rx.Init.DestInc = DMA_DINC_INCREMENTED;
    hdma_rx.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    hdma_rx.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    hdma_rx.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    hdma_rx.Init.SrcBurstLength = 1;
    hdma_rx.Init.DestBurstLength = 1;
    hdma_rx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
    hdma_rx.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    hdma_rx.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&hdma_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&_spi.handle, hdmarx, hdma_rx);

    if (HAL_DMA_ConfigChannelAttributes(&hdma_rx, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

}


void SPI_DMAH5::initNVIC() {
	// Configure NVIC for DMA (TX, RX)
	SPI_DMA::initNVIC(GPDMA1_Channel1_IRQn, GPDMA1_Channel0_IRQn);
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
uint32_t SPI_DMAH5::getClkFreq() {
	return HAL_RCC_GetPCLK2Freq();
}

SPI_DMAH5::~SPI_DMAH5() {
	// TODO Auto-generated destructor stub
}

/* SPI 2 */

/*
 * this class should override this initSPI() method, enable SPI clock
 * and call SPI_DMA::initSPIDefault(SPI_TypeDef *spi_reg) with the correct
 * SPI_REGISTER_BASE (e.g. SPI1) to initialize SPI
 */
void SPI_DMAH5_SPI2::initSPI() {
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

void SPI_DMAH5_SPI2::initDMA() {
	/* DMA controller clock enable */
	__HAL_RCC_GPDMA2_CLK_ENABLE();

    // Initialize DMA for TX
    hdma_tx.Instance = GPDMA2_Channel1;
    hdma_tx.Init.Request = GPDMA2_REQUEST_SPI2_TX;

    // Initialize DMA for RX
    hdma_rx.Instance = GPDMA2_Channel0;
    hdma_rx.Init.Request = GPDMA2_REQUEST_SPI2_RX;

    SPI_DMAH5::initDMADefault();

}

void SPI_DMAH5_SPI2::initNVIC() {
	// Configure NVIC for DMA (TX, RX)
	SPI_DMA::initNVIC(GPDMA2_Channel1_IRQn, GPDMA2_Channel0_IRQn);

}

uint32_t SPI_DMAH5_SPI2::getClkFreq() {
	/* SPI_2 and SPI_3. Source CLK is PCKL1 */
	return HAL_RCC_GetPCLK1Freq();
}


/* SPI 3 */

/*
 * this class should override this initSPI() method, enable SPI clock
 * and call SPI_DMA::initSPIDefault(SPI_TypeDef *spi_reg) with the correct
 * SPI_REGISTER_BASE (e.g. SPI1) to initialize SPI
 */
void SPI_DMAH5_SPI3::initSPI() {
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

void SPI_DMAH5_SPI3::initDMA() {
	/* DMA controller clock enable */
	__HAL_RCC_GPDMA2_CLK_ENABLE();

    // Initialize DMA for TX
    hdma_tx.Instance = GPDMA2_Channel3;
    hdma_tx.Init.Request = GPDMA2_REQUEST_SPI3_TX;

    // Initialize DMA for RX
    hdma_rx.Instance = GPDMA2_Channel2;
    hdma_rx.Init.Request = GPDMA2_REQUEST_SPI3_RX;

    SPI_DMAH5::initDMADefault();

}

void SPI_DMAH5_SPI3::initNVIC() {
	// Configure NVIC for DMA (TX, RX)
	SPI_DMA::initNVIC(GPDMA2_Channel3_IRQn, GPDMA2_Channel2_IRQn);

}

uint32_t SPI_DMAH5_SPI3::getClkFreq() {
	/* SPI_2 and SPI_3. Source CLK is PCKL1 */
	return HAL_RCC_GetPCLK1Freq();
}

#endif //STM32H5xx
