/*
 * SingleBufTransferCopy.cpp
 *
 *  Created on: Jan 25, 2025
 *      Author: andrew
 */

#include "SingleBufTransferCopy.h"

SingleBufTransferCopy::SingleBufTransferCopy() {
	// TODO Auto-generated constructor stub

}

/*
 * This method create a temporary buffer on the stack to receive data during
 * a DMA transfer, then copies the temp buffer back into buf
 */

void SingleBufTransferCopy::transfer(SPI_HandleTypeDef *hspi, void *buf, size_t count, bool skipReceive) const {
	uint8_t *rx_buf;
	// assuming that DMA is 'pre-enabled'
	// HAL_SPI_DMAResume(&_spi.handle);
	if (skipReceive) {
		HAL_SPI_Transmit_DMA(hspi, (uint8_t*) buf, count);
	} else {
		/* can we use the same buffer? that can save up on allocating
		 * a temporary buffer on the stack here and save up on a memcpy */
		rx_buf = (uint8_t *) alloca(count);
		HAL_SPI_TransmitReceive_DMA(hspi, (uint8_t*) buf, (uint8_t*) rx_buf, count);
		//wait for transfer to complete
		while(HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY);
		memcpy(buf, rx_buf, count);
	}
}


SingleBufTransferCopy::~SingleBufTransferCopy() {
	// TODO Auto-generated destructor stub
}

extern const SingleBufferTransfer& singleBufTransferImpl = SingleBufTransferCopy();

