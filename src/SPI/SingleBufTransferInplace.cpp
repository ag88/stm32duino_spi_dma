/*
 * SingleBufTransferInplace.cpp
 *
 *  Created on: Jan 25, 2025
 *      Author: andrew
 */

#include "SingleBufTransferInplace.h"


SingleBufTransferInplace::SingleBufTransferInplace() {
	// TODO Auto-generated constructor stub

}

/*
 * This method use buf for both DMA transfer and receive,
 * it does not check if the data may be overwritten.
 */
void SingleBufTransferInplace::transfer(SPI_HandleTypeDef *hspi, void *buf, size_t count, bool skipReceive) const {
	// assuming that DMA is 'pre-enabled'
	// HAL_SPI_DMAResume(&_spi.handle);
	if (skipReceive) {
		HAL_SPI_Transmit_DMA(hspi, (uint8_t*) buf, count);
	} else {
		// note buf is used for both transmit and receive here
		HAL_SPI_TransmitReceive_DMA(hspi, (uint8_t*) buf, (uint8_t*) buf, count);
		// wait for transfer to complete
		while(HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY);
	}
}


SingleBufTransferInplace::~SingleBufTransferInplace() {
	// TODO Auto-generated destructor stub
}

const SingleBufferTransfer& singleBufTransferImpl = SingleBufTransferInplace();
