/*
 * SingleBufTransferInplace.h
 *
 *  Created on: Jan 25, 2025
 *      Author: andrew
 */

#ifndef SRC_SPI_SINGLEBUFTRANSFERINPLACE_H_
#define SRC_SPI_SINGLEBUFTRANSFERINPLACE_H_

#include "SingleBufferTransfer.h"

class SingleBufTransferInplace: public SingleBufferTransfer {
public:
	SingleBufTransferInplace();

	/*
	 * This method use buf for both DMA transfer and receive,
	 * it does not check if the data may be overwritten.
	 */
	void transfer(SPI_HandleTypeDef *hspi, void *buf, size_t count, bool skipReceive) const override;

	virtual ~SingleBufTransferInplace();

};

extern const SingleBufferTransfer& singleBufTransferImpl;

#endif /* SRC_SPI_SINGLEBUFTRANSFERINPLACE_H_ */
