/*
 * SingleBufTransferCopy.h
 *
 *  Created on: Jan 25, 2025
 *      Author: andrew
 */

#ifndef SRC_SPI_SINGLEBUFXFERCOPY_H_
#define SRC_SPI_SINGLEBUFXFERCOPY_H_

#include "SingleBufferTransfer.h"

class SingleBufTransferCopy: public SingleBufferTransfer {
public:
	SingleBufTransferCopy();

	/*
	 * This method create a temporary buffer on the stack to receive data during
	 * a DMA transfer, then copies the temp buffer back into buf
	 */
	void transfer(SPI_HandleTypeDef *hspi, void *buf, size_t count, bool skipReceive) const override;

	virtual ~SingleBufTransferCopy();
};

extern const SingleBufferTransfer& singleBufTransferImpl;

#endif /* SRC_SPI_SINGLEBUFXFERCOPY_H_ */
