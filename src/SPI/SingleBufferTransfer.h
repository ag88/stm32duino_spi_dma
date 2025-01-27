/*
 * SingleBufferTransfer.h
 *
 *  Created on: Jan 25, 2025
 *      Author: andrew
 */

#ifndef SRC_SPI_SINGLEBUFTRANSFER_H_
#define SRC_SPI_SINGLEBUFTRANSFER_H_

extern "C" {
#include "stm32_def.h"
}


class SingleBufferTransfer {
public:
	SingleBufferTransfer();

	virtual void transfer(SPI_HandleTypeDef *hspi, void *buf, size_t count, bool skipReceive) const = 0;

	virtual ~SingleBufferTransfer();
};

#endif /* SRC_SPI_SINGLEBUFTRANSFER_H_ */
