/*
 * SPI.h
 *
 *  Created on: Jan 12, 2025
 *      Author: andrew
 */

#ifndef INCLUDED_SPI_H_
#define INCLUDED_SPI_H_

#if defined(STM32F4xx)
#include "SPIDMA_F4XX.h"
extern SPI_DMAF4 SPI;

#elif defined(STM32F3xx)
#include "SPIDMA_F3XX.h"
extern SPI_DMAF3 SPI;

#elif defined(STM32G4xx)
#include "SPIDMA_G4XX.h"
extern SPI_DMAG4 SPI;

#elif defined(STM32H5xx)
#include "SPIDMA_H5XX.h"
extern SPI_DMAH5 SPI;

#elif defined(STM32H7xx)
#include "SPIDMA_H7XX.h"
extern SPI_DMAH7 SPI;

#elif defined(STM32F1xx)
#include "SPIDMA_F1XX.h"
extern SPI_DMAF1 SPI;

#else
#include "SPIBasic.h"
extern SPIBasic SPI;

#endif


#endif /* INCLUDED_SPI_H_ */
