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

#elif defined(STM32G4xx)
#include "SPIDMA_G4XX.h"
extern SPI_DMAG4 SPI;

#elif defined(STM32F1xx)
#include "SPIDMA_F1XX.h"
extern SPI_DMAF1 SPI;

#else
#include "SPIClass.h"

extern SPIClass SPI;
#endif


#endif /* INCLUDED_SPI_H_ */
