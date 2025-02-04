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
typedef SPI_DMAF4 SPIClass1;
typedef SPI_DMAF4_SPI2 SPIClass2;
typedef SPI_DMAF4_SPI3 SPIClass3;

#elif defined(STM32F3xx)
#include "SPIDMA_F3XX.h"
extern SPI_DMAF3 SPI;
typedef SPI_DMAF3 SPIClass1;
typedef SPI_DMAF3_SPI2 SPIClass2;
typedef SPI_DMAF3_SPI3 SPIClass3;

#elif defined(STM32G4xx)
#include "SPIDMA_G4XX.h"
extern SPI_DMAG4 SPI;
typedef SPI_DMAG4 SPIClass1;
typedef SPI_DMAG4_SPI2 SPIClass2;
typedef SPI_DMAG4_SPI3 SPIClass3;

#elif defined(STM32H5xx)
#include "SPIDMA_H5XX.h"
extern SPI_DMAH5 SPI;
typedef SPI_DMAH5 SPIClass1;
typedef SPI_DMAH5_SPI2 SPIClass2;
typedef SPI_DMAH5_SPI3 SPIClass3;

#elif defined(STM32F7xx)
#include "SPIDMA_F7XX.h"
extern SPI_DMAF7 SPI;
typedef SPI_DMAF7 SPIClass1;
typedef SPI_DMAF7_SPI2 SPIClass2;
typedef SPI_DMAF7_SPI3 SPIClass3;

#elif defined(STM32H7xx)
#include "SPIDMA_H7XX.h"
extern SPI_DMAH7 SPI;
typedef SPI_DMAH7 SPIClass1;
typedef SPI_DMAH7_SPI2 SPIClass2;
typedef SPI_DMAH7_SPI3 SPIClass3;


#elif defined(STM32F1xx)
#include "SPIDMA_F1XX.h"
extern SPI_DMAF1 SPI;
typedef SPI_DMAF1 SPIClass1;
typedef SPI_DMAF1_SPI2 SPIClass2;
#if defined(SPI3_BASE)
typedef SPI_DMAF1_SPI3 SPIClass3;
#endif

#else
#include "SPIBasic.h"
extern SPIBasic SPI;
typedef SPIBasic SPIClass1;
typedef SPIBasic_SPI2 SPIClass2;
typedef SPIBasic_SPI3 SPIClass3;


#endif


#endif /* INCLUDED_SPI_H_ */
