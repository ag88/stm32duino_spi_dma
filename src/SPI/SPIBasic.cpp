/*
 * SPIBasic.cpp
 *
 *  Created on: Jan 9, 2025
 *      Author: andrew
 *
 *  Credit for various codes here, and the original SPIClass implementation goes to
 *  Frederic Pillon (fpiSTM) and others who created the Arduino_Core_STM32 stm32duino core
 *
 *  https://github.com/stm32duino/Arduino_Core_STM32
 *
 *  Some of the SPIClass methods are overridden here.
 *  e.g. those init() and transfer() methods
 *
 */

#include "SPIBasic.h"
#include "core_debug.h"
#include "stm32yyxx_ll_spi.h"
/*
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_spi.h"
*/

SPIBasic SPI;

void SPIBasic::begin() {
	spihandle.State = HAL_SPI_STATE_RESET;
	_spiSettings = SPISettings();
	init();
}

/* this overrides init() from SPIClass */
void SPIBasic::init() {

	//SPIClass::init();
	initPins();
	initSPI();
	//initNVIC();

}

/* initialize SPI Pins called by init()*/
void SPIBasic::initPins() {

#if defined(SUBGHZSPI_BASE)
  if (spi_reg != SUBGHZSPI) {
#endif

    SPI_TypeDef *spi_mosi = (SPI_TypeDef *) pinmap_peripheral(pin_mosi, PinMap_SPI_MOSI);
    SPI_TypeDef *spi_miso = (SPI_TypeDef *) pinmap_peripheral(pin_miso, PinMap_SPI_MISO);
    SPI_TypeDef *spi_sclk = (SPI_TypeDef *) pinmap_peripheral(pin_sclk, PinMap_SPI_SCLK);
    SPI_TypeDef *spi_ssel = (SPI_TypeDef *) pinmap_peripheral(pin_ssel, PinMap_SPI_SSEL);

    if (spi_mosi == NULL || spi_miso == NULL || spi_sclk == NULL) {
      core_debug("ERROR: at least one SPI pin has no peripheral\n");
      return;
    }

    SPI_TypeDef *spi_data = (SPI_TypeDef *) pinmap_merge_peripheral(spi_mosi, spi_miso);
    SPI_TypeDef *spi_cntl = (SPI_TypeDef *) pinmap_merge_peripheral(spi_sclk, spi_ssel);

    spi_reg = (SPI_TypeDef *) pinmap_merge_peripheral(spi_data, spi_cntl);

    // Are all pins connected to the same SPI instance?
    if (spi_data == NULL || spi_cntl == NULL || spi_reg == NULL) {
      core_debug("ERROR: SPI pins mismatch\n");
      return;
    }

    /* Configure SPI GPIO pins */
    pinmap_pinout(pin_mosi, PinMap_SPI_MOSI);
    pinmap_pinout(pin_miso, PinMap_SPI_MISO);
    pinmap_pinout(pin_sclk, PinMap_SPI_SCLK);

    if (pin_ssel != NC)
    	pinmap_pinout(pin_ssel, PinMap_SPI_SSEL);

#if defined(SUBGHZSPI_BASE)
  } else {
    if (pin_mosi != NC || pin_miso != NC || pin_sclk != NC || pin_ssel != NC) {
      core_debug("ERROR: SUBGHZ_SPI cannot define custom pins\n");
      return;
    }
  }
#endif

}


/* this implementation is generic, enable SPI clock
 * and call SPIBasic::initSPIDefault(SPI_TypeDef *spi_reg) with the correct
 * SPI_REGISTER_BASE (e.g. SPI1) to initialize SPI
 *
 **/
void SPIBasic::initSPI() {

	if (spi_reg == NULL) {
		spi_reg = getSPIBASE();
		if (spi_reg == NULL) return; // spi_reg == null !
	}

	enableSPICLK(spi_reg);
	initSPIDefault(spi_reg);
}

/*
 * returns SPI_BASE address from pins
 */
SPI_TypeDef* SPIBasic::getSPIBASE() {

	SPI_TypeDef* spibase;

#if defined(SUBGHZSPI_BASE)
    if (obj->handle.Instance == SUBGHZSPI)
    	return SUBGHZSPI;
#endif //else

    spibase = (SPI_TypeDef*) pinmap_peripheral(pin_sclk, PinMap_SPI_SCLK);

    if (spibase == NULL)
    	core_debug("getSPIBASE returns NULL, invalid pin assignments");

    return spibase;
}


#define SPI_SPEED_CLOCK_DIV2_MHZ    ((uint32_t)2)
#define SPI_SPEED_CLOCK_DIV4_MHZ    ((uint32_t)4)
#define SPI_SPEED_CLOCK_DIV8_MHZ    ((uint32_t)8)
#define SPI_SPEED_CLOCK_DIV16_MHZ   ((uint32_t)16)
#define SPI_SPEED_CLOCK_DIV32_MHZ   ((uint32_t)32)
#define SPI_SPEED_CLOCK_DIV64_MHZ   ((uint32_t)64)
#define SPI_SPEED_CLOCK_DIV128_MHZ  ((uint32_t)128)
#define SPI_SPEED_CLOCK_DIV256_MHZ  ((uint32_t)256)

void SPIBasic::initSPIDefault(SPI_TypeDef *spireg) {

	if (spi_reg != NULL) {
		spi_reg = spireg;
	} else {
		core_debug("spi_reg must not be NULL");
		return;
	}

	uint32_t speed = _spiSettings.getClockFreq();
	SPIMode spimode = _spiSettings.getDataMode();
	BitOrder bitorder = _spiSettings.getBitOrder();

	/*
    spihandle.Instance = SPI1;
    spihandle.Init.Mode = SPI_MODE_MASTER;
    spihandle.Init.Direction = SPI_DIRECTION_2LINES;
    spihandle.Init.DataSize = SPI_DATASIZE_8BIT;
    spihandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    spihandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    spihandle.Init.NSS = SPI_NSS_SOFT;
    spihandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    spihandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spihandle.Init.TIMode = SPI_TIMODE_DISABLE;
    spihandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spihandle.Init.CRCPolynomial = 10;
    */


	  // Configure the SPI pins
	  if (pin_ssel != NC) {
	    spihandle.Init.NSS = SPI_NSS_HARD_OUTPUT;
	  } else {
	    spihandle.Init.NSS = SPI_NSS_SOFT;
	  }

	  /* Fill default value */
	  spihandle.Instance			= spi_reg;
	  spihandle.Init.Mode			= SPI_MODE_MASTER;

	  //uint32_t spi_freq = spi_getClkFreqInst(spi_reg);
	  uint32_t spi_freq = getClkFreq();
	  /* For SUBGHZSPI,  'SPI_BAUDRATEPRESCALER_*' == 'SUBGHZSPI_BAUDRATEPRESCALER_*' */
	  if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV2_MHZ)) {
	    spihandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV4_MHZ)) {
	    spihandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV8_MHZ)) {
	    spihandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV16_MHZ)) {
	    spihandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV32_MHZ)) {
	    spihandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV64_MHZ)) {
	    spihandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV128_MHZ)) {
	    spihandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	  } else {
	    /*
	     * As it is not possible to go below (spi_freq / SPI_SPEED_CLOCK_DIV256_MHZ).
	     * Set prescaler at max value so get the lowest frequency possible.
	     */
	    spihandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	  }

	  spihandle.Init.Direction         = SPI_DIRECTION_2LINES;

	  if ((spimode == SPI_MODE0) || (spimode == SPI_MODE2)) {
	    spihandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
	  } else {
	    spihandle.Init.CLKPhase          = SPI_PHASE_2EDGE;
	  }

	  if ((spimode == SPI_MODE0) || (spimode == SPI_MODE1)) {
	    spihandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
	  } else {
	    spihandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
	  }

	  spihandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	  spihandle.Init.CRCPolynomial     = 7;
	  spihandle.Init.DataSize          = SPI_DATASIZE_8BIT;

	  if (bitorder == 0) {
	    spihandle.Init.FirstBit          = SPI_FIRSTBIT_LSB;
	  } else {
	    spihandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	  }

	  spihandle.Init.TIMode            = SPI_TIMODE_DISABLE;


    if (HAL_SPI_Init(&spihandle) != HAL_OK) {
        // Initialization Error
        Error_Handler();
    }

    __HAL_SPI_ENABLE(&spihandle);

}

/*
 * enable SPI clock
 */
void SPIBasic::enableSPICLK(SPI_TypeDef *spireg) {
#if defined SPI1_BASE
	// Enable SPI clock
	if (spireg == SPI1 ) {
		if (!! __HAL_RCC_SPI1_IS_CLK_ENABLED()) {
			__HAL_RCC_SPI1_CLK_ENABLE();
			__HAL_RCC_SPI1_FORCE_RESET();
			__HAL_RCC_SPI1_RELEASE_RESET();
		}
		return;
	}
#endif

#if defined SPI2_BASE
	// Enable SPI clock
	if (spireg == SPI2) {
		if (! __HAL_RCC_SPI2_IS_CLK_ENABLED() ) {
			__HAL_RCC_SPI2_CLK_ENABLE();
			__HAL_RCC_SPI2_FORCE_RESET();
			__HAL_RCC_SPI2_RELEASE_RESET();
		}
		return;
	}
#endif

#if defined SPI3_BASE
	// Enable SPI clock
	if (spireg == SPI3 ) {
		if (! __HAL_RCC_SPI3_IS_CLK_ENABLED() ) {
			__HAL_RCC_SPI3_CLK_ENABLE();
			__HAL_RCC_SPI3_FORCE_RESET();
			__HAL_RCC_SPI3_RELEASE_RESET();
		}
		return;
	}
#endif

#if defined SPI4_BASE
	// Enable SPI clock
	if (spireg == SPI4 ) {
		if (! __HAL_RCC_SPI4_IS_CLK_ENABLED() ) {
			__HAL_RCC_SPI4_CLK_ENABLE();
			__HAL_RCC_SPI4_FORCE_RESET();
			__HAL_RCC_SPI4_RELEASE_RESET();
		}
		return;
	}
#endif

#if defined SPI5_BASE
	// Enable SPI clock
	if (spireg == SPI5 ) {
		if (! __HAL_RCC_SPI5_IS_CLK_ENABLED()) {
			__HAL_RCC_SPI5_CLK_ENABLE();
			__HAL_RCC_SPI5_FORCE_RESET();
			__HAL_RCC_SPI5_RELEASE_RESET();
		}
		return;
	}
#endif

}


uint8_t SPIBasic::transfer(uint8_t data, bool skipReceive) {
	uint8_t r = 0;

	// this is necessarily ugly due to different SPI hardware
	// many mcus use:
	//   TXE (transmit buf empty) flag
	//   RXNE (receive buf no empty) flag
	// and a single data register DR
	// while mcus like stm32 H5/H7 uses TXP / RXP flags and 2 data registers TXDR, RXDR

#if defined(SPI_SR_TXP)
	while( ! (spi_reg->SR & SPI_SR_TXP_Msk) == 0 ); //spinlock
#else
	while( ! (spi_reg->SR & SPI_SR_TXE_Msk) == 0 ); //spinlock
#endif
	//LL_SPI_TransmitData8(spi_reg, data);
#if defined(SPI_DR_DR)
	spi_reg->DR = data;
#else
	spi_reg->TXDR = data;
#endif

	// do we need to timeout? in theory timeout should not happen
#if defined(SPI_SR_RXP)
	while( ! (spi_reg->SR & SPI_SR_RXP_Msk) == 0 ); //spinlock
#else
	while( ! (spi_reg->SR & SPI_SR_RXNE_Msk) == 0 ); //spinlock
#endif

	//r = LL_SPI_ReceiveData8(spi_reg);
	// read the byte regardless of skipReceive to clear the RXNE or RXP flag
#if defined(SPI_DR_DR)
	r = spi_reg->DR & 0xffU;
#else
	r = spi_reg->RXDR & 0xffU;
#endif
	if (skipReceive) r = 0;

	// HAL_SPIBasicResume(&handle);
	return r;
}


void SPIBasic::transfer(void *buf, size_t count, bool skipReceive) {
	transfer((uint8_t *) buf, (uint8_t *) buf, count, skipReceive);
}

/* Expand SPI API
 * https://github.com/arduino/ArduinoCore-API/discussions/189
 *
 * @brief  Transfer several bytes. 2 buffers used to send and receive data.
 *         begin() or beginTransaction() must be called at least once before.
 * @param  tx_buf: pointer to the bytes to send.
 * @param  rx_buf: pointer to the bytes to receive.
 * @param  count: number of bytes to send/receive.
 * @param  skipReceive: skip receiving data after transmit or not.
 *         SPI_TRANSMITRECEIVE or SPI_TRANSMITONLY.
 *         Optional, default: SPI_TRANSMITRECEIVE.
 *
 * note this is abstract, derived class needs to implement it
 */
void SPIBasic::transfer(const void *tx_buf, void *rx_buf, size_t count) {
	transfer((uint8_t *) tx_buf, (uint8_t *) rx_buf, count, false);
}

/* this is the actual implementation of the prior with a skipreceive flag
 * in that mode rx_buf is left untouched */
void SPIBasic::transfer(const uint8_t *tx_buf, uint8_t *rx_buf, size_t count, bool skipReceive) {

	for (size_t i = 0; i < count; i++) {
#if defined(SPI_SR_TXP)
		while( ! (spi_reg->SR & SPI_SR_TXP_Msk) == 0 ); //spinlock
#else
		while (!(spi_reg->SR & SPI_SR_TXE_Msk) == 0); //spinlock
#endif

		//LL_SPI_TransmitData8(spi_reg, data);
#if defined(SPI_DR_DR)
		spi_reg->DR = *(tx_buf+i);
#else
		spi_reg->TXDR = *(tx_buf+i);
#endif

		if (! skipReceive) {
			// do we need to timeout? in theory timeout should not happen
		#if defined(SPI_SR_RXP)
			while( ! (spi_reg->SR & SPI_SR_RXP_Msk) == 0 ); //spinlock
		#else
			while (!(spi_reg->SR & SPI_SR_RXNE_Msk) == 0); //spinlock
		#endif

		// r = LL_SPI_ReceiveData8(spi_reg);
		#if defined(SPI_DR_DR)
			*(rx_buf+i) = spi_reg->DR & 0xffU;
		#else
			*(rx_buf+i) = spi_reg->RXDR & 0xffU;
		#endif
		} // if ! skipReceive

	} // for

	if (skipReceive) {
		/* wait for transmit empty */
#if defined(SPI_SR_TXP)
		while( ! (spi_reg->SR & SPI_SR_TXP_Msk) == 0 ); //spinlock
#else
		while (!(spi_reg->SR & SPI_SR_TXE_Msk) == 0); //spinlock
#endif
		__HAL_SPI_CLEAR_OVRFLAG(&spihandle);
	}

}



/*
 * This function returns the spi peripherial frequency
 */

uint32_t SPIBasic::getClkFreq() {

	uint32_t spi_freq = SystemCoreClock;

	if (spi_reg == NULL) // return sysclk if spi_reg is null
		return SystemCoreClock;

	if (spi_reg != NULL) {
#if defined(STM32C0xx) || defined(STM32F0xx) || defined(STM32G0xx) || \
	    defined(STM32U0xx)
	    /* SPIx source CLK is PCKL1 */
	    spi_freq = HAL_RCC_GetPCLK1Freq();
#else
#if defined(SPI1_BASE)
	    if (spi_reg == SPI1) {
	#if defined(RCC_PERIPHCLK_SPI1) || defined(RCC_PERIPHCLK_SPI123)
		#ifdef RCC_PERIPHCLK_SPI1
	      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI1);
		#else
	      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123);
		#endif
	      if (spi_freq == 0)
	#endif
	      {
	        /* SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2 */
	        spi_freq = HAL_RCC_GetPCLK2Freq();
	      }
	    }
	#endif // SPI1_BASE
#if defined(SPI2_BASE)
	    if (spi_reg == SPI2) {
	#if defined(RCC_PERIPHCLK_SPI2) || defined(RCC_PERIPHCLK_SPI123) ||\
	   	defined(RCC_PERIPHCLK_SPI23)
		#ifdef RCC_PERIPHCLK_SPI2
	    	spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI2);
		#elif defined(RCC_PERIPHCLK_SPI123)
	    	spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123);
		#else
	    	spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI23);
		#endif
	      if (spi_freq == 0)
	#endif
	      {
	        /* SPI_2 and SPI_3. Source CLK is PCKL1 */
	        spi_freq = HAL_RCC_GetPCLK1Freq();
	      }
	    }
#endif // SPI2_BASE

#if defined(SPI3_BASE)
	    if (spi_reg == SPI3) {
	#if defined(RCC_PERIPHCLK_SPI3) || defined(RCC_PERIPHCLK_SPI123) ||\
	    defined(RCC_PERIPHCLK_SPI23)
		#ifdef RCC_PERIPHCLK_SPI3
	      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI3);
		#elif defined(RCC_PERIPHCLK_SPI123)
	      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123);
		#else
	      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI23);
		#endif
	      if (spi_freq == 0)
	#endif
	      {
	        /* SPI_2 and SPI_3. Source CLK is PCKL1 */
	        spi_freq = HAL_RCC_GetPCLK1Freq();
	      }
	    }
#endif // SPI3_BASE

#if defined(SPI4_BASE)
	    if (spi_reg == SPI4) {
    #if defined(RCC_PERIPHCLK_SPI4) || defined(RCC_PERIPHCLK_SPI45)
		#ifdef RCC_PERIPHCLK_SPI4
	      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI4);
		#else
	      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI45);
		#endif
	      if (spi_freq == 0)
    #endif
	      {
	        /* SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2 */
	        spi_freq = HAL_RCC_GetPCLK2Freq();
	      }
	    }
#endif // SPI4_BASE

#if defined(SPI5_BASE)
	    if (spi_reg == SPI5) {
	#if defined(RCC_PERIPHCLK_SPI5) || defined(RCC_PERIPHCLK_SPI45)
		#ifdef RCC_PERIPHCLK_SPI5
	      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI5);
		#else
	      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI45);
		#endif
	      if (spi_freq == 0)
	#endif
	      {
	        /* SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2 */
	        spi_freq = HAL_RCC_GetPCLK2Freq();
	      }
	    }
#endif // SPI5_BASE

#if defined(SPI6_BASE)
	    if (spi_reg == SPI6) {
	#if defined(RCC_PERIPHCLK_SPI6)
	      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI6);
	      if (spi_freq == 0)
	#endif
	      {
	        /* SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2 */
	        spi_freq = HAL_RCC_GetPCLK2Freq();
	      }
	    }
#endif // SPI6_BASE

#if defined(SUBGHZSPI_BASE)
	    if (spi_reg == SUBGHZSPI) {
	      /* Source CLK is APB3 (PCLK3) is derived from AHB3 clock  */
	      spi_freq = HAL_RCC_GetHCLK3Freq();
	    }
	#endif // SUBGHZSPI_BASE
#endif
	}

	return spi_freq;

}


SPIBasic::~SPIBasic() {
	// TODO Auto-generated destructor stub
}
