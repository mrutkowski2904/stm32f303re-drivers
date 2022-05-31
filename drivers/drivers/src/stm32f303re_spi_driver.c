#include "stm32f303re_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pHandle);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}
	}
}

void SPI_Init(SPI_Handle_t *pSPIHandle) {
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// configure SPI_CR
	uint32_t spix_cr1_tmp = 0;
	uint32_t spix_cr2_tmp = 0x0700;

	// device mode
	spix_cr1_tmp |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	// bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		// bidi mode should be cleared
		spix_cr1_tmp &= ~(1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		// bidi mode should be enabled
		spix_cr1_tmp |= (1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig
			== SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		// bidi mode should be cleared
		spix_cr1_tmp &= ~(1 << SPI_CR1_BIDIMODE);
		// RXONLY bit must be set
		spix_cr1_tmp |= (1 << SPI_CR1_RXONLY);
	}

	// configure spi clock speed
	spix_cr1_tmp |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	// configure ds
	spix_cr2_tmp |= (pSPIHandle->SPIConfig.SPI_DS << SPI_CR2_DS);

	// configure cpol
	spix_cr1_tmp |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// configure cpha
	spix_cr1_tmp |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	// configure ssm
	spix_cr1_tmp |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 = spix_cr1_tmp;
	pSPIHandle->pSPIx->CR2 = spix_cr2_tmp;
}

// Blocking
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {
	while (len > 0) {
		// wait until TXE is not set - until buffer is not empty
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET)
			;

		// check data size
		uint32_t cr2_tmp_ds = pSPIx->CR2;
		cr2_tmp_ds = cr2_tmp_ds & (0xf << SPI_CR2_DS); // only keep data size bits
		cr2_tmp_ds = (cr2_tmp_ds >> SPI_CR2_DS); // move data size bits to the 0th bit
		if (cr2_tmp_ds == SPI_DS_8BITS) {
			// load data into DR
			*((volatile uint8_t*)&pSPIx->DR) = *pTxBuffer;
			len--;
			pTxBuffer++; // move by byte
		} else if (cr2_tmp_ds == SPI_DS_16BITS) {
			// load data into DR
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			len -= 2;
			(uint16_t*) pTxBuffer++; // move by two bytes
		}

	}

}

// Blocking
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len) {
	while (len > 0) {
		// wait until RXNE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET)
			;

		// check data size
		uint32_t cr2_tmp_ds = pSPIx->CR2;
		cr2_tmp_ds = cr2_tmp_ds & (0xf << SPI_CR2_DS); // only keep data size bits
		cr2_tmp_ds = (cr2_tmp_ds >> SPI_CR2_DS); // move data size bits to the 0th bit
		if (cr2_tmp_ds == SPI_DS_8BITS) {
			// load data from DR
			*pRxBuffer = pSPIx->DR;
			len--;
			pRxBuffer++; // move by byte
		} else if (cr2_tmp_ds == SPI_DS_16BITS) {
			// load data from DR
			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			len -= 2;
			(uint16_t*) pRxBuffer++; // move by two bytes
		}

	}
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			// ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) { // 32 to 63
		// ISER1
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// ISER2
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	} else {
		if (IRQNumber <= 31) {
			// ICER0
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) { // 32 to 63
		// ICER1
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// ICER2
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	// find ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	// only 4 upper bits of priority section are implemented
	uint8_t shift_ammount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_ammount);
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer,
		uint32_t len) {
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX) {
		// Save Tx buffer address and len information in global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		// Mark SPI State as busy so no other code can use it
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// Enable TXEIE Bit to enable interrupt when TXE is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return state;
}

uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer,
		uint32_t len) {
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX) {
		// Save Rx buffer address and len information in global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;

		// Mark SPI State as busy so no other code can use it
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// Enable RXNEIE Bit to enable interrupt when data is recieved
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	return state;
}

void SPI_IRQHandling(SPI_Handle_t *pHandle) {
	uint8_t temp1, temp2;
	// check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	if (temp1 && temp2) {
		// handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if (temp1 && temp2) {
		// handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for OVR
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if (temp1 && temp2) {
		// handle OVR
		spi_ovr_interrupt_handle(pHandle);
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if (EnOrDi) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if (EnOrDi) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

// helper functions
static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle) {
	// check data size
	uint32_t cr2_tmp_ds = pHandle->pSPIx->CR2;
	cr2_tmp_ds = cr2_tmp_ds & (0xf << SPI_CR2_DS); // only keep data size bits
	cr2_tmp_ds = (cr2_tmp_ds >> SPI_CR2_DS); // move data size bits to the 0th bit

	if (cr2_tmp_ds == SPI_DS_8BITS) {
		// load data into DR
		*((volatile uint8_t*)&pHandle->pSPIx->DR) = *(pHandle->pTxBuffer);
		pHandle->TxLen--;
		pHandle->pTxBuffer++; // move by byte
	} else if (cr2_tmp_ds == SPI_DS_16BITS) {
		// load data into DR
		pHandle->pSPIx->DR = *((uint16_t*) pHandle->pTxBuffer);
		pHandle->TxLen -= 2;
		(uint16_t*) pHandle->pTxBuffer++; // move by two bytes
	}

	if (!pHandle->TxLen) {
		// len = 0 all data transmitted
		SPI_CloseTransmission(pHandle);
		SPI_ApplicationEventCallback(pHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle) {
	// check data size
	uint32_t cr2_tmp_ds = pHandle->pSPIx->CR2;
	cr2_tmp_ds = cr2_tmp_ds & (0xf << SPI_CR2_DS); // only keep data size bits
	cr2_tmp_ds = (cr2_tmp_ds >> SPI_CR2_DS); // move data size bits to the 0th bit

	if (cr2_tmp_ds == SPI_DS_8BITS) {
		// load data from DR
		*(pHandle->pRxBuffer) = (uint8_t) pHandle->pSPIx->DR;
		pHandle->RxLen--;
		pHandle->pRxBuffer++; // move by byte
	} else if (cr2_tmp_ds == SPI_DS_16BITS) {
		// load data from DR
		*((uint16_t*) pHandle->pTxBuffer) = (uint16_t) pHandle->pSPIx->DR;
		pHandle->RxLen -= 2;
		(uint16_t*) pHandle->pRxBuffer++; // move by two bytes
	}

	if (!pHandle->RxLen) {
		// len = 0 all data recieved
		// disable interrupt triggering for Rx ready
		SPI_CloseReception(pHandle);
		SPI_ApplicationEventCallback(pHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_interrupt_handle(SPI_Handle_t *pHandle) {
	// clear ovr flag
	uint8_t tmp;
	if (pHandle->TxState != SPI_BUSY_IN_TX) {
		tmp = pHandle->pSPIx->DR; // cleared by reading DR and SR
		tmp = pHandle->pSPIx->SR;
	}
	(void) tmp; // to hide unused variable warning
	// inform the application
	SPI_ApplicationEventCallback(pHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pHandle) {
	pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pHandle->pTxBuffer = NULL;
	pHandle->TxLen = 0;
	pHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pHandle) {
	pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pHandle->pRxBuffer = NULL;
	pHandle->RxLen = 0;
	pHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
	uint8_t tmp;
	tmp = pSPIx->DR; // cleared by reading DR and SR
	tmp = pSPIx->SR;
	(void) tmp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,
		uint8_t AppEv) {
}
