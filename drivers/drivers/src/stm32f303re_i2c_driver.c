#include "stm32f303re_i2c_driver.h"

// Helper functions
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ConfigureAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t addr);
static void I2C_ConfigureAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t addr);

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	} else {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}
}

void I2C_Init(I2C_Handle_t *pI2CHandle) {
	// enable clock for peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// uint32_t i2c_timingr_tmp = 0;

	// HSI as clock source for I2C
	switch (pI2CHandle->I2C_Config.I2C_SCLSpeed) {
	case I2C_SCL_SPEED_SM:
		pI2CHandle->pI2Cx->TIMINGR |= (1 << I2C_TIMINGR_PRESC);
		pI2CHandle->pI2Cx->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL);
		pI2CHandle->pI2Cx->TIMINGR |= (0xf << I2C_TIMINGR_SCLH);
		pI2CHandle->pI2Cx->TIMINGR |= (0x2 << I2C_TIMINGR_SDADEL);
		pI2CHandle->pI2Cx->TIMINGR |= (0x4 << I2C_TIMINGR_SCLDEL);
		break;

	case I2C_SCL_SPEED_FM4K:
		pI2CHandle->pI2Cx->TIMINGR |= (0 << I2C_TIMINGR_PRESC);
		pI2CHandle->pI2Cx->TIMINGR |= (0x9 << I2C_TIMINGR_SCLL);
		pI2CHandle->pI2Cx->TIMINGR |= (0x3 << I2C_TIMINGR_SCLH);
		pI2CHandle->pI2Cx->TIMINGR |= (0x1 << I2C_TIMINGR_SDADEL);
		pI2CHandle->pI2Cx->TIMINGR |= (0x3 << I2C_TIMINGR_SCLDEL);
		break;

	case I2C_SCL_SPEED_FMP5K:
		pI2CHandle->pI2Cx->TIMINGR |= (0 << I2C_TIMINGR_PRESC);
		pI2CHandle->pI2Cx->TIMINGR |= (0x6 << I2C_TIMINGR_SCLL);
		pI2CHandle->pI2Cx->TIMINGR |= (0x3 << I2C_TIMINGR_SCLH);
		pI2CHandle->pI2Cx->TIMINGR |= (0x0 << I2C_TIMINGR_SDADEL);
		pI2CHandle->pI2Cx->TIMINGR |= (0x1 << I2C_TIMINGR_SCLDEL);
		break;
	}

	// slave addr
	pI2CHandle->pI2Cx->OAR1 |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	// enable slave addr
	pI2CHandle->pI2Cx->OAR1 |= (1 << 15);
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
		uint32_t len, uint8_t slaveAddr, uint8_t rs) {
	// set how many bytes to transfer
	pI2CHandle->pI2Cx->CR2 |= (len << I2C_CR2_NBYTES);
	I2C_ConfigureAddressPhaseWrite(pI2CHandle->pI2Cx, slaveAddr);

	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	while (len > 0) {
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
			;
		pI2CHandle->pI2Cx->TXDR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
		;
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TC))
		;

	if (rs == I2C_RS_DISABLE) {
		// Generate STOP after current byte - no repeated start
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}

}

void I2C_MasterRecieveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
		uint32_t len, uint8_t slaveAddr, uint8_t rs) {
	// set how many bytes to transfer
	pI2CHandle->pI2Cx->CR2 |= (len << I2C_CR2_NBYTES);
	I2C_ConfigureAddressPhaseRead(pI2CHandle->pI2Cx, slaveAddr);

	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	while (len > 0) {
		// wait until data is ready to read
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
			;
		*pRxBuffer = pI2CHandle->pI2Cx->RXDR;

		/*
		 if(len == 1){
		 // Generate STOP after current byte
		 I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		 }
		 */

		pRxBuffer++;
		len--;
	}
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TC))
		;
	if (rs == I2C_RS_DISABLE) {
		// Generate STOP after current byte
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

static void I2C_EnableErrorIT(I2C_Handle_t *pI2CHandle) {
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ERRIE);
}

static void I2C_DisableErrorIT(I2C_Handle_t *pI2CHandle) {
	pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ERRIE);
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
		uint32_t len, uint8_t slaveAddr) {
	uint8_t state = pI2CHandle->TxRxState;
	if (state == I2C_STATE_READY) {
		pI2CHandle->DevAddr = slaveAddr;
		pI2CHandle->TxLen = len;
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxRxState = I2C_STATE_BUSY_TX;

		pI2CHandle->pI2Cx->CR2 &= ~(0xff << I2C_CR2_NBYTES);
		pI2CHandle->pI2Cx->CR2 |= (len << I2C_CR2_NBYTES);

		// autoend
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_AUTOEND);

		I2C_ConfigureAddressPhaseWrite(pI2CHandle->pI2Cx, slaveAddr);

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// interrupt for tx buffer empty
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_TXIE);

		// interrupt for transfer complete
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOPIE);

		// interrupt for nack
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_NACKIE);

		I2C_EnableErrorIT(pI2CHandle);

	}
	return state;
}

uint8_t I2C_MasterRecieveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
		uint32_t len, uint8_t slaveAddr) {
	uint8_t state = pI2CHandle->TxRxState;
	if (state == I2C_STATE_READY) {
		pI2CHandle->DevAddr = slaveAddr;
		pI2CHandle->RxLen = len;
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->TxRxState = I2C_STATE_BUSY_RX;

		// autoend
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_AUTOEND);

		pI2CHandle->pI2Cx->CR2 &= ~(0xff << I2C_CR2_NBYTES);
		pI2CHandle->pI2Cx->CR2 |= (len << I2C_CR2_NBYTES);
		I2C_ConfigureAddressPhaseRead(pI2CHandle->pI2Cx, slaveAddr);

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// interrupt for rx buffer fill
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_RXIE);

		// interrupt for transfer complete
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOPIE);

		// interrupt for nack
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_NACKIE);

		I2C_EnableErrorIT(pI2CHandle);

	}
	return state;
}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	// find ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	// only 4 upper bits of priority section are implemented
	uint8_t shift_ammount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_ammount);
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {
	if (pI2Cx->ISR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_IRQSlaveConfig(I2C_Handle_t *pI2CHandle, uint8_t EnOrDi) {

	if (EnOrDi == ENABLE) {
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ADDRIE);
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_RXIE);
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_TXIE);
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOPIE);
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_NACKIE);
		I2C_EnableErrorIT(pI2CHandle);
	} else {
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ADDRIE);
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_RXIE);
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_TXIE);
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_STOPIE);
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_NACKIE);
		I2C_DisableErrorIT(pI2CHandle);
	}
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t value) {
	pI2C->TXDR = value;
}

uint8_t I2C_SlaveRecieveData(I2C_RegDef_t *pI2C) {
	return (uint8_t) pI2C->RXDR;
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {
	// Interrupt handling for both master and slave mode
	uint32_t temp1, temp2;

	// ADDR event (slave only)
	temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_ADDRIE);
	temp2 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR);
	if (temp1 && temp2) {
		// clear addr flag
		pI2CHandle->pI2Cx->ICR |= (1 << I2C_ICR_ADDRCF);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_SLAVE_ADDR);
	}

	// Nack interrupt
	temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_NACKIE);
	temp2 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_NACKF);
	if (temp1 && temp2) {
		// clear nack flag
		pI2CHandle->pI2Cx->ICR |= (1 << I2C_ICR_NACKCF);
		I2C_ApplicationEventCallback(pI2CHandle,
		I2C_EVENT_NACK_RECIEVED);
	}

	// Stop detected
	temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_STOPIE);
	temp2 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_STOPF);
	if (temp1 && temp2) {
		// will be triggered in both master and slave mode

		// clear stop flag
		pI2CHandle->pI2Cx->ICR |= (1 << I2C_ICR_STOPCF);

		// master mode
		if (pI2CHandle->pI2Cx->OAR1 & (1 << I2C_OAR1_OA1EN)) {
			// Transmitter
			if (pI2CHandle->TxRxState == I2C_STATE_BUSY_TX
					&& pI2CHandle->TxLen == 0) {

				// reset members of handle structure
				pI2CHandle->TxRxState = I2C_STATE_READY;
				pI2CHandle->pTxBuffer = NULL;
				pI2CHandle->TxLen = 0;

				// disable interrupts
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_TXIE);
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_STOPIE);
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_NACKIE);
				I2C_DisableErrorIT(pI2CHandle);

				// notify application after transmission complete
				I2C_ApplicationEventCallback(pI2CHandle,
				I2C_EVENT_TX_COMPLTETE);

			}

			// Reciever
			else if (pI2CHandle->TxRxState == I2C_STATE_BUSY_RX
					&& pI2CHandle->RxLen == 0) {
				// reset members of handle structure
				pI2CHandle->TxRxState = I2C_STATE_READY;
				pI2CHandle->pRxBuffer = NULL;
				pI2CHandle->RxLen = 0;

				// disable interrupts
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_RXIE);
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_STOPIE);
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_NACKIE);
				I2C_DisableErrorIT(pI2CHandle);

				I2C_ApplicationEventCallback(pI2CHandle,
				I2C_EVENT_RX_COMPLTETE);
			}
		}
	}

	// Transmit interrupt
	temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_TXIE);
	temp2 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXIS);
	if (temp1 && temp2) {

		// slave mode
		if (pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_ADDRIE)) {
			// check if transmitter
			if (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_DIR)) {
				// slave in trasmitter mode
				I2C_ApplicationEventCallback(pI2CHandle,
				I2C_EVENT_SLAVE_TRANSMIT);
			}
		}
		// master mode
		else {
			if (pI2CHandle->TxRxState == I2C_STATE_BUSY_TX) {
				if (pI2CHandle->TxLen > 0) {
					*((volatile uint8_t*) &pI2CHandle->pI2Cx->TXDR) =
							*(pI2CHandle->pTxBuffer);
					pI2CHandle->TxLen--;
					pI2CHandle->pTxBuffer++;
				}
			}
		}
	}

	// Recieve interrupt
	temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_RXIE);
	temp2 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE);
	if (temp1 && temp2) {
		if (pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_ADDRIE)) {
			// check if reciever
			if (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_DIR)) {
				// slave in reciever mode
				I2C_ApplicationEventCallback(pI2CHandle,
				I2C_EVENT_SLAVE_RECIEVE);
			}
		}

		else {
			// master mode
			if (pI2CHandle->TxRxState == I2C_STATE_BUSY_RX) {

				if (pI2CHandle->RxLen > 0) {
					uint8_t data = (uint8_t) pI2CHandle->pI2Cx->RXDR;
					*pI2CHandle->pRxBuffer = data;
					pI2CHandle->pRxBuffer++;
					pI2CHandle->RxLen--;
				}
			}
		}
	}
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle) {
	uint8_t temp1, temp2;
	temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_ERRIE);

	// Bus error
	temp2 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BERR);
	if (temp1 && temp2) {
		pI2CHandle->pI2Cx->ICR |= (1 << I2C_ICR_BERRCF);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_ERROR_BERR);
	}

	// Overrun/Underrun error
	temp2 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_OVR);
	if (temp1 && temp2) {
		pI2CHandle->pI2Cx->ICR |= (1 << I2C_ICR_OVRCF);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_ERROR_OVR);
	}

	// Timeout error
	temp2 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TIMEOUT);
	if (temp1 && temp2) {
		pI2CHandle->pI2Cx->ICR |= (1 << I2C_ICR_TIMOUTCF);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_ERROR_TIMEOUT);
	}
}

__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,
		uint8_t AppEv) {

}

// must be called before START
static void I2C_ConfigureAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t addr) {
	// in 7 bit addr mode, addr is shifted by 1 in SADD
	pI2Cx->CR2 |= (addr << (I2C_CR2_SADD + 1));
	pI2Cx->CR2 &= ~(1 << I2C_CR2_RD_WRN); // master requests write
}

// must be called before START
static void I2C_ConfigureAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t addr) {
	// in 7 bit addr mode, addr is shifted by 1 in SADD
	pI2Cx->CR2 |= (addr << (I2C_CR2_SADD + 1));
	pI2Cx->CR2 |= (1 << I2C_CR2_RD_WRN); // master requests read
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR2 |= (1 << I2C_CR2_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR2 |= (1 << I2C_CR2_STOP);
}
