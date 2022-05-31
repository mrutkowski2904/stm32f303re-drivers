#ifndef INC_STM32F303RE_SPI_DRIVER_H_
#define INC_STM32F303RE_SPI_DRIVER_H_

#include "stm32f303re.h"

typedef struct {
	uint8_t SPI_DeviceMode; 	// @SPI_DeviceMode
	uint8_t SPI_BusConfig; 		// @SPI_BusConfig
	uint8_t SPI_SclkSpeed;		// @SPI_SclkSpeed
	uint8_t SPI_DS;				// @SPI_DS
	uint8_t SPI_CPOL;			// @SPI_CPOL
	uint8_t SPI_CPHA;			// @SPI_CPHA
	uint8_t SPI_SSM;			// @SPI_SSM
} SPI_Config_t;

typedef struct {
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;

	// for interrupt based usage:
	uint8_t *pTxBuffer; // application tx buffer addr
	uint8_t *pRxBuffer; // application rx buffer addr
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;

} SPI_Handle_t;

// SPI Application states
#define SPI_READY							0
#define SPI_BUSY_IN_TX						1
#define SPI_BUSY_IN_RX						2

// SPI IT Events
#define SPI_EVENT_TX_CMPLT					1
#define SPI_EVENT_RX_CMPLT					2
#define SPI_EVENT_OVR_ERR					3

// @SPI_DeviceMode
#define SPI_DEVICE_MODE_SLAVE				0
#define SPI_DEVICE_MODE_MASTER				1

// @SPI_BusConfig
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

// @SPI_SclkSpeed
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

// @SPI_DS
#define SPI_DS_8BITS						0x7
#define SPI_DS_16BITS						0xf

// @SPI_CPOL
#define SPI_CPOL_LOW						0
#define SPI_CPOL_HIGH						1

// @SPI_CPHA
#define SPI_CPHA_LOW						0
#define SPI_CPHA_HIGH						1

// @SPI_SSM
#define SPI_SSM_DI							0
#define SPI_SSM_EN							1

// ------------------------------------------------------------------------
// Peripheral Clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

// Init and de-init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// Data Send and Recieve
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer,
		uint32_t len);
uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer,
		uint32_t len);

// IRQ Configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

// Other peripheral control APIs
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pHandle);
void SPI_CloseReception(SPI_Handle_t *pHandle);

// Get flag
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

// Application callback
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F303RE_SPI_DRIVER_H_ */
