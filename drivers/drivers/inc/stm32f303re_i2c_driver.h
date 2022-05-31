#ifndef INC_STM32F303RE_I2C_DRIVER_H_
#define INC_STM32F303RE_I2C_DRIVER_H_

#include "stm32f303re.h"

typedef struct {
	uint32_t I2C_SCLSpeed; // @I2C_SCLSpeed
	uint8_t I2C_DeviceAddress;
} I2C_Config_t;

typedef struct {
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState; // @I2C_STATE
	uint8_t DevAddr;
// uint8_t Sr; // repeated start
} I2C_Handle_t;

// @I2C_SCLSpeed
#define I2C_SCL_SPEED_SM				100000U
#define I2C_SCL_SPEED_FM4K				400000U
#define I2C_SCL_SPEED_FMP5K				500000U

// @I2C_AckControl
// CR2 NACK (ONLY IN SLAVE MODE)
#define I2C_ACK_ENABLE					1
#define I2C_ACK_DISABLE					0

// @I2C_STATE
#define I2C_STATE_READY					0
#define I2C_STATE_BUSY_RX				1
#define I2C_STATE_BUSY_TX				2

// Control Repeated start
#define I2C_RS_DISABLE					0
#define I2C_RS_ENABLE 					1

// I2C status flags
#define I2C_FLAG_TXE					(1 << I2C_ISR_TXE)
#define I2C_FLAG_TXIS					(1 << I2C_ISR_TXIS)
#define I2C_FLAG_RXNE					(1 << I2C_ISR_RXNE)
#define I2C_FLAG_TC						(1 << I2C_ISR_TC)
#define I2C_FLAG_ADDR					(1 << I2C_ISR_ADDR)
#define I2C_FLAG_STOPF					(1 << I2C_ISR_STOPF)
#define I2C_FLAG_NACKF					(1 << I2C_ISR_NACKF)
#define I2C_FLAG_OVR					(1 << I2C_ISR_OVR)
#define I2C_FLAG_BERR					(1 << I2C_ISR_BERR)
#define I2C_FLAG_BUSY					(1 << I2C_ISR_BUSY)
#define I2C_FLAG_ARLO					(1 << I2C_ISR_ARLO)
#define I2C_FLAG_TIMEOUT				(1 << I2C_ISR_TIMEOUT)
#define I2C_FLAG_DIR					(1 << I2C_ISR_DIR)

// Callback events
#define I2C_EVENT_TX_COMPLTETE			1
#define I2C_EVENT_RX_COMPLTETE			2
#define I2C_EVENT_NACK_RECIEVED			3
#define I2C_EVENT_ERROR_OVR				4
#define I2C_EVENT_ERROR_TIMEOUT			5
#define I2C_EVENT_ERROR_BERR			6
#define I2C_EVENT_SLAVE_TRANSMIT		7
#define I2C_EVENT_SLAVE_RECIEVE			8
#define I2C_EVENT_SLAVE_ADDR			9 // slave mode addr match

// ------------------------------------------------------------------------
// Peripheral Clock setup
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

// Init and de-init
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

// data send and recieve (blocking)
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
		uint32_t len, uint8_t slaveAddr, uint8_t rs);

void I2C_MasterRecieveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
		uint32_t len, uint8_t slaveAddr, uint8_t rs);

// Data send and recieve (interrupt)
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
		uint32_t len, uint8_t slaveAddr);

uint8_t I2C_MasterRecieveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
		uint32_t len, uint8_t slaveAddr);

// IRQ Configuration and ISR handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

// Other peripheral control APIs
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

// Get flag
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

// Application callback
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,
		uint8_t AppEv);

void I2C_IRQSlaveConfig(I2C_Handle_t *pI2CHandle, uint8_t EnOrDi);
void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t value);
uint8_t I2C_SlaveRecieveData(I2C_RegDef_t *pI2C);

#endif /* INC_STM32F303RE_I2C_DRIVER_H_ */
