#ifndef INC_STM32F303RE_GPIO_DRIVER_H_
#define INC_STM32F303RE_GPIO_DRIVER_H_

#include "stm32f303re.h"

typedef struct {
	uint8_t GPIO_PinNumber;			// Possible values @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;			// Possible values @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			// Possible values @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;	// Possible values @GPIO_PIN_PUPD_CONTROL
	uint8_t GPIO_PinOPType;			// Possible values @GPIO_PIN_OUTPUT_TYPES
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct {
	GPIO_RegDef_t *pGPIOx; // base addr of GPIO Port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; // holds gpio pin configuration settings
} GPIO_Handle_t;

// @GPIO_PIN_NUMBERS
// GPIO Pin numbers macros
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

// @GPIO_PIN_MODES
// GPIO Pin mode macros
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALT		2
#define GPIO_MODE_ANALOG	3
// custom codes for interrupt
#define GPIO_MODE_IT_FT		4 // faling edge interrupt
#define GPIO_MODE_IT_RT		5 // rising edge interrupt
#define GPIO_MODE_IT_RFT	6 // faling and rising edge interrupt

// @GPIO_PIN_OUTPUT_TYPES
// GPIO Pin output types
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

// @GPIO_PIN_SPEED
// GPIO Pin possible speeds (3 only in this MCU)
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED		1
#define GPIO_SPEED_HIGH		3

// @GPIO_PIN_PUPD_CONTROL
// GPIO Pin pull & up pull down configuration macros
#define GPIO_PIN_NO_PUPD	0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

// APIs supported by this driver
// Peripheral Clock Setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

// Init & Deinit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// RW
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// IRQ Configuration and ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F303RE_GPIO_DRIVER_H_ */
