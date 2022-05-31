#ifndef INC_STM32F303RE_H_
#define INC_STM32F303RE_H_

#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

// Processor specific details
#define NO_PR_BITS_IMPLEMENTED		4
// NVIC Interrupt enable registers
#define NVIC_ISER0 					((__vo uint32_t*)0xE000E100UL)
#define NVIC_ISER1 					((__vo uint32_t*)0xE000E104UL)
#define NVIC_ISER2 					((__vo uint32_t*)0xE000E108UL)
#define NVIC_ISER3 					((__vo uint32_t*)0xE000E10CUL)
// NVIC Interrupt disable registers
#define NVIC_ICER0 					((__vo uint32_t*)0XE000E180UL)
#define NVIC_ICER1 					((__vo uint32_t*)0XE000E184UL)
#define NVIC_ICER2 					((__vo uint32_t*)0XE000E188UL)
#define NVIC_ICER3 					((__vo uint32_t*)0XE000E18CUL)
// NVIC Interrupt priority registers
#define NVIC_PR_BASE_ADDR			((__vo uint32_t*)0xE000E400UL)

#define FLASH_BASE_ADDR				0x08000000UL // 512 KB
#define SRAM_BASE_ADDR				0x20000000UL // 64 KB
#define CCMRAM_BASE_ADDR			0x10000000UL // 16 KB

#define PERIPH_BASE_ADDR			0x40000000UL
#define APB1_BASE_ADDR				PERIPH_BASE_ADDR
#define APB2_BASE_ADDR				(PERIPH_BASE_ADDR + 0x10000UL)
#define AHB1_BASE_ADDR				(PERIPH_BASE_ADDR + 0x20000UL)
#define AHB2_BASE_ADDR				(PERIPH_BASE_ADDR + 0x8000000UL)
#define AHB3_BASE_ADDR				(PERIPH_BASE_ADDR + 0x10000000UL)

// AHB1 Peripherals
#define RCC_BASE_ADDR				(AHB1_BASE_ADDR + 0x1000U)

// AHB2 Peripherals
#define GPIOA_BASE_ADDR				(AHB2_BASE_ADDR + 0x0000U)
#define GPIOB_BASE_ADDR				(AHB2_BASE_ADDR + 0x0400U)
#define GPIOC_BASE_ADDR				(AHB2_BASE_ADDR + 0x0800U)
#define GPIOD_BASE_ADDR				(AHB2_BASE_ADDR + 0x0C00U)
#define GPIOE_BASE_ADDR				(AHB2_BASE_ADDR + 0x1000U)
#define GPIOF_BASE_ADDR				(AHB2_BASE_ADDR + 0x1400U)

// AHB3 Peripherals
// not used

// APB1 Peripherals
#define I2C1_BASE_ADDR				(APB1_BASE_ADDR + 0x5400U)
#define I2C2_BASE_ADDR				(APB1_BASE_ADDR + 0x5800U)
#define I2C3_BASE_ADDR				(APB1_BASE_ADDR + 0x7800U)
#define USART2_BASE_ADDR			(APB1_BASE_ADDR + 0x4400U)
#define USART3_BASE_ADDR			(APB1_BASE_ADDR + 0x4800U)
#define UART4_BASE_ADDR				(APB1_BASE_ADDR + 0x4C00U)
#define UART5_BASE_ADDR				(APB1_BASE_ADDR + 0x5000U)
#define SPI2_BASE_ADDR				(APB1_BASE_ADDR + 0x3800U)
#define SPI3_BASE_ADDR				(APB1_BASE_ADDR + 0x3C00U)

// APB2 Peripherals
#define EXTI_BASE_ADDR				(APB2_BASE_ADDR + 0x0400U)
#define SPI1_BASE_ADDR				(APB2_BASE_ADDR + 0x3000U)
#define USART1_BASE_ADDR			(APB2_BASE_ADDR + 0x3800U)
#define SPI1_BASE_ADDR				(APB2_BASE_ADDR + 0x3000U)
#define SYSCFG_BASE_ADDR			(APB2_BASE_ADDR + 0x0000U)

/********************* Peripheral register definition structures *********************/

typedef struct {
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFRL;
	__vo uint32_t AFRH;
	__vo uint32_t BRR;
} GPIO_RegDef_t;

typedef struct {
	__vo uint32_t CR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t APB1RSTR;
	__vo uint32_t AHBENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t APB1ENR;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t AHBRSTR;
	__vo uint32_t CFGR2;
	__vo uint32_t CFGR3;
} RCC_RegDef_t;

typedef struct {
	__vo uint32_t IMR1;
	__vo uint32_t EMR1;
	__vo uint32_t RTSR1;
	__vo uint32_t FTSR1;
	__vo uint32_t SWIER1;
	__vo uint32_t PR1;
	__vo uint32_t IMR2;
	__vo uint32_t EMR2;
	__vo uint32_t RTSR2;
	__vo uint32_t FTSR2;
	__vo uint32_t SWIER2;
	__vo uint32_t PR2;
} EXTI_RegDef_t;

typedef struct {
	__vo uint32_t CFGR1;
	__vo uint32_t RCR;
	uint32_t EXTICR[4];
	__vo uint32_t CFGR2;
	uint32_t BLANK[14];
	__vo uint32_t CFGR3;
	__vo uint32_t CFGR4;
} SYSCFG_RegDef_t;

typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
} SPI_RegDef_t;

typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t TIMINGR;
	__vo uint32_t TIMEOUTR;
	__vo uint32_t ISR;
	__vo uint32_t ICR;
	__vo uint32_t PECR;
	__vo uint32_t RXDR;
	__vo uint32_t TXDR;
} I2C_RegDef_t;

typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t BRR;
	__vo uint32_t GTPR;
	__vo uint32_t RTOR;
	__vo uint32_t RQR;
	__vo uint32_t ISR;
	__vo uint32_t ICR;
	__vo uint32_t RDR;
	__vo uint32_t TDR;
} USART_RegDef_t;

// Peripheral definitions
#define GPIOA 						((GPIO_RegDef_t*) GPIOA_BASE_ADDR)
#define GPIOB 						((GPIO_RegDef_t*) GPIOB_BASE_ADDR)
#define GPIOC 						((GPIO_RegDef_t*) GPIOC_BASE_ADDR)
#define GPIOD 						((GPIO_RegDef_t*) GPIOD_BASE_ADDR)
#define GPIOE 						((GPIO_RegDef_t*) GPIOE_BASE_ADDR)
#define GPIOF 						((GPIO_RegDef_t*) GPIOF_BASE_ADDR)

#define SPI1						((SPI_RegDef_t*) SPI1_BASE_ADDR)
#define SPI2						((SPI_RegDef_t*) SPI2_BASE_ADDR)
#define SPI3						((SPI_RegDef_t*) SPI3_BASE_ADDR)

#define I2C1						((I2C_RegDef_t*) I2C1_BASE_ADDR)
#define I2C2						((I2C_RegDef_t*) I2C2_BASE_ADDR)
#define I2C3						((I2C_RegDef_t*) I2C3_BASE_ADDR)

#define USART1						((USART_RegDef_t*) USART1_BASE_ADDR)
#define USART2						((USART_RegDef_t*) USART2_BASE_ADDR)
#define USART3						((USART_RegDef_t*) USART3_BASE_ADDR)
#define UART4						((USART_RegDef_t*) UART4_BASE_ADDR)
#define UART5						((USART_RegDef_t*) UART5_BASE_ADDR)

#define RCC							((RCC_RegDef_t*) RCC_BASE_ADDR)
#define EXTI						((EXTI_RegDef_t*) EXTI_BASE_ADDR)
#define SYSCFG						((SYSCFG_RegDef_t*) SYSCFG_BASE_ADDR)

// Clock Enable Macro - GPIOx peripheral
#define GPIOA_PCLK_EN()				(RCC->AHBENR |= (1<<17))
#define GPIOB_PCLK_EN()				(RCC->AHBENR |= (1<<18))
#define GPIOC_PCLK_EN()				(RCC->AHBENR |= (1<<19))
#define GPIOD_PCLK_EN()				(RCC->AHBENR |= (1<<20))
#define GPIOE_PCLK_EN()				(RCC->AHBENR |= (1<<21))
#define GPIOF_PCLK_EN()				(RCC->AHBENR |= (1<<22))

// Clock Enable Macro - I2Cx peripheral
#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1<<30))

// Clock Enable Macro - SPIx peripheral
#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()				(RCC->APB2ENR |= (1<<15))

// Clock Enable Macro - USARTx peripheral
#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1<<14))
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()				(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()				(RCC->APB1ENR |= (1<<20))

// Clock Enable Macro - SYSCFGx peripheral
#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1<<0))

// Clock Disable Macro - GPIOx peripheral
#define GPIOA_PCLK_DI()				(RCC->AHBENR &= ~(1<<17))
#define GPIOB_PCLK_DI()				(RCC->AHBENR &= ~(1<<18))
#define GPIOC_PCLK_DI()				(RCC->AHBENR &= ~(1<<19))
#define GPIOD_PCLK_DI()				(RCC->AHBENR &= ~(1<<20))
#define GPIOE_PCLK_DI()				(RCC->AHBENR &= ~(1<<21))
#define GPIOF_PCLK_DI()				(RCC->AHBENR &= ~(1<<22))

// Clock Disable Macro - I2Cx peripheral
#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1<<30))

// Clock Disable Macro - SPIx peripheral
#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()				(RCC->APB2ENR &= ~(1<<15))

// Clock Disable Macro - USARTx peripheral
#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1<<14))
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()				(RCC->APB1ENR &= ~(1<<20))

// Clock Disable Macro - SYSCFGx peripheral
#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~(1<<0))

// GPIOx Reset Macros
#define GPIOA_REG_RESET()			do{ (RCC->AHBRSTR |= (1<<17)); (RCC->AHBRSTR &= ~(1<<17)); }while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHBRSTR |= (1<<18)); (RCC->AHBRSTR &= ~(1<<18)); }while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHBRSTR |= (1<<19)); (RCC->AHBRSTR &= ~(1<<19)); }while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHBRSTR |= (1<<20)); (RCC->AHBRSTR &= ~(1<<20)); }while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHBRSTR |= (1<<21)); (RCC->AHBRSTR &= ~(1<<21)); }while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHBRSTR |= (1<<22)); (RCC->AHBRSTR &= ~(1<<22)); }while(0)

// IRQ Numbers
// EXTI lines
#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_10			40
// SPI
#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI2					36
#define IRQ_NO_SPI3					51
// I2C
#define IRQ_NO_I2C1_EV				31
#define IRQ_NO_I2C1_ER				32
#define IRQ_NO_I2C2_EV				33
#define IRQ_NO_I2C2_ER				34
#define IRQ_NO_I2C3_EV				72
#define IRQ_NO_I2C3_ER				73

// IRQ Priorities
#define NVIC_IRQ_PRIO0				0
#define NVIC_IRQ_PRIO1				1
#define NVIC_IRQ_PRIO2				2
#define NVIC_IRQ_PRIO3				3
#define NVIC_IRQ_PRIO4				4
#define NVIC_IRQ_PRIO5				5
#define NVIC_IRQ_PRIO6				6
#define NVIC_IRQ_PRIO7				7
#define NVIC_IRQ_PRIO8				8
#define NVIC_IRQ_PRIO9				9
#define NVIC_IRQ_PRIO10				10
#define NVIC_IRQ_PRIO11				11
#define NVIC_IRQ_PRIO12				12
#define NVIC_IRQ_PRIO13				13
#define NVIC_IRQ_PRIO14				14
#define NVIC_IRQ_PRIO15				15

// Generic macros
#define ENABLE 						1
#define DISABLE 					0
#define SET 						ENABLE
#define RESET 						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET
#define FLAG_SET					SET
#define FLAG_RESET					RESET

#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 : 0)

// Bit position definitions of SPI peripheral
#define SPI_CR1_CPHA				0
#define SPI_CR1_CPOL				1
#define SPI_CR1_MSTR				2
#define SPI_CR1_BR					3
#define SPI_CR1_SPE					6
#define SPI_CR1_LSBFIRST			7
#define SPI_CR1_SSI					8
#define SPI_CR1_SSM					9
#define SPI_CR1_RXONLY				10
#define SPI_CR1_CRCL				11
#define SPI_CR1_CRCNEXT				12
#define SPI_CR1_CRCEN				13
#define SPI_CR1_BIDIOE				14
#define SPI_CR1_BIDIMODE			15

#define SPI_CR2_RXDMAEN				0
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_NSSP				3
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE				7
#define SPI_CR2_DS					8
#define SPI_CR2_FRXTH				12
#define SPI_CR2_LDMA_RX				13
#define SPI_CR2_LDMA_TX				14

#define SPI_SR_RXNE					0
#define SPI_SR_TXE					1
#define SPI_SR_CHSIDE				2
#define SPI_SR_UDR					3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF					5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8
#define SPI_SR_FRLV					9
#define SPI_SR_FTLVL				11

// Flags
#define SPI_TXE_FLAG				( 1 << SPI_SR_TXE )
#define SPI_RXNE_FLAG				( 1 << SPI_SR_RXNE )
#define SPI_BUSY_FLAG				( 1 << SPI_SR_BSY )

// I2C registers bit positions
#define I2C_CR1_PE					0
#define I2C_CR1_TXIE				1
#define I2C_CR1_RXIE				2
#define I2C_CR1_ADDRIE				3
#define I2C_CR1_NACKIE				4
#define I2C_CR1_STOPIE				5
#define I2C_CR1_TCIE				6
#define I2C_CR1_ERRIE				7
#define I2C_CR1_DNF					8
#define I2C_CR1_ANFOFF				12
#define I2C_CR1_TXDMAEN				14
#define I2C_CR1_RXDMAEN				15
#define I2C_CR1_SBC					16
#define I2C_CR1_NOSTRETCH			17
#define I2C_CR1_WUPEN				18
#define I2C_CR1_GCEN				19
#define I2C_CR1_SMBHEN				20
#define I2C_CR1_SMBDEN				21
#define I2C_CR1_ALERTEN				22
#define I2C_CR1_PECEN				23

#define I2C_CR2_SADD				0
#define I2C_CR2_RD_WRN				10
#define I2C_CR2_ADD10				11
#define I2C_CR2_HEAD10R				12
#define I2C_CR2_START				13
#define I2C_CR2_STOP				14
#define I2C_CR2_NACK				15
#define I2C_CR2_NBYTES				16
#define I2C_CR2_RELOAD				24
#define I2C_CR2_AUTOEND				25
#define I2C_CR2_PECBYTE				26

#define I2C_OAR1_OA1_0				0
#define I2C_OAR1_OA1_1_7			1
#define I2C_OAR1_OA1_8_9			8
#define I2C_OAR1_OA1MODE			10
#define I2C_OAR1_OA1EN				15

#define I2C_ISR_TXE					0
#define I2C_ISR_TXIS				1
#define I2C_ISR_RXNE				2
#define I2C_ISR_ADDR				3
#define I2C_ISR_NACKF				4
#define I2C_ISR_STOPF				5
#define I2C_ISR_TC					6
#define I2C_ISR_TCR					7
#define I2C_ISR_BERR				8
#define I2C_ISR_ARLO				9
#define I2C_ISR_OVR					10
#define I2C_ISR_PECERR				11
#define I2C_ISR_TIMEOUT				12
#define I2C_ISR_ALERT				13
#define I2C_ISR_BUSY				15
#define I2C_ISR_DIR					16
#define I2C_ISR_ADDCODE				17

#define I2C_ICR_ADDRCF				3
#define I2C_ICR_NACKCF				4
#define I2C_ICR_STOPCF				5
#define I2C_ICR_BERRCF				8
#define I2C_ICR_ARLOCF				9
#define I2C_ICR_OVRCF				10
#define I2C_ICR_PECCF				11
#define I2C_ICR_TIMOUTCF			12
#define I2C_ICR_ALERTCF				13

#define I2C_TIMINGR_SCLL			0
#define I2C_TIMINGR_SCLH			8
#define I2C_TIMINGR_SDADEL			16
#define I2C_TIMINGR_SCLDEL			20
#define I2C_TIMINGR_PRESC			28

#include "stm32f303re_gpio_driver.h"
#include "stm32f303re_spi_driver.h"
#include "stm32f303re_i2c_driver.h"
#include "stm32f303re_rcc_driver.h"

#endif /* INC_STM32F303RE_H_ */
