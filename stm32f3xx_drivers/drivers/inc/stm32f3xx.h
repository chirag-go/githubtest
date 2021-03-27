/*
 * stm32f4xx.h
 *
 *  Created on: Jan 31, 2021
 *      Author: Chirag Goyal
 */

#ifndef INC_STM32F3XX_H_
#define INC_STM32F3XX_H_

#include<stdint.h>

/*
 * Processor specific header files i.e, Cortex M3
 */

/*
 * NVIC ISER is interrupt set enable register, setting 0 has no effect while setting 1 enables the interrupt
 */
#define NVIC_ISER0		((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1		((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2		((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3		((volatile uint32_t*)0xE000E10C)

/*
 * NVIC ICER is interrupt clear enable register, setting 0 does nothing while setting 1 disables the interrupt
 */

#define NVIC_ICER0		((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1		((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2		((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3		((volatile uint32_t*)0xE000E18C)

/*
 * Priority Register base address
 */
#define NVIC_PR_BASEADDR		((volatile uint32_t*)0xE000E400)


#define NO_PR_BITS_IMPLEMENTED		4


#define FLASH_BASEADDR 				0x80000000U		// can be found in the memory map table in both datasheet and rm
#define SRAM1_BASEADDR 				0x20000000U
#define ROM_BASEADDR				0x1FFFD800U		//rom means system memory in case of stm
#define SRAM 						SRAM1_BASEADDR

#define PERIPH_BASE					0x40000000U
#define APB1PERIPH_BASE				PERIPH_BASE
#define APB2PERIPH_BASE				0x40010000U
#define AHB1PERIPH_BASE				0x40020000U
#define AHB2PERIPH_BASE				0x48000000U

#define GPIOA_BASEADDR				AHB2PERIPH_BASE
#define GPIOB_BASEADDR				(AHB2PERIPH_BASE+0x0400)
#define GPIOC_BASEADDR				(AHB2PERIPH_BASE+0X0800)
#define GPIOD_BASEADDR				(AHB2PERIPH_BASE+0X0C00)
#define GPIOE_BASEADDR				(AHB2PERIPH_BASE+0X1000)
#define GPIOF_BASEADDR				(AHB2PERIPH_BASE+0x1400)
#define GPIOG_BASEADDR				(AHB2PERIPH_BASE+0X1800)
#define GPIOH_BASEADDR				(AHB2PERIPH_BASE+0X1C00)

#define EXTI_BASEADDR				(APB2PERIPH_BASE+0X0400)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASE+0)
#define COMP_BASEADDR				(APB2PERIPH_BASE+0X1C)
#define OPAMP_BASEADDR				(APB2PERIPH_BASE+0X38)
#define TIM1_BASEADDR				(APB2PERIPH_BASE+0X2C00)
#define SPI1_BASEADDR				(APB2PERIPH_BASE+0X3000)
#define TIM8_BASEADDR				(APB2PERIPH_BASE+0X3400)
#define USART1_BASEADDR				(APB2PERIPH_BASE+0X3800)
#define SPI4_BASEADDR				(APB2PERIPH_BASE+0X3C00)
#define TIM15_BASEADDR				(APB2PERIPH_BASE+0X4000)
#define TIM16_BASEADDR				(APB2PERIPH_BASE+0X4400)
#define TIM17_BASEADDR				(APB2PERIPH_BASE+0X4800)
#define TIM20_BASEADDR				(APB2PERIPH_BASE+0X5000)


#define I2C3_BASEADDR 				(APB1PERIPH_BASE+0X7800)
#define I2C2_BASEADDR				(APB1PERIPH_BASE+0X5800)
#define I2C1_BASEADDR				(APB1PERIPH_BASE+0X5400)
#define DAC1_BASEADDR				(APB1PERIPH_BASE+0X7400)
//#define DAC2_BASEADDR				(APB1PERIPH_BASE+0x9800)
#define PWR_BASEADDR				(APB1PERIPH_BASE+0X7000)
#define bxCAN_BASEADDR				(APB1PERIPH_BASE+0X6400)
#define USB_SRAM_BASEADDR			(APB1PERIPH_BASE+0X6000)
#define USB_device_BASEADDR			(APB1PERIPH_BASE+0X5C00)
#define UART5_BASEADDR				(APB1PERIPH_BASE+0X5000)
#define UART4_BASEADDR				(APB1PERIPH_BASE+0X4C00)
#define USART3_BASEADDR				(APB1PERIPH_BASE+0X4800)
#define USART2_BASEADDR				(APB1PERIPH_BASE+0X4400)
#define	I2S3ext_BASEADDR			(APB1PERIPH_BASE+0X4000)
#define SPI3_BASEADDR				(APB1PERIPH_BASE+0X3C00)
#define I2S3_BASEADDR				(APB1PERIPH_BASE+0X3C00)
#define SPI2_BASEADDR				(APB1PERIPH_BASE+0X3800)
#define I2S2_BASEADDR				(APB1PERIPH_BASE+0X3800)
#define I2S2ext_BASEADDR			(APB1PERIPH_BASE+0X3400)
#define IWDG_BASEADDR				(APB1PERIPH_BASE+0X3000)
#define WWDG_BASEADDR				(APB1PERIPH_BASE+0X2C00)
#define RTC_BASEADDR				(APB1PERIPH_BASE+0X2800)
#define TIM7_BASEADDR				(APB1PERIPH_BASE+0X1400)
#define TIM6_BASEADDR				(APB1PERIPH_BASE+0X1000)
#define TIM4_BASEADDR				(APB1PERIPH_BASE+0X0800)
#define TIM3_BASEADDR				(APB1PERIPH_BASE+0X0400)
#define TIM2_BASEADDR				(APB1PERIPH_BASE+0X0000)

#define RCC_BASEADDR				(AHB1PERIPH_BASE+0X1000)




typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
	volatile uint32_t BRR;

}GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t AHBRSTR;
	volatile uint32_t CFGR2;
	volatile uint32_t CFGR3;
}RCC_RegDef_t;

typedef struct
{
	volatile uint32_t IMR1;			//offset 0
	volatile uint32_t EMR1;			//offset 4
	volatile uint32_t RTSR1;		//offset 8
	volatile uint32_t FTSR1;		//offset c
	volatile uint32_t SWIER1;		//offset 10
	volatile uint32_t PR1;			//offset 14
	volatile uint32_t garbage[2];	//offset 18, 1c
	volatile uint32_t IMR2;			//offset 20
	volatile uint32_t EMR2;			//offset 24
	volatile uint32_t RTSR2;		//offset 28
	volatile uint32_t FTSR2;		//offset 2c
	volatile uint32_t SWIER2;		//offset 30
	volatile uint32_t PR2;			//offset 34
}EXTI_RegDef_t;

typedef struct {
	volatile uint32_t CFGR1;
	volatile uint32_t RCR;
	volatile uint32_t EXTICR[4];
	volatile uint32_t CFGR2;		//offset is 0x18
	volatile uint32_t reserved[11];
	volatile uint32_t CFGR4;
	volatile uint32_t reserved1;
	volatile uint32_t CFGR3;		//offset should be 0x50

}SYSCFG_RegDef_t;

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
}SPI_RegDef_t;

#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)


#define RCC			((RCC_RegDef_t *)RCC_BASEADDR)

#define EXTI		((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SYSCFG_PCLK_EN()	((RCC->APB2ENR) |= (1<<0))

#define SPI1			((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4			((SPI_RegDef_t *)SPI4_BASEADDR)



/*
 * CLOCK ENABLE MACROS FOR GPIOs
 */

#define GPIOA_PCLK_EN()		(RCC->AHBENR |= (1<<17))
#define GPIOB_PCLK_EN()		(RCC->AHBENR |= (1<<18))
#define GPIOC_PCLK_EN()		(RCC->AHBENR |= (1<<19))
#define GPIOD_PCLK_EN()		(RCC->AHBENR |= (1<<20))
#define GPIOE_PCLK_EN()		(RCC->AHBENR |= (1<<21))
#define GPIOF_PCLK_EN()		(RCC->AHBENR |= (1<<22))
#define GPIOG_PCLK_EN()		(RCC->AHBENR |= (1<<23))
#define GPIOH_PCLK_EN()		(RCC->AHBENR |= (1<<16))




/*
 * Clock Enable macros for i2cx
 */

#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1<<22))



/*
 * clock enable macros for spix
 */

#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1<<15))



/*
 * Clock disable macros for SPI
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1<<15))

/*
 * Bit definition macros for SPI Registers
 */
#define SPI_CR1_CPHA	 0
#define SPI_CR1_CPOL	 1
#define SPI_CR1_MSTR	 2
#define SPI_CR1_BR		 3
#define SPI_CR1_SPE		 6
#define SPI_CR1_LSBF	 7
#define SPI_CR1_SSI		 8
#define SPI_CR1_SSM		 9
#define SPI_CR1_RXONLY	 10
#define SPI_CR1_CRCL	 11
#define SPI_CR1_DFF		SPI_CR1_CRCL
#define SPI_CR1_CRCN	 12
#define SPI_CR1_CRCEN	 13
#define SPI_CR1_BIDIOE	 14
#define SPI_CR1_BIDIMODE 15


#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEK		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_NSSP		3
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7
#define SPI_CR2_DS			8
#define SPI_CR2_FRXTH		12
#define SPI_CR2_LDMA_RX		13
#define SPI_CR2_LDMA_TX		14


#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8
#define SPI_SR_FRLVL		9
#define SPI_SR_FTLVL		11






/*
 * Clock disable macros for gpio
 */

#define GPIOA_PCLK_DI()		(RCC->AHBENR &= ~(1<<17))
#define GPIOB_PCLK_DI()		(RCC->AHBENR &= ~(1<<18))
#define GPIOC_PCLK_DI()		(RCC->AHBENR &= ~(1<<19))
#define GPIOD_PCLK_DI()		(RCC->AHBENR &= ~(1<<20))
#define GPIOE_PCLK_DI()		(RCC->AHBENR &= ~(1<<21))
#define GPIOF_PCLK_DI()		(RCC->AHBENR &= ~(1<<22))
#define GPIOG_PCLK_DI()		(RCC->AHBENR &= ~(1<<23))
#define GPIOH_PCLK_DI()		(RCC->AHBENR &= ~(1<<16))

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()		do{	(RCC->AHBRSTR |= (1<<17));	RCC->AHBRSTR &= ~(1<<17);	}while(0)
#define GPIOB_REG_RESET()		do{	(RCC->AHBRSTR |= (1<<18));	RCC->AHBRSTR &= ~(1<<18);	}while(0)
#define GPIOC_REG_RESET()		do{	(RCC->AHBRSTR |= (1<<19));	RCC->AHBRSTR &= ~(1<<19);	}while(0)
#define GPIOD_REG_RESET()		do{	(RCC->AHBRSTR |= (1<<20));	RCC->AHBRSTR &= ~(1<<20);	}while(0)
#define GPIOE_REG_RESET()		do{	(RCC->AHBRSTR |= (1<<21));	RCC->AHBRSTR &= ~(1<<21);	}while(0)
#define GPIOF_REG_RESET()		do{	(RCC->AHBRSTR |= (1<<22));	RCC->AHBRSTR &= ~(1<<22);	}while(0)
#define GPIOG_REG_RESET()		do{	(RCC->AHBRSTR |= (1<<23));	RCC->AHBRSTR &= ~(1<<23);	}while(0)
#define GPIOH_REG_RESET()		do{	(RCC->AHBRSTR |= (1<<16));	RCC->AHBRSTR &= ~(1<<16);	}while(0)


/*
 * IRQ numbers seen from the vector table in nvic register
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/*
 * IRQ priority
 */
#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		0
#define NVIC_IRQ_PRI2		0
#define NVIC_IRQ_PRI3		0
#define NVIC_IRQ_PRI4		0
#define NVIC_IRQ_PRI5		0
#define NVIC_IRQ_PRI6		0
#define NVIC_IRQ_PRI7		0
#define NVIC_IRQ_PRI8		0
#define NVIC_IRQ_PRI9		0
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15


/*
 * to return the portcode (for gpio as of now)
 */

#define GPIO_BASEADDR_TO_CODE(x)  (	(x == GPIOA)?0:\
									(x == GPIOB)?1:\
									(x == GPIOC)?2:\
									(x == GPIOD)?3:\
									(x == GPIOE)?4:\
									(x == GPIOF)?5:\
									(x == GPIOG)?6:\
									(x == GPIOH)?7:0 )

/*
 * some macros
 */
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define FLAG_SET SET
#define FLAG_RESET RESET



#include"stm32f3xx_gpio_driver.h"
#include"stm32f3xx_spi.h"

#endif /* INC_STM32F3XX_H_ */
