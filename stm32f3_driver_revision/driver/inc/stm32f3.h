/*
 * stm32f3.h
 *
 *  Created on: Feb 13, 2021
 *      Author: Chirag Goyal
 */

#ifndef INC_STM32F3_H_
#define INC_STM32F3_H_

#define		FLASH_BASEADDR		0x08000000U
#define		SRAM_BASEADDR		0x20000000U
#define		SYS_MEM_BASEADDR	0x1FFFD800U
#define		CCM_RAM_BASEADDR	0x10000000U
#define		APB1_BASEADDR		0x40000000U
#define		APB2_BASEADDR		0x40010000U
#define		AHB1_BASEADDR		0x40020000U
#define		AHB2_BASEADDR		0x48000000U
#define		AHB3_BASEADDR		0x50000000U

#define		GPIOA_OFFSET		0x0U
#define		GPIOB_OFFSET		0x0400U
#define		GPIOC_OFFSET		0x0800U
#define		GPIOD_OFFSET		0x0C00U
#define		GPIOE_OFFSET		0x1000U
#define		GPIOF_OFFSET		0x1400U
#define		GPIOG_OFFSET		0x1800U
#define		GPIOH_OFFSET		0x1C00U


#define		GPIOA_BASEADDR		(AHB2_BASEADDR+GPIOA_OFFSET)
#define		GPIOB_BASEADDR		(AHB2_BASEADDR+GPIOB_OFFSET)
#define		GPIOC_BASEADDR		(AHB2_BASEADDR+GPIOC_OFFSET)
#define		GPIOD_BASEADDR		(AHB2_BASEADDR+GPIOD_OFFSET)
#define		GPIOE_BASEADDR		(AHB2_BASEADDR+GPIOE_OFFSET)
#define		GPIOF_BASEADDR		(AHB2_BASEADDR+GPIOF_OFFSET)
#define		GPIOG_BASEADDR		(AHB2_BASEADDR+GPIOG_OFFSET)
#define		GPIOH_BASEADDR		(AHB2_BASEADDR+GPIOH_OFFSET)

#define		RCC_OFFSET			0x1000U
#define		RCC_BASEADDR		(AHB1_BASEADDR+RCC_OFFSET)

typedef struct{
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
}RCC_regdef_t;

//RCC_regdef_t *RCC = (uint32_t*)RCC_BASEADDR;



typedef struct{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
	volatile uint32_t BRR;
}GPIO_regdef_t;

/*
 * volatile char ch;
 *
 */


#define 	RCC			((RCC_regdef_t*)RCC_BASEADDR)
#define		GPIOA		((GPIO_regdef_t*) GPIOA_BASEADDR)		//moder 0x0 otyper 0x4 ospeedr 0x8
#define		GPIOB		((GPIO_regdef_t*) GPIOB_BASEADDR)
#define		GPIOC		((GPIO_regdef_t*) GPIOC_BASEADDR)
#define		GPIOD		((GPIO_regdef_t*) GPIOD_BASEADDR)
#define		GPIOE		((GPIO_regdef_t*) GPIOE_BASEADDR)
#define		GPIOF		((GPIO_regdef_t*) GPIOF_BASEADDR)
#define		GPIOG		((GPIO_regdef_t*) GPIOG_BASEADDR)
#define		GPIOH		((GPIO_regdef_t*) GPIOH_BASEADDR)




#define		GPIOA_CLK_EN()		(RCC->AHBENR |= (1<<17))
#define		GPIOB_CLK_EN()		(RCC->AHBENR |= (1<<18))
#define		GPIOC_CLK_EN()		(RCC->AHBENR |= (1<<19))
#define		GPIOD_CLK_EN()		(RCC->AHBENR |= (1<<20))
#define		GPIOE_CLK_EN()		(RCC->AHBENR |= (1<<21))
#define		GPIOF_CLK_EN()		(RCC->AHBENR |= (1<<22))
#define		GPIOG_CLK_EN()		(RCC->AHBENR |= (1<<23))
#define		GPIOH_CLK_EN()		(RCC->AHBENR |= (1<<16))


#define		GPIOA_CLK_DIS()		(RCC->AHBENR &= ~(1<<17))
#define		GPIOB_CLK_DIS()		(RCC->AHBENR &= ~(1<<18))
#define		GPIOC_CLK_DIS()		(RCC->AHBENR &= ~(1<<19))
#define		GPIOD_CLK_DIS()		(RCC->AHBENR &= ~(1<<20))
#define		GPIOE_CLK_DIS()		(RCC->AHBENR &= ~(1<<21))
#define		GPIOF_CLK_DIS()		(RCC->AHBENR &= ~(1<<22))
#define		GPIOG_CLK_DIS()		(RCC->AHBENR &= ~(1<<23))
#define		GPIOH_CLK_DIS()		(RCC->AHBENR &= ~(1<<16))


#define		GPIOA_REG_RESET()	do{	(RCC->AHBRSTR |= (1<<17));	(RCC->AHBRSTR &= ~(1<<17));	}while(0)
#define		GPIOB_REG_RESET()	do{	(RCC->AHBRSTR |= (1<<18));	(RCC->AHBRSTR &= ~(1<<18));	}while(0)
#define		GPIOC_REG_RESET()	do{	(RCC->AHBRSTR |= (1<<19));	(RCC->AHBRSTR &= ~(1<<19));	}while(0)
#define		GPIOD_REG_RESET()	do{	(RCC->AHBRSTR |= (1<<20));	(RCC->AHBRSTR &= ~(1<<20));	}while(0)
#define		GPIOE_REG_RESET()	do{	(RCC->AHBRSTR |= (1<<21));	(RCC->AHBRSTR &= ~(1<<21));	}while(0)
#define		GPIOF_REG_RESET()	do{	(RCC->AHBRSTR |= (1<<22));	(RCC->AHBRSTR &= ~(1<<22));	}while(0)
#define		GPIOG_REG_RESET()	do{	(RCC->AHBRSTR |= (1<<23));	(RCC->AHBRSTR &= ~(1<<23));	}while(0)
#define		GPIOH_REG_RESET()	do{	(RCC->AHBRSTR |= (1<<16));	(RCC->AHBRSTR &= ~(1<<16));	}while(0)



#define 	ENABLE		1
#define		DISABLE		0









#endif /* INC_STM32F3_H_ */
