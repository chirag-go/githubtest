/*
 * stm32f3xx_gpio_driver.c
 *
 *  Created on: 02-Feb-2021
 *      Author: Chirag Goyal
 */


#include "stm32f3xx_gpio_driver.h"


/*****************************************************
 *@fn			-	GPIO_PeriClockControl
 *@brief		-	This function enables or disables peripheral clock for the given Gpio port
 *@
 *@param[in]	-	Base address of the gpio peripheral
 *@param[in]	-	Enable or disable macros
 *@param[in]	-
 *@return		-	none
 *@note			- 	none
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}

	else
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}


/*****************************************************
 *@fn			-	GPIO_Init
 *@brief		-	This function initialises the gpio pin/port(?)
 *@
 *@param[in]	-	Base address of the gpio pin/port(?)
 *@param[in]	-
 *@param[in]	-
 *@return		-	none
 *@note	-
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;  // temp register

	//Enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG){
		//non interrupt mode
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));		//clearing
		pGPIOHandle->pGPIOx->MODER |= temp;			//setting

	}
	else{
		//interrupt mode
		/*
		 * Some points for interrupt:
		 * 1. Pin must be in input configuration
		 * 2. Configure the edge trigger
		 *	// watch notes for more points, am no able to understand as of now
		 */

		//1. Configure either FTSR or RTSR register
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//configure for falling edge interrupt || register is FTSR
			EXTI->FTSR1 |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);				//setting up the falling trigger selection register
			EXTI->RTSR1 &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);			//resetting rising trigger selection register
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//configure rising edge interrupt		|| register is RTSR
			EXTI->RTSR1 |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR1 &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//configure for both		|| both FTSR and RTSR
			EXTI->FTSR1 |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR1 |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in SYSCNFG_EXTICR
		//we have to make 2 temp variables one will store the index with dividing by4
		// and the other will hold value with %4
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode <<(temp2 * 4);


		//3. enable the exti interrupt delievery using IMR
		EXTI->IMR1 |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	//2. configure the speed
	temp=0;
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//3. configure the pupd setting
	temp=0;
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	//4. configure the optype
	temp=0;
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		//configure the alt functionality
		// the course does not clear the alt functionality registers
		// it only sets them assuming initial state to be zero
		uint8_t temp1, temp2;
		temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
	}

}



void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx==GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx==GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx==GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx==GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx==GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx==GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx==GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx==GPIOH)
	{
		GPIOH_REG_RESET();
	}
}


/*
 * read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value= (uint8_t)((pGPIOx->IDR>>PinNumber) & 0x00000001);
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value= (uint16_t)pGPIOx->IDR;
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value)
{
	if(Value==SET){
		//write 1 to output data register
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value)
{
	pGPIOx->ODR=Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	//toggle using c is included with xor
	//a^=1;
	pGPIOx->ODR ^= (1<<PinNumber);
}

/*
 * Interrupt
 */

void GPIO_IRQInterruptCofig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE){
		if(IRQNumber < 32){
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber >= 32 && IRQNumber < 64){
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >=64 && IRQNumber <96){
			//program ISER2 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 64));
		}
	}else{
		if(IRQNumber <= 31){
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){
			*NVIC_ISER0 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >=64 && IRQNumber <96){
			*NVIC_ISER0 |= (1 << (IRQNumber % 64));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. IPR register
	uint8_t iprx =IRQNumber /4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8*iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + (iprx)) |= (IRQPriority << shift_amount);			//maybe wrong code, might get fucked
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register
	if(EXTI->PR1 & (1<<PinNumber)){
		//clear
		EXTI->PR1 |= (1<<PinNumber);
	}
}
