/*
 * gpio_driver.c
 *
 *  Created on: Feb 14, 2021
 *      Author: Chirag Goyal
 */
#include "gpio_driver.h"


/*
 * Instructions to write code on own
 * 1. Make a clock control function for various GPIOs
 * 2. Init function configures the complete gpio pin config
 * 3. Deinit will reset the gpio
 * 4. Read from and write to pin, port or toggle
 * 			uint8_t GPIO_ReadFromInputPin(GPIO_regdef_t *pGPIOx, uint8_t PinNumber)
 *
 */


void GPIO_PeriClockControl(GPIO_regdef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi==ENABLE){
		if(pGPIOx==GPIOA){
			GPIOA_CLK_EN();
		}else if(pGPIOx==GPIOB){
			GPIOB_CLK_EN();
		}else if(pGPIOx==GPIOC){
			GPIOC_CLK_EN();
		}else if(pGPIOx==GPIOD){
			GPIOD_CLK_EN();
		}else if(pGPIOx==GPIOE){
			GPIOE_CLK_EN();
		}else if(pGPIOx==GPIOF){
			GPIOF_CLK_EN();
		}else if(pGPIOx==GPIOG){
			GPIOG_CLK_EN();
		}else if(pGPIOx==GPIOH){
			GPIOH_CLK_EN();
		}
	}
	else{
		if(pGPIOx==GPIOA){
			GPIOA_CLK_DIS();
		}else if(pGPIOx==GPIOB){
			GPIOB_CLK_DIS();
		}else if(pGPIOx==GPIOC){
			GPIOC_CLK_DIS();
		}else if(pGPIOx==GPIOD){
			GPIOD_CLK_DIS();
		}else if(pGPIOx==GPIOE){
			GPIOE_CLK_DIS();
		}else if(pGPIOx==GPIOF){
			GPIOF_CLK_DIS();
		}else if(pGPIOx==GPIOG){
			GPIOG_CLK_DIS();
		}else if(pGPIOx==GPIOH){
			GPIOH_CLK_DIS();
		}
	}
}



void GPIO_Init(GPIO_regdef_t *pGPIOx,GPIO_PinConfig_t PinConfig)
{
	/*
	 * Notes to self for making init, guessing how to make it
	 * in this we define all of the pin configuration with the respective registers
	 * the pin config includes:
	 * 		pin number
	 * 		pin mode
	 * 		pin speed
	 * 		pin alt functionality
	 * 		pin pupd
	 * 		pin op type
	 */

	GPIO_PeriClockControl(pGPIOx, ENABLE);

	volatile uint32_t temp = 0;

	//setting up the MODER register for pin mode
	if(PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG){
		//non interrupt
		pGPIOx->MODER &= ~(3<<(2*PinConfig.GPIO_PinNumber));
		temp |= (PinConfig.GPIO_PinMode<<(2*PinConfig.GPIO_PinNumber));
		pGPIOx->MODER |= temp;
		temp=0;
	}
	else{
		//interrupt

	}

	//setting up speed register
	temp |= (PinConfig.GPIO_PinSpeed<<(2*PinConfig.GPIO_PinNumber));
	pGPIOx->OSPEEDR &= ~(3<<(2*PinConfig.GPIO_PinNumber));
	pGPIOx->OSPEEDR |= temp;
	temp=0;

	//setting up op type
	temp |= (PinConfig.GPIO_OpMode<<(PinConfig.GPIO_PinNumber));
	pGPIOx->OTYPER &= ~(1<<PinConfig.GPIO_PinNumber);
	pGPIOx->OTYPER |= temp;
	temp=0;

	//setting up pupd
	temp |= (PinConfig.GPIO_PuPd<<(2*PinConfig.GPIO_PinNumber));
	pGPIOx->PUPDR &= ~(3<<(2*PinConfig.GPIO_PinNumber));
	pGPIOx->PUPDR |= temp;
	temp=0;

	//setting up alt functionality
	//leaving it pending as of now

}


//i should prolly make deinit, leaving it pending for now


void GPIO_WriteToPin(GPIO_regdef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value==1){
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}
void GPIO_TogglePin(GPIO_regdef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx ->ODR ^= (1<<PinNumber);
}
uint8_t GPIO_ReadFromPin(GPIO_regdef_t *pGPIOx,uint8_t PinNumber)
{
	uint8_t value;
	value= (uint8_t)((pGPIOx->IDR>>PinNumber) & 0x00000001);
	return value;
}







