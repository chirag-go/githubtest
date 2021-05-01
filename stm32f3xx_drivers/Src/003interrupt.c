/*
 * 002ledtoggleusingbutton.c
 *
 *  Created on: 09-Feb-2021
 *      Author: Chirag Goyal
 */

#include<string.h>
#include "stm32f3xx_gpio_driver.h"
//#include "stm32f3xx.h"

void delay()
{
	for(int i=0;i<300000;i++);
}

int main (void)
{
	GPIO_Handle_t user;
	GPIO_Handle_t led;
	memset(&user,0,sizeof(user));	// for initialising 0s in all of the members of structure
	memset(&led,0,sizeof(led));

	led.pGPIOx = GPIOA;
	led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&led);

	//PC13
	user.pGPIOx = GPIOC;
	user.GPIO_PinConfig.GPIO_PinNumber = 13;
	user.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	user.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	user.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&user);

	//IRQ Configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptCofig(IRQ_NO_EXTI15_10, ENABLE);


	while(1);

}


void EXTI15_10_IRQHandler()
{
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
}
