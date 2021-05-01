/*
 * 002ledtoggleusingbutton.c
 *
 *  Created on: 09-Feb-2021
 *      Author: Chirag Goyal
 */

#include "stm32f3xx_gpio_driver.h"
//#include "stm32f3xx.h"

void delay()
{
	for(int i=0;i<300000;i++);
}

int main (void)
{
	GPIO_handle_t user;
	GPIO_handle_t led;

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
	user.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	user.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	user.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&user);

	while(1){
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)==0){
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
			delay();
		}
	}

}
