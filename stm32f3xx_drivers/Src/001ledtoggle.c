/*
 * 001ledtoggle.c
 *
 *  Created on: 08-Feb-2021
 *      Author: Chirag Goyal
 */
#include "stm32f3xx_gpio_driver.h"
//#include "stm32f3xx.h"

void delay(void)
{
	for(uint32_t i=0;i<50000; i++);
}

int main(void)
{
	GPIO_handle_t led;
	led.pGPIOx= GPIOA;
	led.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	led.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	led.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_HIGH;
	led.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	led.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	/* For open drain
	 *
	 */
	//led.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_OD;
	//led.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;


	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&led);

	while(1){
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		delay();
	}
}
