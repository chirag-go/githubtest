/*
 * led_toggle.c
 *
 *  Created on: Feb 14, 2021
 *      Author: Chirag Goyal
 */

#include<gpio_driver.h>

/*
 * Instructions to write code:
 * 1. Make a gpio handle variable, led in this case
 * 2. Set all of the elements of pinconfig
 * 3. Enable the clock of the gpio
 * 4. Call init using &led
 * 5. Led toggle function with a delay
 */
void delay()
{
	for(int i=0;i<500000;i++);
}
int main()
{
/*
 * 	without button

	GPIO_regdef_t *reg;
	GPIO_PinConfig_t config;

	reg=GPIOA;
	config.GPIO_PinNumber = 5;
	config.GPIO_PinMode = GPIO_MODE_OUTPUT;
	config.GPIO_OpMode = GPIO_OP_PUPD;
	config.GPIO_PuPd = GPIO_PUPD_NOPUPD;
	config.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_Init(reg, config);

	while(1){
		GPIO_TogglePin(reg, 5);
		delay();
	}
*/



/*
 * with button
 */

	GPIO_regdef_t *ButtonReg;
	GPIO_PinConfig_t ButtonConfig;

	GPIO_regdef_t *LedReg;
	GPIO_PinConfig_t LedConfig;

	/*
	 * initialising led		A5
	 */
	LedReg=GPIOA;
	LedConfig.GPIO_PinNumber = 5;
	LedConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	LedConfig.GPIO_OpMode = GPIO_OP_PUPD;
	LedConfig.GPIO_PuPd = GPIO_PUPD_NOPUPD;
	LedConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_Init(LedReg, LedConfig);


	/*
	 * initialising button	C13
	 */
	ButtonReg=GPIOC;
	ButtonConfig.GPIO_PinNumber = 13;
	ButtonConfig.GPIO_PinMode =GPIO_MODE_INPUT;
	ButtonConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	//ButtonConfig.GPIO_OpMode = GPIO_OP_PUPD;
	ButtonConfig.GPIO_PuPd = GPIO_PUPD_NOPUPD;
	GPIO_Init(ButtonReg, ButtonConfig);

	GPIO_PeriClockControl(GPIOC, ENABLE);

	while(1){
		if((GPIO_ReadFromPin(GPIOC,13))==0){
			GPIO_TogglePin(GPIOA,5);
			delay();
		}
	}






}
