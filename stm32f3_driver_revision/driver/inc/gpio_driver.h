/*
 * driver.h
 *
 *  Created on: Feb 14, 2021
 *      Author: Chirag Goyal
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include "stdint.h"
#include "stm32f3.h"


/*
 * Instructions to write the code on my own
 * 1. Make a pin config struct
 * 2. Handle structure that will hold a regdef pointer *pGIO and a pinconfig variable
 * 3. Define pin numbers and various modes
 * 4. Clock control, init deinit
 * 5. Read write and interrupt declaration
 */


typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PuPd;
	uint8_t GPIO_AltFunc [2];
	uint8_t GPIO_OpMode;
}GPIO_PinConfig_t;
/*
typedef struct{
	GPIO_regdef_t *pGPIOx;
	GPIO_PinConfig_t PinConfig;
}GPIO_Handle_t;
*/
#define 	GPIO_SPEED_HIGH 	3
#define		GPIO_SPEED_MEDIUM	1
#define		GPIO_SPEED_LOW		0


#define		GPIO_MODE_INPUT		0
#define		GPIO_MODE_OUTPUT	1
#define		GPIO_MODE_ALT		2
#define		GPIO_MODE_ANALOG	3


#define		GPIO_PUPD_NOPUPD	0
#define		GPIO_PUPD_PU		1
#define		GPIO_PUPD_PD		2


#define		GPIO_OP_PUPD		0
#define 	GPIO_OP_OD			1

/*
 * Clock control
 */
void GPIO_PeriClockControl(GPIO_regdef_t *pGPIOx,uint8_t EnOrDi);

/*
 * Init deinit
 */
void GPIO_Init(GPIO_regdef_t *pGPIOx,GPIO_PinConfig_t PinConfig);
void GPIO_DeInit(GPIO_regdef_t *pGPIOx);


uint8_t GPIO_ReadFromPin(GPIO_regdef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromPort(GPIO_regdef_t *pGPIOx);
void GPIO_WriteToPin(GPIO_regdef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToPort(GPIO_regdef_t *pGPIOx, uint16_t Value);
void GPIO_TogglePin(GPIO_regdef_t *pGPIOx, uint8_t PinNumber);




#endif /* INC_GPIO_DRIVER_H_ */
