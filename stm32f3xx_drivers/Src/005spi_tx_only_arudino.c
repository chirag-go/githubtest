/*
 * 005spi_tx_only_arudino.c
 *
 *  Created on: 25-Mar-2021
 *      Author: Chirag Goyal
 */


/*
 * PB12->SPI2_NSS
 * PB13->SPI2_CLK
 * PB14->SPI2_MISO
 * PB15->SPI2_MOSI
 * AF5
 */


/*
 * The peripheral clock for SPI is set at 36MHz but the output is at 8MHZ (idk why)
 * The instructors output clock seems to be 16MHz tho
 */
#include "stm32f3xx.h"
#include<string.h>

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

	//CLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
}
void GPIO_ButtonInit(void)
{

	GPIO_Handle_t user;
	//PC13
	user.pGPIOx = GPIOC;
	user.GPIO_PinConfig.GPIO_PinNumber = 13;
	user.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	user.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	user.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&user);
}
void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV4;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;

	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2handle);


}
void delay(void)
{
	for(uint32_t i=0;i<5000; i++)
		for(uint32_t j=0;j<50;j++);
}
int main(void)
{
	char user_data[] = "Hello World";

	GPIO_ButtonInit();

	SPI2_GPIOInits();

	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);

	SPI_SSIConfig(SPI2,ENABLE);

	while(1){
		while( GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13));
		delay();

		//SPI Peripheral needs to be enabled after configuration of spi
		SPI_PeripheralControl(SPI2, ENABLE);
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2,&dataLen,1);
		SPI_SendData(SPI2,(uint8_t *)user_data,strlen(user_data));
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
		SPI_PeripheralControl(SPI2, DISABLE);
	}
	return 0;
}
