/*
 * 004spi_tx_test.c
 *
 *  Created on: 17-Mar-2021
 *      Author: Chirag Goyal
 */

/*
 * PB12->SPI2_NSS
 * PB13->SPI2_CLK
 * PB14->SPI2_MISO
 * PB15->SPI2_MOSI
 * AF5
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



	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);


	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_WriteToOutputPin(GPIOB,12,1);
	GPIO_Init(&SPIPins);
}
void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;

	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2handle);


}

int main(void)
{
	char user_data[] = "Hello World";

	SPI2_GPIOInits();

	SPI2_Inits();

	SPI_SSIConfig(SPI2,ENABLE);

	//SPI Peripheral needs to be enabled after configuration of spi
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2,(uint8_t *)user_data,strlen(user_data));
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
	SPI_PeripheralControl(SPI2, DISABLE);
	while(1);
	return 0;
}
