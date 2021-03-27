/*
 * stm32f3xx_spi.c
 *
 *  Created on: 23-Feb-2021
 *      Author: Chirag Goyal
 */

#include "stm32f3xx_spi.h"




/*
 * clock control
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		if(pSPIx==SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx==SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx==SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx==SPI4)
		{
			SPI4_PCLK_EN();
		}
	}

	else
	{
		if(pSPIx==SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx==SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx==SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if(pSPIx==SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}


/*
 * Init and de-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//first lets configure the SPI_CR1 register
	uint32_t tempreg = 0;

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//1. configure the device mode
	/*if(pSPIHandle->SPIConfig.SPI_DeviceMode == 1){
		pSPIHandle->pSPIx->CR1 |= (1<<2);
	}
	else if(pSPIHandle->SPIConfig.SPI_DeviceMode == 0){
		pSPIHandle->pSPIx->CR1 &= ~(1<<2);
	}*/
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << 2);

	//2. Configure the bus config
	// full duplex
	// bidimode = 0
	// rxonly = 0

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// bidimode = 0, bidioe =  and rx only = 0
		tempreg &= ~(1<<15);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// bidimode = 1, bidioe =  and rx only =0
		tempreg |= (1<<15);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		// bidimode = 0, bidioe = 0 and rx only = 1
		tempreg &= ~(1<<15);
		tempreg |= (1<<15);
	}

	//3. Initialise the clock speed
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed <<3);

	//4. Bit length
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << 11);

	//5. Initialise CPOL and CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL<<1);
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA<<0);

	//SPI_CR2(SPI2) |= SPI_CR2_SSOE;
	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM<<SPI_CR1_SSM);



	pSPIHandle->pSPIx->CR1 = tempreg;


	//pSPIHandle->pSPIx->CR2 |= (1<<)
}
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	//this is not complete, since he gave it in to do
	if(pSPIx==SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx==SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx==SPI3)
	{
		SPI3_REG_RESET();
	}
	else if(pSPIx==SPI4)
	{
		SPI4_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName)
{
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*
 * Data send and recieve
 */
void SPI_SendData (SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len)
{
	SPI_PeripheralControl(SPI2, ENABLE);
	while(Len>0){
		//1. wait until TXE is set
		//while(!(pSPIx->SR &(1<<1)));
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		//2. check DFF bit in CR1
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF)){
			//16 bit dff
			//1. load the data into the DR
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else{
			//without volatile the bits sent are 8 bit, however clock is generated for 16 bit
			(*(volatile uint8_t *)&pSPIx->DR) = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}
void SPI_RecieveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len)
{

}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == 1)
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	else
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
}


void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == 1)
		pSPIx->CR1 |= (1<<SPI_CR1_SSI);
	else
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);
}

void SPI_SSOEConfig (SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == 1)
		pSPIx->CR2 |= (1<<SPI_CR2_SSOE);
	else
		pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE);
}

/*
 * IRQ Configuration and ISR handling
 */

void SPI_IRQInterruptCofig(uint8_t IRQNumber, uint8_t EnorDi)
{

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

}
