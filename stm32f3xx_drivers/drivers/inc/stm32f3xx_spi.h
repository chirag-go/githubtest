/*
 * stm32f3xx_spi.h
 *
 *  Created on: 23-Feb-2021
 *      Author: Chirag Goyal
 */

#ifndef INC_STM32F3XX_SPI_H_
#define INC_STM32F3XX_SPI_H_

#include "stm32f3xx.h"



/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;			//in my case it's CRCL : bit length
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0

/*
 *  @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD 				1
#define SPI_BUS_CONFIG_HD 				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/*
 *  @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 *  @SPI_DFF
 */
#define SPI_DFF_8BITS	0
#define SPI_DFF_16BITS	1

/*
 *  @CPOL
 */
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

/*
 *  @CPHA
 */
#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0

/*
 *  @SPI_SSM
 */
#define SPI_SSM_DI		0
#define SPI_SSM_EN		1



/*
 * Spi related status flag
 */
#define SPI_TXE_FLAG		(1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG		(1<<SPI_SR_RXNE)
#define SPI_BUSY_FLAG		(1<<SPI_SR_BSY)



/*
 * Spi register reset macros
 */
#define SPI1_REG_RESET()			do{	(RCC->APB1RSTR |= (1<<12));		(RCC->APB2RSTR &= ~(1<<12));	}while(1);
#define SPI2_REG_RESET()			do{	(RCC->APB1RSTR |= (1<<14));		(RCC->APB1RSTR &= ~(1<<14));	}while(1);
#define SPI3_REG_RESET()			do{	(RCC->APB1RSTR |= (1<<15));		(RCC->APB1RSTR &= ~(1<<15));	}while(1);
#define SPI4_REG_RESET()			do{	(RCC->APB1RSTR |= (1<<15));		(RCC->APB2RSTR &= ~(1<<15));	}while(1);


/*
 * clock control
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);


/*
 * Init and de-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and recieve
 */
void SPI_SendData (SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len);


/*
 * IRQ Configuration and ISR handling
 */

void SPI_IRQInterruptCofig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * SPI Peripheral Control
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_SSOEConfig (SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

#endif /* INC_STM32F3XX_SPI_H_ */
