#ifndef __OWN_SPI_H_
#define __OWN_SPI_H_

#define SPI_ENABLE 1
#if SPI_ENABLE

#ifdef __cplusplus
 extern "C" {
#endif

#include "own_type.h"
#include "own_gpio.h"
#include "stm32f10x_spi.h"
						  	    													  
void SPIx_SetSpeed(SPI_TypeDef * SPIx,uint8_t SpeedSet); //设置SPIx速度   

//串行外设接口SPI的初始化，SPI配置成主模式							  
void SPIx_Init(SPI_TypeDef * SPIx,
			   uint16_t Direction,
			   uint16_t Mode,
			   uint16_t DataSize,
			   uint16_t CPOL,
			   uint16_t CPHA,
			   uint16_t NSS,
			   uint16_t BaudRatePrescaler,
			   uint16_t FirstBit,
			   uint16_t CRCPolynomial);

//SPIx 读写一个字节
//返回值:读取到的字节
uint8_t SPIx_ReadWriteByte(SPI_TypeDef * SPIx,uint8_t TxData);

// SPI总线速度设置 
#define SPI_SPEED_2   		0
#define SPI_SPEED_4   		1
#define SPI_SPEED_8   		2
#define SPI_SPEED_16  		3
#define SPI_SPEED_32 		4
#define SPI_SPEED_64 		5
#define SPI_SPEED_128 		6
#define SPI_SPEED_256 		7


/************SPI1的物理信息**********/
#define SPI1_CS		PA_04
#define SPI1_SCK	PA_05
#define SPI1_MISO	PA_06
#define SPI1_MOSI	PA_07
#define	RCC_APBxPeriph_SPI1	RCC_APB2Periph_SPI1

/************SPI2的物理信息**********/
#define SPI2_CS		PB_12
#define SPI2_SCK	PB_13
#define SPI2_MISO	PB_14
#define SPI2_MOSI	PB_15
#define	RCC_APBxPeriph_SPI2	RCC_APB1Periph_SPI2

/************SPI3的物理信息**********/
#define SPI3_CS		PA_15
#define SPI3_SCK	PB_03
#define SPI3_MISO	PB_04
#define SPI3_MOSI	PB_05
#define	RCC_APBxPeriph_SPI3	RCC_APB1Periph_SPI3


#ifdef __cplusplus
}
#endif

#endif

#endif

