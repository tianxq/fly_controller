#ifndef _RS_485_H_20141225_
#define _RS_485_H_20141225_

#include <stm32f10x.h>

#define RS4851				1
#define RS4852				2

#define RS4851_GPIO_PIN		GPIO_Pin_8
#define RS4851_GPIO_PORT	GPIOA 
#define RS4851_GPIO_CLK		RCC_APB2Periph_GPIOA

#define RS4852_GPIO_PIN		GPIO_Pin_4
#define RS4852_GPIO_PORT	GPIOA 
#define RS4852_GPIO_CLK		RCC_APB2Periph_GPIOA


#define RS4851TXENABLE()	GPIO_SetBits(RS4851_GPIO_PORT,RS4851_GPIO_PIN)
#define RS4851TXDISABLE()	GPIO_ResetBits(RS4851_GPIO_PORT,RS4851_GPIO_PIN)

#define RS4852TXENABLE()	GPIO_SetBits(RS4852_GPIO_PORT,RS4852_GPIO_PIN)
#define RS4852TXDISABLE()	GPIO_ResetBits(RS4852_GPIO_PORT,RS4852_GPIO_PIN)


#define UART_CONFIG_PAR_NONE    0x00000000  // No parity
#define UART_CONFIG_PAR_EVEN    0x00000002  // Even parity
#define UART_CONFIG_PAR_ODD     0x00000004  // Odd parity


/************************************************************
*485收发控制脚初始化
*************************************************************/
void rs485_1_Init(uint32_t bps,uint8_t Parity);
void rs485_2_Init(uint32_t bps,uint8_t Parity);

/************************************************************
*485发送数据
pSn:1,第一路
	2,第二路
	其他，无效
*************************************************************/
void RS485Send(uint8_t pSn,uint8_t *rxbuf,uint16_t len);

/************************************************************
*485接收数据
pSn:1,第一路
	2,第二路
	其他，无效
*************************************************************/
uint16_t RS485Recive(uint8_t pSn,uint8_t *rxbuf,uint16_t len,uint16_t timeout);
















#endif
