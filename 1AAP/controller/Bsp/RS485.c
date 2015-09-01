
#include "includes.h"



/************************************************************
*485收发控制脚初始化
*************************************************************/
void rs485_1_Init(uint32_t bps,uint8_t Parity)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* GPIO clock enable */
	RCC_APB2PeriphClockCmd(RS4851_GPIO_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =  RS4851_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(RS4851_GPIO_PORT, &GPIO_InitStructure);
	
	ComPort1Init(bps,Parity);
	RS4851TXDISABLE();
}

/************************************************************
*485收发控制脚初始化
*************************************************************/
void rs485_2_Init(uint32_t bps,uint8_t Parity)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* GPIO clock enable */
	RCC_APB2PeriphClockCmd(RS4852_GPIO_CLK, ENABLE);
	//Reset GPIO
	GPIO_InitStructure.GPIO_Pin =  RS4852_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_Init(RS4852_GPIO_PORT, &GPIO_InitStructure);

	ComPort2Init(bps,Parity);
	RS4852TXDISABLE();
}

/************************************************************
*485发送数据
pSn:1,第一路
	2,第二路
	其他，无效
*************************************************************/
void RS485Send(unsigned char pSn,unsigned char *rxbuf,unsigned short len)
{
	if(pSn == 1)
	{
		RS4851TXENABLE();
		ComPortWrite(&nCom1,rxbuf,len);
		while(0 == ComPortWaitTxEnd(&nCom1))
		{
			//延时
		}
		RS4851TXDISABLE();
	}
	else if(pSn == 2)
	{
		RS4852TXENABLE();
		ComPortWrite(&nCom2,rxbuf,len);
		while(0 == ComPortWaitTxEnd(&nCom2))
		{
			//延时
		}
		RS4852TXDISABLE();		
	}
}

/************************************************************
*485接收数据
pSn:1,第一路
	2,第二路
	其他，无效
*************************************************************/
uint16_t RS485Recive(uint8_t pSn,uint8_t *rxbuf,uint16_t len,uint16_t timeout)
{
	uint16_t rxLen = 0;
	
	if(pSn == 1)
	{
		rxLen = ComPortRead(&nCom1,rxbuf,len,timeout);
	}
	else if(pSn == 2)
	{
		rxLen = ComPortRead(&nCom2,rxbuf,len,timeout);
	}
	return rxLen;	
}










/**************************** End of file **********************/
