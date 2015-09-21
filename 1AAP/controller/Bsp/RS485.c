
#include "includes.h"



/************************************************************
*485�շ����ƽų�ʼ��
*************************************************************/
void rs485_1_Init(uint32_t bps,uint16_t Parity)
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
*485�շ����ƽų�ʼ��
*************************************************************/
void rs485_2_Init(uint32_t bps,uint16_t Parity)
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
*485��������
pSn:1,��һ·
	2,�ڶ�·
	��������Ч
*************************************************************/
void RS485Send(unsigned char pSn,unsigned char *rxbuf,unsigned short len)
{
	if(pSn == 1)
	{
		RS4851TXENABLE();
		ComPortWrite(&nCom1,rxbuf,len);
		while(0 == ComPortWaitTxEnd(&nCom1))
		{
			//��ʱ
		}
		RS4851TXDISABLE();
	}
	else if(pSn == 2)
	{
		RS4852TXENABLE();
		ComPortWrite(&nCom2,rxbuf,len);
		while(0 == ComPortWaitTxEnd(&nCom2))
		{
			//��ʱ
		}
		RS4852TXDISABLE();		
	}
}

/************************************************************
*485��������
pSn:1,��һ·
	2,�ڶ�·
	��������Ч
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

/************************************************************
*485У���
*************************************************************/
uint8_t checksum8(uint8_t *chk8,uint8_t num) 
{        
	uint8_t temp=0;
	for( ;num>0;num--)
	{
		temp=temp+*chk8;
		if(temp<*chk8)
		temp++;
		chk8++;
	}    
	return temp;
}







/**************************** End of file **********************/
