
#include <string.h>
#include "includes.h"

//#define USEUART4

/************************** ����1���շ��ͻ����� ****************************/
//DMA����Buff���ڴ�DMA�������ݵ�UsartBuff
#define USART1DMARXMAXLEN 128
#define USART1DMATXMAXLEN 128
static unsigned char mUsart1DMARxBuff[USART1DMARXMAXLEN];
static unsigned char mUsart1DMATxBuff[USART1DMATXMAXLEN];

#define USART1RXMAXLEN	64
//Ŀǰ�ǲ���Ҫ���ͻ������ģ��������ݻ����Ͽ���ͨ��DMAһ�η������
#define USART1TXMAXLEN	1	
static unsigned char mUsart1RxBuff[USART1RXMAXLEN];
static unsigned char mUsart1TxBuff[USART1TXMAXLEN];

/**************************** ����2���շ��ͻ����� ************************/
//DMA����Buff���ڴ�DMA�������ݵ�UsartBuff
#define USART2DMARXMAXLEN 256
#define USART2DMATXMAXLEN 254
static unsigned char mUsart2DMARxBuff[USART2DMARXMAXLEN];
static unsigned char mUsart2DMATxBuff[USART2DMATXMAXLEN];

#define USART2RXMAXLEN	256
//Ŀǰ�ǲ���Ҫ���ͻ������ģ��������ݻ����Ͽ���ͨ��DMAһ�η������
#define USART2TXMAXLEN	1
static unsigned char mUsart2RxBuff[USART2RXMAXLEN];
static unsigned char mUsart2TxBuff[USART2TXMAXLEN];

/**************************** ����3���շ��ͻ����� ************************/
//DMA����Buff���ڴ�DMA�������ݵ�UsartBuff
#define USART3DMARXMAXLEN 256
#define USART3DMATXMAXLEN 254
static unsigned char mUsart3DMARxBuff[USART3DMARXMAXLEN];
static unsigned char mUsart3DMATxBuff[USART3DMATXMAXLEN];

#define USART3RXMAXLEN	256
//Ŀǰ�ǲ���Ҫ���ͻ������ģ��������ݻ����Ͽ���ͨ��DMAһ�η������
#define USART3TXMAXLEN	128
static unsigned char mUsart3RxBuff[USART3RXMAXLEN];
static unsigned char mUsart3TxBuff[USART3TXMAXLEN];

#ifdef USEUART4
/**************************** ����4���շ��ͻ����� ************************/
//DMA����Buff���ڴ�DMA�������ݵ�UsartBuff
#define USART4DMARXMAXLEN 254
#define USART4DMATXMAXLEN 254
static unsigned char mUsart4DMARxBuff[USART4DMARXMAXLEN];
static unsigned char mUsart4DMATxBuff[USART4DMATXMAXLEN];

#define USART4RXMAXLEN	512
//Ŀǰ�ǲ���Ҫ���ͻ������ģ��������ݻ����Ͽ���ͨ��DMAһ�η������
#define USART4TXMAXLEN	1
static unsigned char mUsart4RxBuff[USART4RXMAXLEN];
static unsigned char mUsart4TxBuff[USART4TXMAXLEN];
#endif

/**************************** ���ڲ��������������� ************************/
//����1PCͨѶ����
UsartInfo nCom1;
//����2 M72D GPRSģ��
UsartInfo nCom2;
//����3 TD3020 ������λģ��
UsartInfo nCom3;

#ifdef USEUART4
//����4 ����ģ��
UsartInfo nCom4;
#endif

/******************************* ����1��ʼ�� *******************************/
unsigned char ComPort1Init(uint32_t Bps,uint16_t Party)//ComPort1Init()
{
	//���ڳ�ʼ������
	nCom1.nCom = USART1;										//���ں�
	
	//����GPIO
	nCom1.nGpioPort.nRCC_GPIORxTx	= RCC_APB2Periph_GPIOA;		//TX,RX,GPIOʱ��
	nCom1.nGpioPort.nRxTxPort		= GPIOA;					//���շ����������ڶ˿�	
	nCom1.nGpioPort.nGPIO_Rx		= GPIO_Pin_10;				//���ڽ�������
	nCom1.nGpioPort.nGPIO_Tx		= GPIO_Pin_9;				//���ڷ�������
	
	//����ͨѶ����
	nCom1.nUsartBase.nBaudRate		= Bps;						//���ڲ�����	CBR_115200
	nCom1.nUsartBase.nStopBits		= 1;						//ֹͣλ
	switch(Party)
	{
		case UART_CONFIG_PAR_NONE: nCom1.nUsartBase.nParity = USART_Parity_No;	break;
		case UART_CONFIG_PAR_EVEN: nCom1.nUsartBase.nParity = USART_Parity_Even;	break;	//У��λ	żУ��
		case UART_CONFIG_PAR_ODD: nCom1.nUsartBase.nParity = USART_Parity_Odd;	break;
		default: nCom1.nUsartBase.nParity = USART_Parity_No;	break;
	}
	
	//�����������
	nCom1.nUsartHalPara.nUsartDRBase= USART1_DR_Base;			//���ڻ���ַ
	nCom1.nUsartHalPara.nRCC_USART	= RCC_APB2Periph_USART1;	//����ʱ��
	nCom1.nUsartHalPara.nUsartIRQ	= USART1_IRQn;				//USART�ж�ͨ��
	
	//����DMA����
	nCom1.nUsartDma.nDMARxIRQ		= DMA1_Channel5_IRQn;		//����DMA�����ж�
	nCom1.nUsartDma.nDMATxIRQ		= DMA1_Channel4_IRQn;		//����DMA�����ж�	
	nCom1.nUsartDma.nDMARxChannel	= DMA1_Channel5;			//����DMA����ͨ��
	nCom1.nUsartDma.nDMATxChannel	= DMA1_Channel4;			//����DMA����ͨ��
	nCom1.nUsartDma.nUsartDMARxlen	= USART1DMARXMAXLEN;		//����DMA���ճ���
	nCom1.nUsartDma.nUsartDMATxlen	= USART1DMATXMAXLEN;		//����DMA���ͳ���
	nCom1.nUsartDma.UsartDMARxBuff	= mUsart1DMARxBuff;			//����DMA���ջ�����
	nCom1.nUsartDma.UsartDMATxBuff	= mUsart1DMATxBuff;			//����DMA���ͻ�����

	//���ô���״̬
	nCom1.nUsartSt.IsTxEnd = 0;									//���������ɱ�־
	nCom1.nUsartSt.nRxByteTimeout = 0;							//�ַ��䳬ʱ������û��
	nCom1.nUsartSt.nRxFirstByteTimeout = 0;						//���ַ���ʱ������û��

	//���ڽ��շ��ͻ�����
	nCom1.nBuff.UsartRxBuff = mUsart1RxBuff;
	nCom1.nBuff.UsartTxBuff = mUsart1TxBuff;	
	nCom1.nBuff.rx_wr_index = 0;								//���ڽ���дָ��
	nCom1.nBuff.rx_rd_index = 0;								//���ڽ��ն�ָ��
	nCom1.nBuff.rx_counter  = 0;								//���ڽ���δ�����ݸ���
	nCom1.nBuff.rxMaxCnt	= USART1RXMAXLEN;					//���ڻ����С
	
	//��ʼ������
	UsartInitDMA(&nCom1);
	
	//��������1����
	StartUsartDMARx(&nCom1);	

	return 1;
}

/******************************* ����2��ʼ�� *******************************/
unsigned char ComPort2Init(uint32_t Bps,uint16_t Party)
{
	//���ڳ�ʼ������
	nCom2.nCom = USART2;										//���ں�
	
	//����GPIO
	nCom2.nGpioPort.nRCC_GPIORxTx	= RCC_APB2Periph_GPIOA;		//TX,RX,GPIOʱ��
	nCom2.nGpioPort.nRxTxPort		= GPIOA;					//���շ����������ڶ˿�	
	nCom2.nGpioPort.nGPIO_Rx		= GPIO_Pin_3;				//���ڽ�������
	nCom2.nGpioPort.nGPIO_Tx		= GPIO_Pin_2;				//���ڷ�������
	
	//����ͨѶ����
	nCom2.nUsartBase.nBaudRate		= Bps;						//���ڲ�����	M72D_Baudrate
	nCom2.nUsartBase.nStopBits		= 1;						//ֹͣλ
	switch(Party)
	{
		case UART_CONFIG_PAR_NONE: nCom2.nUsartBase.nParity = USART_Parity_No;	break;
		case UART_CONFIG_PAR_EVEN: nCom2.nUsartBase.nParity = USART_Parity_Even;	break;	//У��λ	żУ��
		case UART_CONFIG_PAR_ODD: nCom2.nUsartBase.nParity = USART_Parity_Odd;	break;
		default: nCom2.nUsartBase.nParity = USART_Parity_No;	break;
	}
	
	//�����������
	nCom2.nUsartHalPara.nUsartDRBase= USART2_DR_Base;			//���ڻ���ַ
	nCom2.nUsartHalPara.nRCC_USART	= RCC_APB1Periph_USART2;	//����ʱ��
	nCom2.nUsartHalPara.nUsartIRQ	= USART2_IRQn;				//USART�ж�ͨ��
	
	//����DMA����
	nCom2.nUsartDma.nDMARxIRQ		= DMA1_Channel6_IRQn;		//����DMA�����ж�
	nCom2.nUsartDma.nDMATxIRQ		= DMA1_Channel7_IRQn;		//����DMA�����ж�	
	nCom2.nUsartDma.nDMARxChannel	= DMA1_Channel6;			//����DMA����ͨ��
	nCom2.nUsartDma.nDMATxChannel	= DMA1_Channel7;			//����DMA����ͨ��
	nCom2.nUsartDma.nUsartDMARxlen	= USART2DMARXMAXLEN;		//����DMA���ճ���
	nCom2.nUsartDma.nUsartDMATxlen	= USART2DMATXMAXLEN;		//����DMA���ͳ���
	nCom2.nUsartDma.UsartDMARxBuff	= mUsart2DMARxBuff;			//����DMA���ջ�����
	nCom2.nUsartDma.UsartDMATxBuff	= mUsart2DMATxBuff;			//����DMA���ͻ�����

	//���ô���״̬
	nCom2.nUsartSt.IsTxEnd = 0;									//���������ɱ�־
	nCom2.nUsartSt.nRxByteTimeout = 0;							//�ַ��䳬ʱ������û��
	nCom2.nUsartSt.nRxFirstByteTimeout = 0;						//���ַ���ʱ������û��

	//���ڽ��շ��ͻ�����
	nCom2.nBuff.UsartRxBuff = mUsart2RxBuff;
	nCom2.nBuff.UsartTxBuff = mUsart2TxBuff;	
	nCom2.nBuff.rx_wr_index = 0;								//���ڽ���дָ��
	nCom2.nBuff.rx_rd_index = 0;								//���ڽ��ն�ָ��
	nCom2.nBuff.rx_counter  = 0;								//���ڽ���δ�����ݸ���
	nCom2.nBuff.rxMaxCnt	= USART2RXMAXLEN;					//���ڻ����С
	
	//��ʼ������
	UsartInitDMA(&nCom2);
	
	//��������1����
	StartUsartDMARx(&nCom2);	

	return 1;
}

/******************************* ����3��ʼ�� *******************************/
unsigned char TD3020ComInit(uint32_t Bps)
{
	//���ڳ�ʼ������
	nCom3.nCom = USART3;										//���ں�
	
	//����GPIO
	nCom3.nGpioPort.nRCC_GPIORxTx	= RCC_APB2Periph_GPIOB;		//TX,RX,GPIOʱ��
	nCom3.nGpioPort.nRxTxPort		= GPIOB;					//���շ����������ڶ˿�	
	nCom3.nGpioPort.nGPIO_Rx		= GPIO_Pin_11;				//���ڽ�������
	nCom3.nGpioPort.nGPIO_Tx		= GPIO_Pin_10;				//���ڷ�������
	
	//����ͨѶ����
	nCom3.nUsartBase.nBaudRate		= Bps;						//���ڲ�����CBR_9600
	nCom3.nUsartBase.nStopBits		= 1;						//ֹͣλ
	nCom3.nUsartBase.nParity		= 0;						//У��λ
	
	//�����������
	nCom3.nUsartHalPara.nUsartDRBase= USART3_DR_Base;			//���ڻ���ַ
	nCom3.nUsartHalPara.nRCC_USART	= RCC_APB1Periph_USART3;	//����ʱ��
	nCom3.nUsartHalPara.nUsartIRQ	= USART3_IRQn;				//USART�ж�ͨ��
	
	//����DMA����
	nCom3.nUsartDma.nDMARxIRQ		= DMA1_Channel3_IRQn;		//����DMA�����ж�
	nCom3.nUsartDma.nDMATxIRQ		= DMA1_Channel2_IRQn;		//����DMA�����ж�	
	nCom3.nUsartDma.nDMARxChannel	= DMA1_Channel3;			//����DMA����ͨ��
	nCom3.nUsartDma.nDMATxChannel	= DMA1_Channel2;			//����DMA����ͨ��
	nCom3.nUsartDma.nUsartDMARxlen	= USART3DMARXMAXLEN;		//����DMA���ճ���
	nCom3.nUsartDma.nUsartDMATxlen	= USART3DMATXMAXLEN;		//����DMA���ͳ���
	nCom3.nUsartDma.UsartDMARxBuff	= mUsart3DMARxBuff;			//����DMA���ջ�����
	nCom3.nUsartDma.UsartDMATxBuff	= mUsart3DMATxBuff;			//����DMA���ͻ�����

	//���ô���״̬
	nCom3.nUsartSt.IsTxEnd = 0;									//���������ɱ�־
	nCom3.nUsartSt.nRxByteTimeout = 0;							//�ַ��䳬ʱ������û��
	nCom3.nUsartSt.nRxFirstByteTimeout = 0;						//���ַ���ʱ������û��

	//���ڽ��շ��ͻ�����
	nCom3.nBuff.UsartRxBuff = mUsart3RxBuff;
	nCom3.nBuff.UsartTxBuff = mUsart3TxBuff;	
	nCom3.nBuff.rx_wr_index = 0;								//���ڽ���дָ��
	nCom3.nBuff.rx_rd_index = 0;								//���ڽ��ն�ָ��
	nCom3.nBuff.rx_counter  = 0;								//���ڽ���δ�����ݸ���
	nCom3.nBuff.rxMaxCnt	= USART3RXMAXLEN;					//���ڻ����С
	
	//��ʼ������
	UsartInitDMA(&nCom3);
	
	//��������1����
	StartUsartDMARx(&nCom3);	

	return 1;
}

#ifdef USEUART4
/******************************* ����3��ʼ�� *******************************/
unsigned char BlueToothComInit(uint32_t Bps)
{
	//���ڳ�ʼ������
	nCom4.nCom = UART4;										//���ں�
	
	//����GPIO
	nCom4.nGpioPort.nRCC_GPIORxTx	= RCC_APB2Periph_GPIOC;		//TX,RX,GPIOʱ��
	nCom4.nGpioPort.nRxTxPort		= GPIOC;					//���շ����������ڶ˿�	
	nCom4.nGpioPort.nGPIO_Rx		= GPIO_Pin_11;				//���ڽ�������
	nCom4.nGpioPort.nGPIO_Tx		= GPIO_Pin_10;				//���ڷ�������
	
	//����ͨѶ����
	nCom4.nUsartBase.nBaudRate		= Bps;						//���ڲ�����CBR_9600
	nCom4.nUsartBase.nStopBits		= 1;						//ֹͣλ
	nCom4.nUsartBase.nParity		= 0;						//У��λ
	
	//�����������
	nCom4.nUsartHalPara.nUsartDRBase= USART4_DR_Base;			//���ڻ���ַ
	nCom4.nUsartHalPara.nRCC_USART	= RCC_APB1Periph_UART4;		//����ʱ��
	nCom4.nUsartHalPara.nUsartIRQ	= UART4_IRQn;				//USART�ж�ͨ��
	
	//����DMA����
	nCom4.nUsartDma.nDMARxIRQ		= DMA2_Channel3_IRQn;		//����DMA�����ж�
	nCom4.nUsartDma.nDMATxIRQ		= DMA2_Channel4_5_IRQn;		//����DMA�����ж�	
	nCom4.nUsartDma.nDMARxChannel	= DMA2_Channel3;			//����DMA����ͨ��
	nCom4.nUsartDma.nDMATxChannel	= DMA2_Channel5;			//����DMA����ͨ��
	nCom4.nUsartDma.nUsartDMARxlen	= USART4DMARXMAXLEN;		//����DMA���ճ���
	nCom4.nUsartDma.nUsartDMATxlen	= USART4DMATXMAXLEN;		//����DMA���ͳ���
	nCom4.nUsartDma.UsartDMARxBuff	= mUsart4DMARxBuff;			//����DMA���ջ�����
	nCom4.nUsartDma.UsartDMATxBuff	= mUsart4DMATxBuff;			//����DMA���ͻ�����

	//���ô���״̬
	nCom4.nUsartSt.IsTxEnd = 0;									//���������ɱ�־
	nCom4.nUsartSt.nRxByteTimeout = 0;							//�ַ��䳬ʱ������û��
	nCom4.nUsartSt.nRxFirstByteTimeout = 0;						//���ַ���ʱ������û��

	//���ڽ��շ��ͻ�����
	nCom4.nBuff.UsartRxBuff = mUsart4RxBuff;
	nCom4.nBuff.UsartTxBuff = mUsart4TxBuff;	
	nCom4.nBuff.rx_wr_index = 0;								//���ڽ���дָ��
	nCom4.nBuff.rx_rd_index = 0;								//���ڽ��ն�ָ��
	nCom4.nBuff.rx_counter  = 0;								//���ڽ���δ�����ݸ���
	nCom4.nBuff.rxMaxCnt	= USART4RXMAXLEN;					//���ڻ����С
	
	//��ʼ������
	UsartInitDMA(&nCom4);
	
	//��������1����
	StartUsartDMARx(&nCom4);	

	return 1;
}
#endif

/*******************************************************************************************
*������ڻ���
********************************************************************************************/
void ComPortFlush(UsartInfo *info)
{
	UsartBuff nComBuff;
	
	nComBuff = info->nBuff;
	
	nComBuff.rx_wr_index = 0;				//���ڽ���дָ��
	nComBuff.rx_rd_index = 0;				//���ڽ��ն�ָ��
	nComBuff.rx_counter  = 0;				//���ڽ���δ�����ݸ���
	
	nComBuff = nComBuff;
}

/*******************************************************************************************
*���ڽ���
*******************************************************************************************/
unsigned short ComPortRead(UsartInfo *info,unsigned char *rxbuf,unsigned short len,unsigned short timeout)
{
	unsigned char ch,flag;
	unsigned short ret,rcnt,Dly;
	UsartBuff *nCombuff;
	
	nCombuff = &(info->nBuff);
	
	if(nCombuff->UsartRxBuff == NULL)return 0;
	if(len==0)return 0;

	ret = 0;
	Dly = timeout;
	for(rcnt=0;rcnt<len;rcnt++)
	{
		while(1)
		{
			flag = getchar(nCombuff,&ch);
			if(flag)
			{
				*rxbuf++ = ch;
				Dly = OS_TICKS_PER_SEC/100;//timeout;//�ַ��䳬ʱ���յ�һ���ֽں󽫳�ʱ����Ϊ10ms
				break;
			}
			else  if(timeout>0)
			{
				OSTimeDly(1);
				if(Dly-- == 0)//timeout=0 ������ʱ timeout>0����Ҫ�жϳ�ʱ
				{
					break;
				}
			}
            else
            {
                break;
            }
//			FeedWTD();			
		}
		if(flag)
		{
			ret++;
		}
		else
		{
			break;
		}
//		FeedWTD();
	}

	return ret;
}

/*******************************************************************************************
*���ڷ���
*******************************************************************************************/
void ComPortWrite(UsartInfo *info,unsigned char *rxbuf,unsigned short len)
{
	if((info->nCom != USART1)&&(info->nCom != USART2)
		&&(info->nCom != USART3)&&(info->nCom != UART4))return;
	if(len==0)return;

	//���÷�����ɱ�־
	info->nUsartSt.IsTxEnd = 0;
	
	StartUsartDMATx(info,rxbuf,len);
}

/*******************************************************************************************
*�ȴ����ڷ������
*����:	0����δ���
		1�������
*******************************************************************************************/
uint8_t ComPortWaitTxEnd(UsartInfo *info)
{
	if(info->nUsartSt.IsTxEnd == 0)return 0;
	while (!(info->nCom->SR & USART_FLAG_TXE));			//�ȴ����ͻ�������
	while (USART_GetFlagStatus(info->nCom, USART_FLAG_TC) == RESET);//�ȴ��������
	return 1;
}


/******************************************************************************
**UsartInitInfo	�����ڳ�ʼ������
**mComBuff		�����ڽ��ջ���������
*******************************************************************************/
void UsartInitDMA(UsartInfo *info)
{
    USART_InitTypeDef USART_InitStructure;	//���ڳ�ʼ���ṹ
    DMA_InitTypeDef DMA_InitStructure;		//DMA��ʼ���ṹ
    GPIO_InitTypeDef GPIO_InitStructure;	//GPIO��ʼ���ṹ
	NVIC_InitTypeDef NVIC_InitStructure;	//�жϿ��Ƴ�ʼ���ṹ

	//���ڻָ���Ĭ��ֵ
	USART_DeInit(info->nCom);

	//DMA���͵������ջָ���Ĭ��
	DMA_DeInit(info->nUsartDma.nDMARxChannel);
	DMA_DeInit(info->nUsartDma.nDMATxChannel);

    //DMAʱ��ʹ��
	if(info->nCom == UART4)//����4��DMAͨ����DMA2��
	{
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	}
    else
	{
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	}

    //GPIOʱ��ʹ��
    RCC_APB2PeriphClockCmd(info->nGpioPort.nRCC_GPIORxTx | RCC_APB2Periph_AFIO, ENABLE);
	if(info->nCom == USART1)
	{
		RCC_APB2PeriphClockCmd(info->nUsartHalPara.nRCC_USART, ENABLE);
	}
	else	//����2,3,4,5��APB1������
	{
		RCC_APB1PeriphClockCmd(info->nUsartHalPara.nRCC_USART, ENABLE);
	}

	//���ڷ�����������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = info->nGpioPort.nGPIO_Tx;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(info->nGpioPort.nRxTxPort, &GPIO_InitStructure);

	//���ڽ������ų�ʼ��
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = info->nGpioPort.nGPIO_Rx;
    GPIO_Init(info->nGpioPort.nRxTxPort, &GPIO_InitStructure);
			
	//�ж����ȼ�����
    NVIC_SetPriorityGrouping(6); 
    
	//DMA����ͨ����ʼ��
    NVIC_SetPriority(info->nUsartDma.nDMATxIRQ, 0x01); 
    NVIC_EnableIRQ(info->nUsartDma.nDMATxIRQ);

	//DMA����ͨ����ʼ��
    NVIC_SetPriority(info->nUsartDma.nDMARxIRQ, 0x02); 
    NVIC_EnableIRQ(info->nUsartDma.nDMARxIRQ);

	//�����жϳ�ʼ��
    NVIC_InitStructure.NVIC_IRQChannel = info->nUsartHalPara.nUsartIRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//����DMA��������
    DMA_DeInit(info->nUsartDma.nDMATxChannel);
    DMA_InitStructure.DMA_PeripheralBaseAddr = info->nUsartHalPara.nUsartDRBase;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(info->nUsartDma.UsartDMATxBuff);
    DMA_InitStructure.DMA_BufferSize = info->nUsartDma.nUsartDMATxlen;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(info->nUsartDma.nDMATxChannel, &DMA_InitStructure);
    
  	//ʹ��DMA��������ж�
    DMA_ITConfig(info->nUsartDma.nDMATxChannel, DMA_IT_TC, ENABLE); 

	//����DMA��������
    DMA_DeInit(info->nUsartDma.nDMARxChannel);
    DMA_InitStructure.DMA_PeripheralBaseAddr = info->nUsartHalPara.nUsartDRBase;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(info->nUsartDma.UsartDMARxBuff);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = info->nUsartDma.nUsartDMARxlen;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(info->nUsartDma.nDMARxChannel, &DMA_InitStructure);
    
	//ʹ��DMA��������ж�
    DMA_ITConfig(info->nUsartDma.nDMARxChannel, DMA_IT_TC, ENABLE);  

	//����ͨѶ��������
    USART_InitStructure.USART_BaudRate = info->nUsartBase.nBaudRate;
	if(info->nUsartBase.nParity != 0)
	{
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;  //Word Length = 8 Bits
	}
	else
	{
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //Word Length = 8 Bits
	}
    USART_InitStructure.USART_StopBits = info->nUsartBase.nStopBits;  //Stop Bit
    USART_InitStructure.USART_Parity = info->nUsartBase.nParity;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(info->nCom, &USART_InitStructure);

	//����DMA����ͨ��ʹ��
    USART_DMACmd(info->nCom, USART_DMAReq_Tx, ENABLE);
    
	//����DMA����ͨ��ʹ��
    USART_DMACmd(info->nCom, USART_DMAReq_Rx , ENABLE);
    
	//����DMA���ͽ�ֹ
    DMA_Cmd(info->nUsartDma.nDMATxChannel, DISABLE);
    
	//����DMA���ս�ֹ
    DMA_Cmd(info->nUsartDma.nDMARxChannel, DISABLE);
    
	//ʹ�ܴ���
    USART_Cmd(info->nCom, ENABLE);
	
	//ʹ�ܴ��ڿ����ж�
	USART_ITConfig(info->nCom, USART_IT_IDLE, ENABLE);

}

/**************************************************************************************
�Ӵ��ڻ�������һ���ֽ�
***************************************************************************************/
unsigned char getchar(UsartBuff *nComBuff,unsigned char *data)
{
    unsigned char tmp ;

//	û�����ݷ���0	
    if(nComBuff->rx_counter==0) 
	{
        return 0x0;   
    }

//	��ȡ����������
    tmp = nComBuff->UsartRxBuff[nComBuff->rx_rd_index];
	
//	��ָ���1
	nComBuff->rx_rd_index++;
	
//	��Ƕ��������ˣ��������������ĩβ����ͷ��ʼ��
    if(nComBuff->rx_rd_index == nComBuff->rxMaxCnt)
	{	nComBuff->rx_rd_index=0;	}

//	δ��������һ
	nComBuff->rx_counter--;

//	��������
    *data = tmp;

    return 0x1;
}

/******************************************************************************
**����DMA����
******************************************************************************/
void StartUsartDMARx(UsartInfo *info)
{
    //�ر�DMA
    DMA_Cmd(info->nUsartDma.nDMARxChannel, DISABLE);
    
	//д��DMA���ո���
    (info->nUsartDma.nDMARxChannel)->CNDTR = info->nUsartDma.nUsartDMARxlen;
    
    //����DAM����
    DMA_Cmd(info->nUsartDma.nDMARxChannel, ENABLE);
}

/******************************************************************************
**����DMA��������
******************************************************************************/
void StartUsartDMATx(UsartInfo *info,unsigned char *pBuff, unsigned short length)
{
    if(length > 0)
	{
	    //�ر�DMA
	    DMA_Cmd(info->nUsartDma.nDMATxChannel, DISABLE);
	    
	    //����������д�뷢�ͻ�����
		memcpy(info->nUsartDma.UsartDMATxBuff,pBuff,length);
		
		//���÷��ͳ���
	    (info->nUsartDma.nDMATxChannel)->CNDTR = length;
	    
	    //��������
	    DMA_Cmd(info->nUsartDma.nDMATxChannel, ENABLE);
	}
}

/******************************************************************************
**����DMA���ݣ��ڴ��ڿ����жϻ�DMA�����ж�ʱ����
******************************************************************************/
unsigned short UsartDMARx(UsartInfo *info)
{
    unsigned short CurrDataCounterEnd = 0;
    unsigned short i,length = 0;
	
	unsigned char *ptr;
	UsartBuff *nUsartBuf;
	

//	��ȡδ���ո���
	CurrDataCounterEnd = DMA_GetCurrDataCounter(info->nUsartDma.nDMARxChannel);

    if (CurrDataCounterEnd  == info->nUsartDma.nUsartDMARxlen) 
	{
		//����DMA����
		StartUsartDMARx(info);
		return 0;
	}
    length = info->nUsartDma.nUsartDMARxlen - CurrDataCounterEnd ;
	
//	if(length>MaxLen)length = MaxLen;

//	memcpy(pBuff,info->nUsartDma.UsartDMARxBuff,length);
	
//	�����ݳ�DMA���������������ڻ�����
	ptr = info->nUsartDma.UsartDMARxBuff;
	nUsartBuf = &(info->nBuff);
	
	for(i=0;i<length;i++)
	{
		if(++nUsartBuf->rx_counter < nUsartBuf->rxMaxCnt)//���������δ�������
		{
			nUsartBuf->UsartRxBuff[nUsartBuf->rx_wr_index] = *ptr++;
			if(++nUsartBuf->rx_wr_index >= nUsartBuf->rxMaxCnt)
			{nUsartBuf->rx_wr_index = 0;}
		}
		else		//������������������ݣ�
		{
			break;
		}
	}

//	���ý��ճ���
	DMA_SetCurrDataCounter(info->nUsartDma.nDMARxChannel,info->nUsartDma.nUsartDMARxlen);

//	�������ݺ���������
	StartUsartDMARx(info);
	
    return length;
}

