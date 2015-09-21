
#include <string.h>
#include "includes.h"

//#define USEUART4

/************************** 串口1接收发送缓冲区 ****************************/
//DMA接收Buff用于从DMA接收数据到UsartBuff
#define USART1DMARXMAXLEN 128
#define USART1DMATXMAXLEN 128
static unsigned char mUsart1DMARxBuff[USART1DMARXMAXLEN];
static unsigned char mUsart1DMATxBuff[USART1DMATXMAXLEN];

#define USART1RXMAXLEN	64
//目前是不需要发送缓冲区的，所有数据基本上可以通过DMA一次发送完成
#define USART1TXMAXLEN	1	
static unsigned char mUsart1RxBuff[USART1RXMAXLEN];
static unsigned char mUsart1TxBuff[USART1TXMAXLEN];

/**************************** 串口2接收发送缓冲区 ************************/
//DMA接收Buff用于从DMA接收数据到UsartBuff
#define USART2DMARXMAXLEN 256
#define USART2DMATXMAXLEN 254
static unsigned char mUsart2DMARxBuff[USART2DMARXMAXLEN];
static unsigned char mUsart2DMATxBuff[USART2DMATXMAXLEN];

#define USART2RXMAXLEN	256
//目前是不需要发送缓冲区的，所有数据基本上可以通过DMA一次发送完成
#define USART2TXMAXLEN	1
static unsigned char mUsart2RxBuff[USART2RXMAXLEN];
static unsigned char mUsart2TxBuff[USART2TXMAXLEN];

/**************************** 串口3接收发送缓冲区 ************************/
//DMA接收Buff用于从DMA接收数据到UsartBuff
#define USART3DMARXMAXLEN 256
#define USART3DMATXMAXLEN 254
static unsigned char mUsart3DMARxBuff[USART3DMARXMAXLEN];
static unsigned char mUsart3DMATxBuff[USART3DMATXMAXLEN];

#define USART3RXMAXLEN	256
//目前是不需要发送缓冲区的，所有数据基本上可以通过DMA一次发送完成
#define USART3TXMAXLEN	128
static unsigned char mUsart3RxBuff[USART3RXMAXLEN];
static unsigned char mUsart3TxBuff[USART3TXMAXLEN];

#ifdef USEUART4
/**************************** 串口4接收发送缓冲区 ************************/
//DMA接收Buff用于从DMA接收数据到UsartBuff
#define USART4DMARXMAXLEN 254
#define USART4DMATXMAXLEN 254
static unsigned char mUsart4DMARxBuff[USART4DMARXMAXLEN];
static unsigned char mUsart4DMATxBuff[USART4DMATXMAXLEN];

#define USART4RXMAXLEN	512
//目前是不需要发送缓冲区的，所有数据基本上可以通过DMA一次发送完成
#define USART4TXMAXLEN	1
static unsigned char mUsart4RxBuff[USART4RXMAXLEN];
static unsigned char mUsart4TxBuff[USART4TXMAXLEN];
#endif

/**************************** 串口参数即缓冲区定义 ************************/
//串口1PC通讯串口
UsartInfo nCom1;
//串口2 M72D GPRS模块
UsartInfo nCom2;
//串口3 TD3020 北斗定位模块
UsartInfo nCom3;

#ifdef USEUART4
//串口4 蓝牙模块
UsartInfo nCom4;
#endif

/******************************* 串口1初始化 *******************************/
unsigned char ComPort1Init(uint32_t Bps,uint16_t Party)//ComPort1Init()
{
	//串口初始化参数
	nCom1.nCom = USART1;										//串口号
	
	//串口GPIO
	nCom1.nGpioPort.nRCC_GPIORxTx	= RCC_APB2Periph_GPIOA;		//TX,RX,GPIO时钟
	nCom1.nGpioPort.nRxTxPort		= GPIOA;					//接收发送引脚所在端口	
	nCom1.nGpioPort.nGPIO_Rx		= GPIO_Pin_10;				//串口接收引脚
	nCom1.nGpioPort.nGPIO_Tx		= GPIO_Pin_9;				//串口发送引脚
	
	//串口通讯参数
	nCom1.nUsartBase.nBaudRate		= Bps;						//串口波特率	CBR_115200
	nCom1.nUsartBase.nStopBits		= 1;						//停止位
	switch(Party)
	{
		case UART_CONFIG_PAR_NONE: nCom1.nUsartBase.nParity = USART_Parity_No;	break;
		case UART_CONFIG_PAR_EVEN: nCom1.nUsartBase.nParity = USART_Parity_Even;	break;	//校验位	偶校验
		case UART_CONFIG_PAR_ODD: nCom1.nUsartBase.nParity = USART_Parity_Odd;	break;
		default: nCom1.nUsartBase.nParity = USART_Parity_No;	break;
	}
	
	//串口物理参数
	nCom1.nUsartHalPara.nUsartDRBase= USART1_DR_Base;			//串口基地址
	nCom1.nUsartHalPara.nRCC_USART	= RCC_APB2Periph_USART1;	//串口时钟
	nCom1.nUsartHalPara.nUsartIRQ	= USART1_IRQn;				//USART中断通道
	
	//串口DMA参数
	nCom1.nUsartDma.nDMARxIRQ		= DMA1_Channel5_IRQn;		//串口DMA接收中断
	nCom1.nUsartDma.nDMATxIRQ		= DMA1_Channel4_IRQn;		//串口DMA发送中断	
	nCom1.nUsartDma.nDMARxChannel	= DMA1_Channel5;			//串口DMA接收通道
	nCom1.nUsartDma.nDMATxChannel	= DMA1_Channel4;			//串口DMA发送通道
	nCom1.nUsartDma.nUsartDMARxlen	= USART1DMARXMAXLEN;		//串口DMA接收长度
	nCom1.nUsartDma.nUsartDMATxlen	= USART1DMATXMAXLEN;		//串口DMA发送长度
	nCom1.nUsartDma.UsartDMARxBuff	= mUsart1DMARxBuff;			//串口DMA接收缓冲区
	nCom1.nUsartDma.UsartDMATxBuff	= mUsart1DMATxBuff;			//串口DMA发送缓冲区

	//重置串口状态
	nCom1.nUsartSt.IsTxEnd = 0;									//清除发送完成标志
	nCom1.nUsartSt.nRxByteTimeout = 0;							//字符间超时，现在没用
	nCom1.nUsartSt.nRxFirstByteTimeout = 0;						//首字符超时，现在没用

	//串口接收发送缓冲区
	nCom1.nBuff.UsartRxBuff = mUsart1RxBuff;
	nCom1.nBuff.UsartTxBuff = mUsart1TxBuff;	
	nCom1.nBuff.rx_wr_index = 0;								//串口接收写指针
	nCom1.nBuff.rx_rd_index = 0;								//串口接收读指针
	nCom1.nBuff.rx_counter  = 0;								//串口接收未读数据个数
	nCom1.nBuff.rxMaxCnt	= USART1RXMAXLEN;					//串口缓存大小
	
	//初始化串口
	UsartInitDMA(&nCom1);
	
	//启动串口1接收
	StartUsartDMARx(&nCom1);	

	return 1;
}

/******************************* 串口2初始化 *******************************/
unsigned char ComPort2Init(uint32_t Bps,uint16_t Party)
{
	//串口初始化参数
	nCom2.nCom = USART2;										//串口号
	
	//串口GPIO
	nCom2.nGpioPort.nRCC_GPIORxTx	= RCC_APB2Periph_GPIOA;		//TX,RX,GPIO时钟
	nCom2.nGpioPort.nRxTxPort		= GPIOA;					//接收发送引脚所在端口	
	nCom2.nGpioPort.nGPIO_Rx		= GPIO_Pin_3;				//串口接收引脚
	nCom2.nGpioPort.nGPIO_Tx		= GPIO_Pin_2;				//串口发送引脚
	
	//串口通讯参数
	nCom2.nUsartBase.nBaudRate		= Bps;						//串口波特率	M72D_Baudrate
	nCom2.nUsartBase.nStopBits		= 1;						//停止位
	switch(Party)
	{
		case UART_CONFIG_PAR_NONE: nCom2.nUsartBase.nParity = USART_Parity_No;	break;
		case UART_CONFIG_PAR_EVEN: nCom2.nUsartBase.nParity = USART_Parity_Even;	break;	//校验位	偶校验
		case UART_CONFIG_PAR_ODD: nCom2.nUsartBase.nParity = USART_Parity_Odd;	break;
		default: nCom2.nUsartBase.nParity = USART_Parity_No;	break;
	}
	
	//串口物理参数
	nCom2.nUsartHalPara.nUsartDRBase= USART2_DR_Base;			//串口基地址
	nCom2.nUsartHalPara.nRCC_USART	= RCC_APB1Periph_USART2;	//串口时钟
	nCom2.nUsartHalPara.nUsartIRQ	= USART2_IRQn;				//USART中断通道
	
	//串口DMA参数
	nCom2.nUsartDma.nDMARxIRQ		= DMA1_Channel6_IRQn;		//串口DMA接收中断
	nCom2.nUsartDma.nDMATxIRQ		= DMA1_Channel7_IRQn;		//串口DMA发送中断	
	nCom2.nUsartDma.nDMARxChannel	= DMA1_Channel6;			//串口DMA接收通道
	nCom2.nUsartDma.nDMATxChannel	= DMA1_Channel7;			//串口DMA发送通道
	nCom2.nUsartDma.nUsartDMARxlen	= USART2DMARXMAXLEN;		//串口DMA接收长度
	nCom2.nUsartDma.nUsartDMATxlen	= USART2DMATXMAXLEN;		//串口DMA发送长度
	nCom2.nUsartDma.UsartDMARxBuff	= mUsart2DMARxBuff;			//串口DMA接收缓冲区
	nCom2.nUsartDma.UsartDMATxBuff	= mUsart2DMATxBuff;			//串口DMA发送缓冲区

	//重置串口状态
	nCom2.nUsartSt.IsTxEnd = 0;									//清除发送完成标志
	nCom2.nUsartSt.nRxByteTimeout = 0;							//字符间超时，现在没用
	nCom2.nUsartSt.nRxFirstByteTimeout = 0;						//首字符超时，现在没用

	//串口接收发送缓冲区
	nCom2.nBuff.UsartRxBuff = mUsart2RxBuff;
	nCom2.nBuff.UsartTxBuff = mUsart2TxBuff;	
	nCom2.nBuff.rx_wr_index = 0;								//串口接收写指针
	nCom2.nBuff.rx_rd_index = 0;								//串口接收读指针
	nCom2.nBuff.rx_counter  = 0;								//串口接收未读数据个数
	nCom2.nBuff.rxMaxCnt	= USART2RXMAXLEN;					//串口缓存大小
	
	//初始化串口
	UsartInitDMA(&nCom2);
	
	//启动串口1接收
	StartUsartDMARx(&nCom2);	

	return 1;
}

/******************************* 串口3初始化 *******************************/
unsigned char TD3020ComInit(uint32_t Bps)
{
	//串口初始化参数
	nCom3.nCom = USART3;										//串口号
	
	//串口GPIO
	nCom3.nGpioPort.nRCC_GPIORxTx	= RCC_APB2Periph_GPIOB;		//TX,RX,GPIO时钟
	nCom3.nGpioPort.nRxTxPort		= GPIOB;					//接收发送引脚所在端口	
	nCom3.nGpioPort.nGPIO_Rx		= GPIO_Pin_11;				//串口接收引脚
	nCom3.nGpioPort.nGPIO_Tx		= GPIO_Pin_10;				//串口发送引脚
	
	//串口通讯参数
	nCom3.nUsartBase.nBaudRate		= Bps;						//串口波特率CBR_9600
	nCom3.nUsartBase.nStopBits		= 1;						//停止位
	nCom3.nUsartBase.nParity		= 0;						//校验位
	
	//串口物理参数
	nCom3.nUsartHalPara.nUsartDRBase= USART3_DR_Base;			//串口基地址
	nCom3.nUsartHalPara.nRCC_USART	= RCC_APB1Periph_USART3;	//串口时钟
	nCom3.nUsartHalPara.nUsartIRQ	= USART3_IRQn;				//USART中断通道
	
	//串口DMA参数
	nCom3.nUsartDma.nDMARxIRQ		= DMA1_Channel3_IRQn;		//串口DMA接收中断
	nCom3.nUsartDma.nDMATxIRQ		= DMA1_Channel2_IRQn;		//串口DMA发送中断	
	nCom3.nUsartDma.nDMARxChannel	= DMA1_Channel3;			//串口DMA接收通道
	nCom3.nUsartDma.nDMATxChannel	= DMA1_Channel2;			//串口DMA发送通道
	nCom3.nUsartDma.nUsartDMARxlen	= USART3DMARXMAXLEN;		//串口DMA接收长度
	nCom3.nUsartDma.nUsartDMATxlen	= USART3DMATXMAXLEN;		//串口DMA发送长度
	nCom3.nUsartDma.UsartDMARxBuff	= mUsart3DMARxBuff;			//串口DMA接收缓冲区
	nCom3.nUsartDma.UsartDMATxBuff	= mUsart3DMATxBuff;			//串口DMA发送缓冲区

	//重置串口状态
	nCom3.nUsartSt.IsTxEnd = 0;									//清除发送完成标志
	nCom3.nUsartSt.nRxByteTimeout = 0;							//字符间超时，现在没用
	nCom3.nUsartSt.nRxFirstByteTimeout = 0;						//首字符超时，现在没用

	//串口接收发送缓冲区
	nCom3.nBuff.UsartRxBuff = mUsart3RxBuff;
	nCom3.nBuff.UsartTxBuff = mUsart3TxBuff;	
	nCom3.nBuff.rx_wr_index = 0;								//串口接收写指针
	nCom3.nBuff.rx_rd_index = 0;								//串口接收读指针
	nCom3.nBuff.rx_counter  = 0;								//串口接收未读数据个数
	nCom3.nBuff.rxMaxCnt	= USART3RXMAXLEN;					//串口缓存大小
	
	//初始化串口
	UsartInitDMA(&nCom3);
	
	//启动串口1接收
	StartUsartDMARx(&nCom3);	

	return 1;
}

#ifdef USEUART4
/******************************* 串口3初始化 *******************************/
unsigned char BlueToothComInit(uint32_t Bps)
{
	//串口初始化参数
	nCom4.nCom = UART4;										//串口号
	
	//串口GPIO
	nCom4.nGpioPort.nRCC_GPIORxTx	= RCC_APB2Periph_GPIOC;		//TX,RX,GPIO时钟
	nCom4.nGpioPort.nRxTxPort		= GPIOC;					//接收发送引脚所在端口	
	nCom4.nGpioPort.nGPIO_Rx		= GPIO_Pin_11;				//串口接收引脚
	nCom4.nGpioPort.nGPIO_Tx		= GPIO_Pin_10;				//串口发送引脚
	
	//串口通讯参数
	nCom4.nUsartBase.nBaudRate		= Bps;						//串口波特率CBR_9600
	nCom4.nUsartBase.nStopBits		= 1;						//停止位
	nCom4.nUsartBase.nParity		= 0;						//校验位
	
	//串口物理参数
	nCom4.nUsartHalPara.nUsartDRBase= USART4_DR_Base;			//串口基地址
	nCom4.nUsartHalPara.nRCC_USART	= RCC_APB1Periph_UART4;		//串口时钟
	nCom4.nUsartHalPara.nUsartIRQ	= UART4_IRQn;				//USART中断通道
	
	//串口DMA参数
	nCom4.nUsartDma.nDMARxIRQ		= DMA2_Channel3_IRQn;		//串口DMA接收中断
	nCom4.nUsartDma.nDMATxIRQ		= DMA2_Channel4_5_IRQn;		//串口DMA发送中断	
	nCom4.nUsartDma.nDMARxChannel	= DMA2_Channel3;			//串口DMA接收通道
	nCom4.nUsartDma.nDMATxChannel	= DMA2_Channel5;			//串口DMA发送通道
	nCom4.nUsartDma.nUsartDMARxlen	= USART4DMARXMAXLEN;		//串口DMA接收长度
	nCom4.nUsartDma.nUsartDMATxlen	= USART4DMATXMAXLEN;		//串口DMA发送长度
	nCom4.nUsartDma.UsartDMARxBuff	= mUsart4DMARxBuff;			//串口DMA接收缓冲区
	nCom4.nUsartDma.UsartDMATxBuff	= mUsart4DMATxBuff;			//串口DMA发送缓冲区

	//重置串口状态
	nCom4.nUsartSt.IsTxEnd = 0;									//清除发送完成标志
	nCom4.nUsartSt.nRxByteTimeout = 0;							//字符间超时，现在没用
	nCom4.nUsartSt.nRxFirstByteTimeout = 0;						//首字符超时，现在没用

	//串口接收发送缓冲区
	nCom4.nBuff.UsartRxBuff = mUsart4RxBuff;
	nCom4.nBuff.UsartTxBuff = mUsart4TxBuff;	
	nCom4.nBuff.rx_wr_index = 0;								//串口接收写指针
	nCom4.nBuff.rx_rd_index = 0;								//串口接收读指针
	nCom4.nBuff.rx_counter  = 0;								//串口接收未读数据个数
	nCom4.nBuff.rxMaxCnt	= USART4RXMAXLEN;					//串口缓存大小
	
	//初始化串口
	UsartInitDMA(&nCom4);
	
	//启动串口1接收
	StartUsartDMARx(&nCom4);	

	return 1;
}
#endif

/*******************************************************************************************
*清除串口缓存
********************************************************************************************/
void ComPortFlush(UsartInfo *info)
{
	UsartBuff nComBuff;
	
	nComBuff = info->nBuff;
	
	nComBuff.rx_wr_index = 0;				//串口接收写指针
	nComBuff.rx_rd_index = 0;				//串口接收读指针
	nComBuff.rx_counter  = 0;				//串口接收未读数据个数
	
	nComBuff = nComBuff;
}

/*******************************************************************************************
*串口接收
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
				Dly = OS_TICKS_PER_SEC/100;//timeout;//字符间超时，收到一个字节后将超时缩短为10ms
				break;
			}
			else  if(timeout>0)
			{
				OSTimeDly(1);
				if(Dly-- == 0)//timeout=0 立即超时 timeout>0才需要判断超时
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
*串口发送
*******************************************************************************************/
void ComPortWrite(UsartInfo *info,unsigned char *rxbuf,unsigned short len)
{
	if((info->nCom != USART1)&&(info->nCom != USART2)
		&&(info->nCom != USART3)&&(info->nCom != UART4))return;
	if(len==0)return;

	//重置发送完成标志
	info->nUsartSt.IsTxEnd = 0;
	
	StartUsartDMATx(info,rxbuf,len);
}

/*******************************************************************************************
*等待串口发送完成
*返回:	0发送未完成
		1发送完成
*******************************************************************************************/
uint8_t ComPortWaitTxEnd(UsartInfo *info)
{
	if(info->nUsartSt.IsTxEnd == 0)return 0;
	while (!(info->nCom->SR & USART_FLAG_TXE));			//等待发送缓冲器空
	while (USART_GetFlagStatus(info->nCom, USART_FLAG_TC) == RESET);//等待发送完成
	return 1;
}


/******************************************************************************
**UsartInitInfo	：串口初始化参数
**mComBuff		：串口接收缓冲区属性
*******************************************************************************/
void UsartInitDMA(UsartInfo *info)
{
    USART_InitTypeDef USART_InitStructure;	//串口初始化结构
    DMA_InitTypeDef DMA_InitStructure;		//DMA初始化结构
    GPIO_InitTypeDef GPIO_InitStructure;	//GPIO初始化结构
	NVIC_InitTypeDef NVIC_InitStructure;	//中断控制初始化结构

	//串口恢复到默认值
	USART_DeInit(info->nCom);

	//DMA发送到、接收恢复到默认
	DMA_DeInit(info->nUsartDma.nDMARxChannel);
	DMA_DeInit(info->nUsartDma.nDMATxChannel);

    //DMA时钟使能
	if(info->nCom == UART4)//串口4的DMA通道在DMA2上
	{
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	}
    else
	{
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	}

    //GPIO时钟使能
    RCC_APB2PeriphClockCmd(info->nGpioPort.nRCC_GPIORxTx | RCC_APB2Periph_AFIO, ENABLE);
	if(info->nCom == USART1)
	{
		RCC_APB2PeriphClockCmd(info->nUsartHalPara.nRCC_USART, ENABLE);
	}
	else	//串口2,3,4,5在APB1总线上
	{
		RCC_APB1PeriphClockCmd(info->nUsartHalPara.nRCC_USART, ENABLE);
	}

	//串口发送引脚配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = info->nGpioPort.nGPIO_Tx;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(info->nGpioPort.nRxTxPort, &GPIO_InitStructure);

	//串口接收引脚初始化
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = info->nGpioPort.nGPIO_Rx;
    GPIO_Init(info->nGpioPort.nRxTxPort, &GPIO_InitStructure);
			
	//中断优先级配置
    NVIC_SetPriorityGrouping(6); 
    
	//DMA发送通道初始化
    NVIC_SetPriority(info->nUsartDma.nDMATxIRQ, 0x01); 
    NVIC_EnableIRQ(info->nUsartDma.nDMATxIRQ);

	//DMA接收通道初始化
    NVIC_SetPriority(info->nUsartDma.nDMARxIRQ, 0x02); 
    NVIC_EnableIRQ(info->nUsartDma.nDMARxIRQ);

	//串口中断初始化
    NVIC_InitStructure.NVIC_IRQChannel = info->nUsartHalPara.nUsartIRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//串口DMA发送配置
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
    
  	//使能DMA发送完成中断
    DMA_ITConfig(info->nUsartDma.nDMATxChannel, DMA_IT_TC, ENABLE); 

	//串口DMA接收配置
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
    
	//使能DMA接收完成中断
    DMA_ITConfig(info->nUsartDma.nDMARxChannel, DMA_IT_TC, ENABLE);  

	//串口通讯参数配置
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

	//串口DMA发送通道使能
    USART_DMACmd(info->nCom, USART_DMAReq_Tx, ENABLE);
    
	//串口DMA接收通道使能
    USART_DMACmd(info->nCom, USART_DMAReq_Rx , ENABLE);
    
	//串口DMA发送禁止
    DMA_Cmd(info->nUsartDma.nDMATxChannel, DISABLE);
    
	//串口DMA接收禁止
    DMA_Cmd(info->nUsartDma.nDMARxChannel, DISABLE);
    
	//使能串口
    USART_Cmd(info->nCom, ENABLE);
	
	//使能串口空闲中断
	USART_ITConfig(info->nCom, USART_IT_IDLE, ENABLE);

}

/**************************************************************************************
从串口缓冲区读一个字节
***************************************************************************************/
unsigned char getchar(UsartBuff *nComBuff,unsigned char *data)
{
    unsigned char tmp ;

//	没有数据返回0	
    if(nComBuff->rx_counter==0) 
	{
        return 0x0;   
    }

//	读取缓冲区数据
    tmp = nComBuff->UsartRxBuff[nComBuff->rx_rd_index];
	
//	读指针加1
	nComBuff->rx_rd_index++;
	
//	标记读到哪里了，如果读到缓冲区末尾就重头开始读
    if(nComBuff->rx_rd_index == nComBuff->rxMaxCnt)
	{	nComBuff->rx_rd_index=0;	}

//	未读个数减一
	nComBuff->rx_counter--;

//	返回数据
    *data = tmp;

    return 0x1;
}

/******************************************************************************
**启动DMA接收
******************************************************************************/
void StartUsartDMARx(UsartInfo *info)
{
    //关闭DMA
    DMA_Cmd(info->nUsartDma.nDMARxChannel, DISABLE);
    
	//写入DMA接收个数
    (info->nUsartDma.nDMARxChannel)->CNDTR = info->nUsartDma.nUsartDMARxlen;
    
    //启动DAM接收
    DMA_Cmd(info->nUsartDma.nDMARxChannel, ENABLE);
}

/******************************************************************************
**启动DMA发送数据
******************************************************************************/
void StartUsartDMATx(UsartInfo *info,unsigned char *pBuff, unsigned short length)
{
    if(length > 0)
	{
	    //关闭DMA
	    DMA_Cmd(info->nUsartDma.nDMATxChannel, DISABLE);
	    
	    //待发送数据写入发送缓冲区
		memcpy(info->nUsartDma.UsartDMATxBuff,pBuff,length);
		
		//设置发送长度
	    (info->nUsartDma.nDMATxChannel)->CNDTR = length;
	    
	    //启动发送
	    DMA_Cmd(info->nUsartDma.nDMATxChannel, ENABLE);
	}
}

/******************************************************************************
**接收DMA数据，在串口空闲中断或DMA接收中断时调用
******************************************************************************/
unsigned short UsartDMARx(UsartInfo *info)
{
    unsigned short CurrDataCounterEnd = 0;
    unsigned short i,length = 0;
	
	unsigned char *ptr;
	UsartBuff *nUsartBuf;
	

//	获取未接收个数
	CurrDataCounterEnd = DMA_GetCurrDataCounter(info->nUsartDma.nDMARxChannel);

    if (CurrDataCounterEnd  == info->nUsartDma.nUsartDMARxlen) 
	{
		//启动DMA接收
		StartUsartDMARx(info);
		return 0;
	}
    length = info->nUsartDma.nUsartDMARxlen - CurrDataCounterEnd ;
	
//	if(length>MaxLen)length = MaxLen;

//	memcpy(pBuff,info->nUsartDma.UsartDMARxBuff,length);
	
//	将数据冲DMA缓冲区拷贝到串口缓冲区
	ptr = info->nUsartDma.UsartDMARxBuff;
	nUsartBuf = &(info->nBuff);
	
	for(i=0;i<length;i++)
	{
		if(++nUsartBuf->rx_counter < nUsartBuf->rxMaxCnt)//如果缓冲区未满则接收
		{
			nUsartBuf->UsartRxBuff[nUsartBuf->rx_wr_index] = *ptr++;
			if(++nUsartBuf->rx_wr_index >= nUsartBuf->rxMaxCnt)
			{nUsartBuf->rx_wr_index = 0;}
		}
		else		//如果缓冲区满则丢弃数据？
		{
			break;
		}
	}

//	重置接收长度
	DMA_SetCurrDataCounter(info->nUsartDma.nDMARxChannel,info->nUsartDma.nUsartDMARxlen);

//	读完数据后启动接收
	StartUsartDMARx(info);
	
    return length;
}

