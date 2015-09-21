/******************** (C) COPYRIGHT 2008 boost ********************
* File Name          : usart.h
* Author             : csz
* Version            : V1.0.0
* Date               : 2014年5月4日
* Description        : Header for usart.c module
********************************************************************************/

#ifndef __USART_H
#define __USART_H

#include "includes.h"
/*************************** 串口通信波特率参数 ********************************/
#define CBR_2400        2400
#define CBR_4800        4800
#define CBR_9600        9600
#define CBR_14400       14400
#define CBR_19200       19200
#define CBR_38400		38400
#define	CBR_57600		57600
#define CBR_115200      115200

/******************************* 串口基地址 ************************************/
#define USART1_DR_Base  0x40013804
#define USART2_DR_Base  0x40004404
#define USART3_DR_Base  0x40004804
#define USART4_DR_Base	0x40004C04

/**************************** 串口初始化结构 ***********************************/
//串口GPIO相关定义
typedef struct
{
	GPIO_TypeDef		*nRxTxPort;			//接收发送引脚所在端口	
	unsigned int		nRCC_GPIORxTx;		//TX,RX,GPIO时钟
	unsigned short		nGPIO_Rx;			//串口接收引脚
	unsigned short		nGPIO_Tx;			//串口发送引脚	
}UsartGpioPara;

//串口基本通讯参数
typedef struct
{
	unsigned int		nBaudRate;			//串口波特率
	unsigned short		nParity;			//校验位
	unsigned short		nStopBits;			//停止位	
}UsartBasePara;

//串口硬件参数
typedef struct
{
	unsigned int		nUsartDRBase;		//串口基地址
	unsigned int		nRCC_USART;			//串口时钟	
	unsigned short		nUsartIRQ;			//USART中断通道
}UsartHalPara;

//串口DMA配置参数
typedef struct
{
	IRQn_Type			nDMARxIRQ;			//串口DMA接收中断通道
	IRQn_Type			nDMATxIRQ;			//串口DMA发送中断通道
	DMA_Channel_TypeDef	*nDMARxChannel;		//串口DMA接收通道
	DMA_Channel_TypeDef	*nDMATxChannel;		//串口DMA发送通道		
	unsigned int		nUsartDMARxlen;		//串口DMA接收长度			
	unsigned int		nUsartDMATxlen;		//串口DMA发送长度	
	unsigned char		*UsartDMARxBuff;	//串口DMA接收缓冲区
	unsigned char		*UsartDMATxBuff;	//串口DMA发送缓冲区	
}UsartDmaPara;

/******************************** 串口Buff属性 *********************************/
typedef struct
{
	unsigned char *UsartRxBuff;				//串口接收缓冲
	unsigned char *UsartTxBuff;				//串口发送缓冲	
	unsigned short rx_wr_index;				//串口接收写指针
	unsigned short rx_rd_index;				//串口接收读指针
	unsigned short rx_counter;				//串口接收未读数据个数
	unsigned short rxMaxCnt;				//串口缓冲区大小
}UsartBuff;

/******************************** 串口状态属性 *********************************/
typedef struct
{
	unsigned char IsTxEnd;					//DMA发送完成标志
	unsigned char nRxByteTimeout;			//接收字符间超时，TICK数
	unsigned char nRxFirstByteTimeout;		//首字符超时
}UsartSt;

/**************************** 串口初始化结构 ***********************************/
typedef struct
{
	USART_TypeDef		*nCom;				//串口号
	UsartGpioPara		nGpioPort;			//串口GPIO
	UsartBasePara		nUsartBase;			//串口基本通讯参数
	UsartHalPara		nUsartHalPara;		//串口硬件参数
	UsartDmaPara		nUsartDma;			//串口DMA配置
	UsartSt				nUsartSt;			//串口属性
	UsartBuff			nBuff;				//串口接收缓存
}UsartInfo;

/******************************************************************************
**串口全局变量定义
******************************************************************************/
//串口1PC通讯串口
extern UsartInfo nCom1;
//串口2 M72D GPRS模块
extern UsartInfo nCom2;
//串口2 TD3020 北斗模块
extern UsartInfo nCom3;
//串口4 蓝牙模块
extern UsartInfo nCom4;


/******************************************************************************
**串口初始化
*******************************************************************************/
unsigned char ComPort1Init(uint32_t Bps,uint16_t Party);

/******************************* 串口2初始化 *******************************/
unsigned char ComPort2Init(uint32_t Bps,uint16_t Party);

/******************************* 串口3初始化 *******************************/
unsigned char TD3020ComInit(uint32_t Bps);

/******************************* 串口4初始化 *******************************/
unsigned char BlueToothComInit(uint32_t Bps);

/******************************* 私有函数申明 *********************************/
/******************************************************************************
**UsartInitInfo	：串口初始化参数
**mComBuff		：串口接收缓冲区属性
*******************************************************************************/
void UsartInitDMA(UsartInfo *Info);

/******************************************************************************
从串口缓冲区读一个字节
*******************************************************************************/
unsigned char getchar(UsartBuff *Info,unsigned char *data);

/******************************************************************************
**启动DMA接收
*******************************************************************************/
void StartUsartDMARx(UsartInfo *Info);

/******************************************************************************
**启动DMA发送数据
******************************************************************************/
void StartUsartDMATx(UsartInfo *Info,unsigned char *pBuff, unsigned short length);

/******************************************************************************
**接收DMA数据，在串口空闲中断或DMA接收中断时调用
******************************************************************************/
unsigned short UsartDMARx(UsartInfo *Info);

/*******************************************************************************************
*串口发送
*******************************************************************************************/
void ComPortWrite(UsartInfo *info,unsigned char *rxbuf,unsigned short len);

/*******************************************************************************************
*等待串口发送完成
*返回:	0发送未完成
		1发送完成
*******************************************************************************************/
uint8_t ComPortWaitTxEnd(UsartInfo *info);

/*******************************************************************************************
*串口接收
*******************************************************************************************/
unsigned short ComPortRead(UsartInfo *info,unsigned char *rxbuf,unsigned short len,unsigned short timeout);

/*******************************************************************************************
*清除串口缓存
********************************************************************************************/
void ComPortFlush(UsartInfo *info);
	

#endif
