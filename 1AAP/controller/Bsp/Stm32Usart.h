/******************** (C) COPYRIGHT 2008 boost ********************
* File Name          : usart.h
* Author             : csz
* Version            : V1.0.0
* Date               : 2014��5��4��
* Description        : Header for usart.c module
********************************************************************************/

#ifndef __USART_H
#define __USART_H

#include "includes.h"
/*************************** ����ͨ�Ų����ʲ��� ********************************/
#define CBR_2400        2400
#define CBR_4800        4800
#define CBR_9600        9600
#define CBR_14400       14400
#define CBR_19200       19200
#define CBR_38400		38400
#define	CBR_57600		57600
#define CBR_115200      115200

/******************************* ���ڻ���ַ ************************************/
#define USART1_DR_Base  0x40013804
#define USART2_DR_Base  0x40004404
#define USART3_DR_Base  0x40004804
#define USART4_DR_Base	0x40004C04

/**************************** ���ڳ�ʼ���ṹ ***********************************/
//����GPIO��ض���
typedef struct
{
	GPIO_TypeDef		*nRxTxPort;			//���շ����������ڶ˿�	
	unsigned int		nRCC_GPIORxTx;		//TX,RX,GPIOʱ��
	unsigned short		nGPIO_Rx;			//���ڽ�������
	unsigned short		nGPIO_Tx;			//���ڷ�������	
}UsartGpioPara;

//���ڻ���ͨѶ����
typedef struct
{
	unsigned int		nBaudRate;			//���ڲ�����
	unsigned short		nParity;			//У��λ
	unsigned short		nStopBits;			//ֹͣλ	
}UsartBasePara;

//����Ӳ������
typedef struct
{
	unsigned int		nUsartDRBase;		//���ڻ���ַ
	unsigned int		nRCC_USART;			//����ʱ��	
	unsigned short		nUsartIRQ;			//USART�ж�ͨ��
}UsartHalPara;

//����DMA���ò���
typedef struct
{
	IRQn_Type			nDMARxIRQ;			//����DMA�����ж�ͨ��
	IRQn_Type			nDMATxIRQ;			//����DMA�����ж�ͨ��
	DMA_Channel_TypeDef	*nDMARxChannel;		//����DMA����ͨ��
	DMA_Channel_TypeDef	*nDMATxChannel;		//����DMA����ͨ��		
	unsigned int		nUsartDMARxlen;		//����DMA���ճ���			
	unsigned int		nUsartDMATxlen;		//����DMA���ͳ���	
	unsigned char		*UsartDMARxBuff;	//����DMA���ջ�����
	unsigned char		*UsartDMATxBuff;	//����DMA���ͻ�����	
}UsartDmaPara;

/******************************** ����Buff���� *********************************/
typedef struct
{
	unsigned char *UsartRxBuff;				//���ڽ��ջ���
	unsigned char *UsartTxBuff;				//���ڷ��ͻ���	
	unsigned short rx_wr_index;				//���ڽ���дָ��
	unsigned short rx_rd_index;				//���ڽ��ն�ָ��
	unsigned short rx_counter;				//���ڽ���δ�����ݸ���
	unsigned short rxMaxCnt;				//���ڻ�������С
}UsartBuff;

/******************************** ����״̬���� *********************************/
typedef struct
{
	unsigned char IsTxEnd;					//DMA������ɱ�־
	unsigned char nRxByteTimeout;			//�����ַ��䳬ʱ��TICK��
	unsigned char nRxFirstByteTimeout;		//���ַ���ʱ
}UsartSt;

/**************************** ���ڳ�ʼ���ṹ ***********************************/
typedef struct
{
	USART_TypeDef		*nCom;				//���ں�
	UsartGpioPara		nGpioPort;			//����GPIO
	UsartBasePara		nUsartBase;			//���ڻ���ͨѶ����
	UsartHalPara		nUsartHalPara;		//����Ӳ������
	UsartDmaPara		nUsartDma;			//����DMA����
	UsartSt				nUsartSt;			//��������
	UsartBuff			nBuff;				//���ڽ��ջ���
}UsartInfo;

/******************************************************************************
**����ȫ�ֱ�������
******************************************************************************/
//����1PCͨѶ����
extern UsartInfo nCom1;
//����2 M72D GPRSģ��
extern UsartInfo nCom2;
//����2 TD3020 ����ģ��
extern UsartInfo nCom3;
//����4 ����ģ��
extern UsartInfo nCom4;


/******************************************************************************
**���ڳ�ʼ��
*******************************************************************************/
unsigned char ComPort1Init(uint32_t Bps,uint16_t Party);

/******************************* ����2��ʼ�� *******************************/
unsigned char ComPort2Init(uint32_t Bps,uint16_t Party);

/******************************* ����3��ʼ�� *******************************/
unsigned char TD3020ComInit(uint32_t Bps);

/******************************* ����4��ʼ�� *******************************/
unsigned char BlueToothComInit(uint32_t Bps);

/******************************* ˽�к������� *********************************/
/******************************************************************************
**UsartInitInfo	�����ڳ�ʼ������
**mComBuff		�����ڽ��ջ���������
*******************************************************************************/
void UsartInitDMA(UsartInfo *Info);

/******************************************************************************
�Ӵ��ڻ�������һ���ֽ�
*******************************************************************************/
unsigned char getchar(UsartBuff *Info,unsigned char *data);

/******************************************************************************
**����DMA����
*******************************************************************************/
void StartUsartDMARx(UsartInfo *Info);

/******************************************************************************
**����DMA��������
******************************************************************************/
void StartUsartDMATx(UsartInfo *Info,unsigned char *pBuff, unsigned short length);

/******************************************************************************
**����DMA���ݣ��ڴ��ڿ����жϻ�DMA�����ж�ʱ����
******************************************************************************/
unsigned short UsartDMARx(UsartInfo *Info);

/*******************************************************************************************
*���ڷ���
*******************************************************************************************/
void ComPortWrite(UsartInfo *info,unsigned char *rxbuf,unsigned short len);

/*******************************************************************************************
*�ȴ����ڷ������
*����:	0����δ���
		1�������
*******************************************************************************************/
uint8_t ComPortWaitTxEnd(UsartInfo *info);

/*******************************************************************************************
*���ڽ���
*******************************************************************************************/
unsigned short ComPortRead(UsartInfo *info,unsigned char *rxbuf,unsigned short len,unsigned short timeout);

/*******************************************************************************************
*������ڻ���
********************************************************************************************/
void ComPortFlush(UsartInfo *info);
	

#endif
