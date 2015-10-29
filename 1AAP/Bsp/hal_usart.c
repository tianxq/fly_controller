/***************************************************************************************
 *  File:    hal_usart.c
 *
 *  Author:  Andy.Zhang
 *
 *  Data:    2015-8-3
 *
 *  Version: v1.0
 *
 *  Describe:
 *
 * ************************************************************************************
 *   All rights reserved by the author.
 **************************************************************************************/
#include <string.h>

#include "misc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_usart.h"
#include "ucos_ii.h"
#include "hal_usart.h"


/**************************************************************************************
 *   							Variable
 **************************************************************************************/
/* usart1 dma rx buffer*/
unsigned char Usart1DMARxBuff[USART1_DMA_RX_MAX_LEN];
/* usart1 dma tx buffer*/
static unsigned char Usart1DMATxBuff[USART1_DMA_TX_MAX_LEN];
/* usart1 fifo status struct */
uart_fifo_t USART1_RxFIFO;

/* usart3 dma rx buffer*/
static unsigned char Usart3DMARxBuff[USART3_DMA_RX_MAX_LEN];
/* usart3 dma tx buffer*/
static unsigned char Usart3DMATxBuff[USART3_DMA_TX_MAX_LEN];
/**************************************************************************************
 *   							Function
 **************************************************************************************/

/**
 * Initialize the usart1 fifo
 */
void usart1_fifo_init(void)
{
	USART1_RxFIFO.head = USART1_RxFIFO.tail = 0;

	USART1_RxFIFO.pBuffer = Usart1DMARxBuff;
}


static uint16_t get_usart1_rx_cnt(void)
{
    return (uint16_t)(USART1_DMA_RX_MAX_LEN - DMA_GetCurrDataCounter(DMA1_Channel5));
}

/**
 * Read data from the usart1 rx buffer,the rx buffer is a software fifo.
 * @param  pRxBuffer [The destination buffer]
 * @param  max_len   [the max length to read from the usart1 fifo]
 * @return           [the actrully size of the data read from the fifo finally]
 */
uint16_t read_usart1_fifo(uint8_t * restrict pRxBuffer, uint16_t max_len)
{
	uint16_t i,rd_cnt;

	USART1_RxFIFO.tail = get_usart1_rx_cnt();

	for(i=0,rd_cnt=0; i<USART1_RxFIFO.tail; i++,rd_cnt++){
		pRxBuffer[i] = USART1_RxFIFO.pBuffer[USART1_RxFIFO.head++];
	}

	return rd_cnt;
}


/**
 * Read string from usart1 rx fifo.
 * @param  str         [the destination buffer to store the string]
 * @param  max_str_len [the max length of the string to read, make sure the length of the string
 *                     	won't beyond the threshold]
 * @return             [the absolute length of the string finally read out from the fifo]
 */
uint16_t usart1_read_string(char * str,uint16_t max_str_len)
{
	char * restrict p,* restrict dest;
  uint16_t rd_cnt=0;

  p = (char *)(&USART1_RxFIFO.pBuffer[USART1_RxFIFO.head]);
	dest = str;


	USART1_RxFIFO.tail = get_usart1_rx_cnt();

	while( (*p != '\0') && (USART1_RxFIFO.head < USART1_RxFIFO.tail) && (rd_cnt < max_str_len) ){
		*dest++ = *p++;
		USART1_RxFIFO.head++; //increase the fifo head.
    rd_cnt++;
	}

	*dest = '\0';

	return rd_cnt;
}


/**
 * read a frame from the usart1 fifo by checking the frame head and tail.
 * @param  str         [point to a buffer which to store the data read out from the fifo]
 * @param  start_mark  [the head of the frame]
 * @param  end_mark    [the end char of the frame]
 * @param  max_str_len [the max length of the string, the size of the string read out would not beyond this value]
 * @param  timeout_ms  [timeout in ms unit]
 * @return             [status]
 */
uint8_t serialport1_read_frame(char * str, char start_mark, char end_mark, uint16_t max_str_len, uint16_t timeout_ms)
{
    char * restrict dest = str;
    uint16_t tim=timeout_ms/5, rd_cnt=0;

    /* first search the start of the frame */
    do{
        USART1_RxFIFO.tail = get_usart1_rx_cnt();
        /* if the rx fifo is empty */
        if(USART1_RxFIFO.head == USART1_RxFIFO.tail){
            OS_DELAY_MS(5);
            tim--;
        }
        else if( USART1_RxFIFO.pBuffer[USART1_RxFIFO.head] == start_mark )
        	break;
        else
            USART1_RxFIFO.head++;
    }while(tim);

    if(tim == 0)
        return UART_READ_TIMEOUT;
    /* second read the whole frame */
    do{
        USART1_RxFIFO.tail = get_usart1_rx_cnt();

        if(USART1_RxFIFO.head == USART1_RxFIFO.tail){
            OS_DELAY_MS(5);
            tim--;
        }
        else{
             *dest++ = USART1_RxFIFO.pBuffer[USART1_RxFIFO.head];

             rd_cnt++;
        }
    }while( (USART1_RxFIFO.pBuffer[USART1_RxFIFO.head++] != end_mark) && (rd_cnt <= max_str_len) && (tim));

    *dest = '\0';

    if( tim == 0 )
        return UART_READ_TIMEOUT;

    return UART_READ_SUCCESS;
}


void hal_usart1_base_init(uint32_t baudrate, uint16_t parity, uint16_t stop_bit)
{
	USART_InitTypeDef USART_InitStructure;	//串口初始化结构
	GPIO_InitTypeDef GPIO_InitStructure;	//GPIO初始化结构


	assert_param(IS_USART_PARITY(parity));
	assert_param(IS_USART_STOPBITS(stop_bit));

		//GPIO时钟使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  	//串口时钟使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

		//串口发送引脚配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //串口接收引脚初始化
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //串口恢复到默认值
    USART_DeInit(USART1);
    //串口通讯参数配置
    USART_InitStructure.USART_BaudRate = baudrate;

    USART_InitStructure.USART_StopBits = stop_bit;  //Stop Bit
    USART_InitStructure.USART_Parity = parity;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_WordLength = (parity == USART_Parity_No)? USART_WordLength_8b:USART_WordLength_9b;  //Word Length = 8 Bits

    USART_Init(USART1, &USART_InitStructure);

	//使能串口
    USART_Cmd(USART1, ENABLE);
}



void hal_usart1_dma_config(uint8_t * const rxbuffer, uint32_t rxbuffer_size, uint8_t * const txbuffer, uint32_t txbuffer_size)
{
	DMA_InitTypeDef DMA_InitStructure;		//DMA初始化结构

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
		//串口DMA发送通道使能
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    //串口DMA接收通道使能
    USART_DMACmd(USART1, USART_DMAReq_Rx , ENABLE);

		//串口DMA发送配置
    DMA_DeInit(DMA1_Channel4);	//串口DMA发送通道
	
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);//USART1_DR_Base;	//串口基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(txbuffer);//串口发送缓存区
    DMA_InitStructure.DMA_BufferSize = txbuffer_size;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);
  	//不使能DMA发送完成中断
    //DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
		
	DMA_ClearITPendingBit(DMA1_FLAG_TE4);
		
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TE, ENABLE);

	//串口DMA接收配置
    DMA_DeInit(DMA1_Channel5);	//串口DMA接收通道);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);//USART1_DR_Base;	//串口基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(rxbuffer);//串口接收缓存区
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = rxbuffer_size;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

		//不使能DMA接收完成中断
    //DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
    //启动DAM接收
    DMA_Cmd(DMA1_Channel5, ENABLE);
}



bool hal_usart1_strobe_write(uint32_t size)
{
	//DMA_GetCurrDataCounter(DMA1_Channel4);
	
	if ( size > 0)
	{
		DMA_Cmd(DMA1_Channel4, DISABLE);
		
		//设置发送长度
		DMA_SetCurrDataCounter(DMA1_Channel4, size);
		//启动发送
		DMA_Cmd(DMA1_Channel4, ENABLE);
		
		return true;
	}
	
	return false;
}

/**
 * Initialize the stm32 usart1, using the dma channel 14 for transmittion data from
 * mUsart1DMATxBuff to usart1 port and dma channel 15 for receiving data from usart1
 * port to mUsart1DMARxBuff.Do not generate any interrupts,the dma transmit the constant
 * size data repeatedly between the ram buffer and peripheral port.
 *
 * @param baudrate [the baudrate of the usart1]
 */
void hal_usart1_init(uint32_t baudrate)
{
		USART_InitTypeDef USART_InitStructure;	//串口初始化结构
    DMA_InitTypeDef DMA_InitStructure;		//DMA初始化结构
    GPIO_InitTypeDef GPIO_InitStructure;	//GPIO初始化结构
    //NVIC_InitTypeDef NVIC_InitStructure;	//中断控制初始化结构


		assert_param(IS_USART_PARITY(parity));

	//DMA时钟使能
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    //GPIO时钟使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  	//串口时钟使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);


    //串口发送引脚配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //串口接收引脚初始化
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //串口恢复到默认值
    USART_DeInit(USART1);
    //串口通讯参数配置
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //Word Length = 8 Bits
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  //Stop Bit
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    //串口DMA发送通道使能
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    //串口DMA接收通道使能
    USART_DMACmd(USART1, USART_DMAReq_Rx , ENABLE);
	//使能串口
    USART_Cmd(USART1, ENABLE);

    //串口DMA发送配置
    DMA_DeInit(DMA1_Channel4);	//串口DMA发送通道
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);//USART1_DR_Base;	//串口基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(Usart1DMATxBuff);//串口发送缓存区
    DMA_InitStructure.DMA_BufferSize = USART1_DMA_TX_MAX_LEN;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);
  	//不使能DMA发送完成中断
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, DISABLE);

	//串口DMA接收配置
    DMA_DeInit(DMA1_Channel5);	//串口DMA接收通道);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);//USART1_DR_Base;	//串口基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(Usart1DMARxBuff);//串口接收缓存区
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = USART1_DMA_RX_MAX_LEN;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//循环接收
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

	//不使能DMA接收完成中断
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, DISABLE);
    //启动DAM接收
    DMA_Cmd(DMA1_Channel5, ENABLE);
}




void StartUsart1DMATx(uint8_t * pdata, uint16_t size)
{
  if(size > 0)
	{
		//关闭DMA
		DMA_Cmd(DMA1_Channel4, DISABLE);

		//待发送数据写入发送缓冲区
		memcpy(Usart1DMARxBuff, pdata, size);

		//设置发送长度
		DMA_SetCurrDataCounter(DMA1_Channel4, size);

		//启动发送
		DMA_Cmd(DMA1_Channel4, ENABLE);
	}
}




/**
 * write data on usart1 bus.
 * @param  pdata      [a pointer point to the data to transmit on the usart1 bus]
 * @param  size       [the data size]
 * @param  timeout_ms [time out in milliseconds unit]
 * @return            [return the status]
 */
uint8_t hal_usart1_write(uint8_t * restrict pdata, uint16_t size, uint16_t timeout_ms)
{

	uint16_t tim = timeout_ms;

	if(size == 0)
		return UART_DATA_SIZE_ERR;

	//if USART is busy on transmitting, wait for a while
	while((USART_GetFlagStatus( USART1, USART_FLAG_TXE) == RESET) && (--tim) && (timeout_ms))
		OS_DELAY_MS(1);

	if(USART_GetFlagStatus( USART1, USART_FLAG_TXE) == SET){
		StartUsart1DMATx(pdata, size);

		return UART_TX_SUCCESS;
	}
	else
		return UART_BUSY;
}

void hal_usart3_init(uint32_t baudrate)
{
		USART_InitTypeDef USART_InitStructure;	//串口初始化结构
    DMA_InitTypeDef DMA_InitStructure;		//DMA初始化结构
    GPIO_InitTypeDef GPIO_InitStructure;	//GPIO初始化结构

    //usart3_fifo_init();

		//DMA时钟使能
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    //GPIO时钟使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  	//串口时钟使能
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);


    //串口发送引脚配置,PB10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //串口接收引脚初始化,PB11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //串口恢复到默认值
    USART_DeInit(USART3);
    //串口通讯参数配置
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //Word Length = 8 Bits
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  //Stop Bit
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
    //串口DMA发送通道使能
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
    //串口DMA接收通道使能
    USART_DMACmd(USART3, USART_DMAReq_Rx , ENABLE);
	//使能串口
    USART_Cmd(USART3, ENABLE);

    //串口DMA发送配置
    DMA_DeInit(DMA1_Channel2);	//串口DMA发送通道
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);//USART1_DR_Base;	//串口基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(Usart3DMATxBuff);//串口发送缓存区
    DMA_InitStructure.DMA_BufferSize = USART3_DMA_TX_MAX_LEN;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel2, &DMA_InitStructure);
  	//不使能DMA发送完成中断
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, DISABLE);

	//串口DMA接收配置
    DMA_DeInit(DMA1_Channel3);	//串口DMA接收通道);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);//USART1_DR_Base;	//串口基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(Usart3DMARxBuff);//串口接收缓存区
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = USART3_DMA_RX_MAX_LEN;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//循环接收
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);

	//不使能DMA接收完成中断
    DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, DISABLE);
    //启动DAM接收
    DMA_Cmd(DMA1_Channel3, ENABLE);
}


/**
 * write data on usart1 bus.
 * @param  pdata      [a pointer point to the data to transmit on the usart1 bus]
 * @param  size       [the data size]
 * @param  timeout_ms [time out in milliseconds unit]
 * @return            [return the status]
 */
uint8_t hal_usart3_write(uint8_t * restrict pdata, uint16_t size, uint16_t timeout_ms)
{

	uint16_t tim = timeout_ms/5;

	if(size == 0)
		return UART_DATA_SIZE_ERR;

	//if USART is busy on transmitting, wait for a while
	while((USART_GetFlagStatus( USART3, USART_FLAG_TXE) == RESET) && (--tim) && (timeout_ms))
		OS_DELAY_MS(5);

	if(USART_GetFlagStatus( USART3, USART_FLAG_TXE) == SET){
        //关闭DMA
	    DMA_Cmd(DMA1_Channel2, DISABLE);

	    //待发送数据写入发送缓冲区
		memcpy(Usart3DMATxBuff, pdata, size);

		//设置发送长度
	    DMA1_Channel2->CNDTR = size;

	    //启动发送
	    DMA_Cmd(DMA1_Channel2, ENABLE);

		return UART_TX_SUCCESS;
	}
	else
		return UART_BUSY;
}
