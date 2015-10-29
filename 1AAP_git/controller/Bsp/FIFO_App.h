/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __FIFO_APP_H
#define __FIFO_APP_H

/* Includes ------------------------------------------------------------------*/


/* Define   ------------------------------------------------------------------*/
//FIFO数据相关参数

//无线数据接收消息格式
#define RF_RX_Pkg_SIZE	16
#define RF_rx_SIZE	36

/* Data structure   ----------------------------------------------------------*/
typedef void (* Copy_t)(void *A, void *B);
typedef void (* Clear_t)(void *A);

typedef struct {
    void *pBuff;          /*指向缓冲区的指针*/
    unsigned char Size;           /*缓冲区的大小*/
    unsigned char Leng;           /*缓冲区元素的长度*/
    unsigned char Head;           /*队列的头指针,读取数据的指针*/
    unsigned char Tail;           /*队列的尾指针,将要写入数据的指针*/
    unsigned char Empty;          /*0:队列不空 1:队列为空*/
}Queue;
                 
//用户自定义数据结构
//无线数据接收消息格式
typedef struct
{
	unsigned char rssi;
	unsigned char rx_mac[4];
	unsigned char rx_id;
	unsigned char rx_len;
	unsigned char rx_data[RF_rx_SIZE];
}RFrxMsg;


/* Function declaration   ----------------------------------------------------*/

/**
	**Funktion:队列的初始化函数**
**/
void InitQueue(Queue *me, void *pBuff, unsigned char Size, unsigned char Leng);
/**
	**Funktion:往队列中插入数据**
**/
void InsertQueue(Queue *me, void *data, Copy_t pCopy);  
/**
	**Funktion:从队列中获取数据,队列空时返回0**
**/
unsigned char GetQueue(Queue *me, void *data, Copy_t pCopy, Clear_t pClear); 
/**
	**Funktion:判断队列是否为空**
**/ 
unsigned char QueueIsEmpty(Queue *me);   
/**
	**Funktion:用户自定义的复制函数**
**/
//无线部分
void CopyQueue_RFrxFIFO(void *A, void *B);

/**
	**Funktion:用户自定义的清空函数**
**/
//无线部分
void ClearQueue_RFrxFIFO(void *A);


/**
	**Funktion:FIFO应用数据初始化**
**/
//**-----------*QueueBuff----FIFO数据缓冲区数据-------------------**//
//**------------QueueSIZE----FIFO缓冲区数据长度-------------------**//
//**------------SingleSize---FIFO数据单条数据长度-----------------**//

#endif
 
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
