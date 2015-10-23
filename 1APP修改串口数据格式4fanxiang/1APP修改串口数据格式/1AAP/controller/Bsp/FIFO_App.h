/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __FIFO_APP_H
#define __FIFO_APP_H

/* Includes ------------------------------------------------------------------*/


/* Define   ------------------------------------------------------------------*/
//FIFO������ز���

//�������ݽ�����Ϣ��ʽ
#define RF_RX_Pkg_SIZE	16
#define RF_rx_SIZE	36

/* Data structure   ----------------------------------------------------------*/
typedef void (* Copy_t)(void *A, void *B);
typedef void (* Clear_t)(void *A);

typedef struct {
    void *pBuff;          /*ָ�򻺳�����ָ��*/
    unsigned char Size;           /*�������Ĵ�С*/
    unsigned char Leng;           /*������Ԫ�صĳ���*/
    unsigned char Head;           /*���е�ͷָ��,��ȡ���ݵ�ָ��*/
    unsigned char Tail;           /*���е�βָ��,��Ҫд�����ݵ�ָ��*/
    unsigned char Empty;          /*0:���в��� 1:����Ϊ��*/
}Queue;
                 
//�û��Զ������ݽṹ
//�������ݽ�����Ϣ��ʽ
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
	**Funktion:���еĳ�ʼ������**
**/
void InitQueue(Queue *me, void *pBuff, unsigned char Size, unsigned char Leng);
/**
	**Funktion:�������в�������**
**/
void InsertQueue(Queue *me, void *data, Copy_t pCopy);  
/**
	**Funktion:�Ӷ����л�ȡ����,���п�ʱ����0**
**/
unsigned char GetQueue(Queue *me, void *data, Copy_t pCopy, Clear_t pClear); 
/**
	**Funktion:�ж϶����Ƿ�Ϊ��**
**/ 
unsigned char QueueIsEmpty(Queue *me);   
/**
	**Funktion:�û��Զ���ĸ��ƺ���**
**/
//���߲���
void CopyQueue_RFrxFIFO(void *A, void *B);

/**
	**Funktion:�û��Զ������պ���**
**/
//���߲���
void ClearQueue_RFrxFIFO(void *A);


/**
	**Funktion:FIFOӦ�����ݳ�ʼ��**
**/
//**-----------*QueueBuff----FIFO���ݻ���������-------------------**//
//**------------QueueSIZE----FIFO���������ݳ���-------------------**//
//**------------SingleSize---FIFO���ݵ������ݳ���-----------------**//

#endif
 
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
