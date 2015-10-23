/* Includes ------------------------------------------------------------------*/
#include "FIFO_App.h" 
#include "string.h"

/*队列的初始化函数*/
//*me:FIFO队列；*pBuff：缓冲区指针，Size：可以存多少条数据，Leng：每条数据的长度
void InitQueue(Queue *me, void *pBuff, unsigned char Size, unsigned char Leng)
{
    me->pBuff = pBuff;
    me->Size = Size;
    me->Leng = Leng;
    me->Head = 0;
    me->Tail = 0;
    me->Empty = 1;
}

 /*往队列中插入数据*/
void InsertQueue(Queue *me, void *data, Copy_t pCopy) 
{   
    if(me->Empty != 0)/*如果队列为空*/
    {
        me->Empty = 0; /*队列不空*/
    }
    else
    {
        if(me->Head == me->Tail)/*头尾指针重叠，并且队列不为空，即说明队列满*/
        {
            if(++(me->Head) >= me->Size)/*头指针+1*/
            {
                me->Head = 0;
            }
        }
    }
    pCopy(&((unsigned char *)me->pBuff)[me->Tail * me->Leng], data);
		
    if(++(me->Tail) >= me->Size)
    {
        me->Tail = 0;
    }
}

 /*从队列中获取数据,队列空时返回0*/
unsigned char GetQueue(Queue *me, void *data, Copy_t pCopy, Clear_t pClear)    
{
    if(me->Empty != 0)/*如果队列为空*/
    {
        return 0;
    }
    pCopy(data, &((unsigned char *)me->pBuff)[me->Head * me->Leng]);
	//出栈后将数据清零
	pClear(&((unsigned char *)me->pBuff)[me->Head * me->Leng]);
    if(++(me->Head) >= me->Size)/*头指针+1*/
    {
        me->Head = 0;
    }
    if(me->Head == me->Tail)/*取出一个数后，头尾指针重叠，即队列为空*/
    {
        me->Empty = 1; /*队列为空*/
    }
    return 1;
}
/*判断队列是否为空*/
unsigned char QueueIsEmpty(Queue *me)           
{
    return me->Empty;
}
 /*用户自定义的复制函数*/
//无线部分
void CopyQueue_RFrxFIFO(void *A, void *B)                     
{
    *(RFrxMsg *)A = *(RFrxMsg *)B;
}
/*用户自定义的清空函数*/
//无线部分
void ClearQueue_RFrxFIFO(void *A)                     
{
    RFrxMsg ClearRFrxFIFO;

    ClearRFrxFIFO.rssi = 0;
    ClearRFrxFIFO.rx_id=0;
    ClearRFrxFIFO.rx_len=0;
    ClearRFrxFIFO.rx_mac[0]=0;
    ClearRFrxFIFO.rx_mac[1]=0;
    ClearRFrxFIFO.rx_mac[2]=0;
    ClearRFrxFIFO.rx_mac[3]=0;

    memset(ClearRFrxFIFO.rx_data,0,RF_rx_SIZE);
    *(RFrxMsg *)A = ClearRFrxFIFO;
}


/**
	**Funktion:FIFO应用数据初始化**
**/
//**-----------*QueueBuff----FIFO数据缓冲区数据-------------------**//
//**------------QueueSIZE----FIFO缓冲区数据长度-------------------**//
//**------------SingleSize---FIFO数据单条数据长度-----------------**//

