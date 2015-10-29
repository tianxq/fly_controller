/* Includes ------------------------------------------------------------------*/
#include "FIFO_App.h" 
#include "string.h"

/*���еĳ�ʼ������*/
//*me:FIFO���У�*pBuff��������ָ�룬Size�����Դ���������ݣ�Leng��ÿ�����ݵĳ���
void InitQueue(Queue *me, void *pBuff, unsigned char Size, unsigned char Leng)
{
    me->pBuff = pBuff;
    me->Size = Size;
    me->Leng = Leng;
    me->Head = 0;
    me->Tail = 0;
    me->Empty = 1;
}

 /*�������в�������*/
void InsertQueue(Queue *me, void *data, Copy_t pCopy) 
{   
    if(me->Empty != 0)/*�������Ϊ��*/
    {
        me->Empty = 0; /*���в���*/
    }
    else
    {
        if(me->Head == me->Tail)/*ͷβָ���ص������Ҷ��в�Ϊ�գ���˵��������*/
        {
            if(++(me->Head) >= me->Size)/*ͷָ��+1*/
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

 /*�Ӷ����л�ȡ����,���п�ʱ����0*/
unsigned char GetQueue(Queue *me, void *data, Copy_t pCopy, Clear_t pClear)    
{
    if(me->Empty != 0)/*�������Ϊ��*/
    {
        return 0;
    }
    pCopy(data, &((unsigned char *)me->pBuff)[me->Head * me->Leng]);
	//��ջ����������
	pClear(&((unsigned char *)me->pBuff)[me->Head * me->Leng]);
    if(++(me->Head) >= me->Size)/*ͷָ��+1*/
    {
        me->Head = 0;
    }
    if(me->Head == me->Tail)/*ȡ��һ������ͷβָ���ص���������Ϊ��*/
    {
        me->Empty = 1; /*����Ϊ��*/
    }
    return 1;
}
/*�ж϶����Ƿ�Ϊ��*/
unsigned char QueueIsEmpty(Queue *me)           
{
    return me->Empty;
}
 /*�û��Զ���ĸ��ƺ���*/
//���߲���
void CopyQueue_RFrxFIFO(void *A, void *B)                     
{
    *(RFrxMsg *)A = *(RFrxMsg *)B;
}
/*�û��Զ������պ���*/
//���߲���
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
	**Funktion:FIFOӦ�����ݳ�ʼ��**
**/
//**-----------*QueueBuff----FIFO���ݻ���������-------------------**//
//**------------QueueSIZE----FIFO���������ݳ���-------------------**//
//**------------SingleSize---FIFO���ݵ������ݳ���-----------------**//

