#include "..\uCOS-II\Source\ucos_ii.h"

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
//sbus���ݽṹ
typedef struct
{
	INT8U DataHead;
	INT8U Data[22];
	INT8U DataTag;
	INT8U DataEnd;
}SbusDataTypeDef;

//ADת�����������ݽṹ
typedef struct
{
	INT16U channels[18];
	INT8U  failsafe_status;
}ADCDataTypeDef;


//��AD����ת��SBUS����
SbusDataTypeDef SbusIn(ADCDataTypeDef ADCData);
//��SBUS����ת��AD����
ADCDataTypeDef SbusOut(INT8U *sbusData);

