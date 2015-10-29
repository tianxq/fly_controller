#include "..\uCOS-II\Source\ucos_ii.h"

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
//sbus数据结构
typedef struct
{
	INT8U DataHead;
	INT8U Data[22];
	INT8U DataTag;
	INT8U DataEnd;
}SbusDataTypeDef;

//AD转换过来的数据结构
typedef struct
{
	INT16U channels[18];
	INT8U  failsafe_status;
}ADCDataTypeDef;


//将AD数据转成SBUS数据
SbusDataTypeDef SbusIn(ADCDataTypeDef ADCData);
//讲SBUS数据转成AD数据
ADCDataTypeDef SbusOut(INT8U *sbusData);

