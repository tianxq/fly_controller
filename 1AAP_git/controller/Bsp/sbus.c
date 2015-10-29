#include "includes.h"

//将AD数据转成SBUS数据
SbusDataTypeDef SbusIn(ADCDataTypeDef ADCData)
{
	SbusDataTypeDef sbusData;
	
	sbusData.DataHead=0xf0;//HEAD
	sbusData.DataEnd=0;//END
	
	sbusData.Data[0]=ADCData.channels[0]>>3  & 0xFF;
	sbusData.Data[1]=(ADCData.channels[0]<<5 | ADCData.channels[1]>>6)  & 0xFF;
	sbusData.Data[2]=(ADCData.channels[1]<<2 | ADCData.channels[2]>>9)  & 0xFF;
	sbusData.Data[3]=(ADCData.channels[2]<<1 )  & 0xFF;
	sbusData.Data[4]=(ADCData.channels[2]<<7 | ADCData.channels[3]>>4)  & 0xFF;
	sbusData.Data[5]=(ADCData.channels[3]<<4 | ADCData.channels[4]>>7)  & 0xFF;
	sbusData.Data[6]=(ADCData.channels[4]<<1 | ADCData.channels[5]>>10)  & 0xFF;
	sbusData.Data[7]=(ADCData.channels[5]>>2 )  & 0xFF;
	sbusData.Data[8]=(ADCData.channels[5]<<6 | ADCData.channels[6]>>5)  & 0xFF;
	sbusData.Data[9]=(ADCData.channels[6]<<3 | ADCData.channels[7]>>8)  & 0xFF;
	sbusData.Data[10]=(ADCData.channels[7])  & 0xFF;
	
	sbusData.Data[11]=ADCData.channels[8]>>3  & 0xFF;
	sbusData.Data[12]=(ADCData.channels[8]<<5 | ADCData.channels[9]>>6)  & 0xFF;
	sbusData.Data[13]=(ADCData.channels[9]<<2 | ADCData.channels[10]>>9)  & 0xFF;
	sbusData.Data[14]=(ADCData.channels[10]<<1 )  & 0xFF;
	sbusData.Data[15]=(ADCData.channels[10]<<7 | ADCData.channels[11]>>4)  & 0xFF;
	sbusData.Data[16]=(ADCData.channels[11]<<4 | ADCData.channels[12]>>7)  & 0xFF;
	sbusData.Data[17]=(ADCData.channels[12]<<1 | ADCData.channels[13]>>10)  & 0xFF;
	sbusData.Data[18]=(ADCData.channels[13]>>2 )  & 0xFF;
	sbusData.Data[19]=(ADCData.channels[13]<<6 | ADCData.channels[14]>>5)  & 0xFF;
	sbusData.Data[20]=(ADCData.channels[14]<<3 | ADCData.channels[15]>>8)  & 0xFF;
	sbusData.Data[21]=(ADCData.channels[15])  & 0xFF;
	
	sbusData.DataTag=(ADCData.channels[16] |(ADCData.channels[17]<<1)|(ADCData.failsafe_status<<2)) & 0xFF;
	
	return sbusData;
}

//讲SBUS数据转成AD数据
ADCDataTypeDef SbusOut(INT8U *sbusData)
{
	ADCDataTypeDef ADCData;
	
	ADCData.channels[0]  = ((sbusData[1]|sbusData[2]<< 8) & 0x07FF);
  ADCData.channels[1]  = ((sbusData[2]>>3|sbusData[3]<<5) & 0x07FF);
  ADCData.channels[2]  = ((sbusData[3]>>6|sbusData[4]<<2|sbusData[5]<<10) & 0x07FF);
  ADCData.channels[3]  = ((sbusData[5]>>1|sbusData[6]<<7) & 0x07FF);
  ADCData.channels[4]  = ((sbusData[6]>>4|sbusData[7]<<4) & 0x07FF);
  ADCData.channels[5]  = ((sbusData[7]>>7|sbusData[8]<<1|sbusData[9]<<9) & 0x07FF);
  ADCData.channels[6]  = ((sbusData[9]>>2|sbusData[10]<<6) & 0x07FF);
  ADCData.channels[7]  = ((sbusData[10]>>5|sbusData[11]<<3) & 0x07FF); // & the other 8 + 2 channels if you need them
  ADCData.channels[8]  = ((sbusData[12]|sbusData[13]<< 8) & 0x07FF);
  ADCData.channels[9]  = ((sbusData[13]>>3|sbusData[14]<<5) & 0x07FF);
  ADCData.channels[10] = ((sbusData[14]>>6|sbusData[15]<<2|sbusData[16]<<10) & 0x07FF);
  ADCData.channels[11] = ((sbusData[16]>>1|sbusData[17]<<7) & 0x07FF);
  ADCData.channels[12] = ((sbusData[17]>>4|sbusData[18]<<4) & 0x07FF);
  ADCData.channels[13] = ((sbusData[18]>>7|sbusData[19]<<1|sbusData[20]<<9) & 0x07FF);
  ADCData.channels[14] = ((sbusData[20]>>2|sbusData[21]<<6) & 0x07FF);
  ADCData.channels[15] = ((sbusData[21]>>5|sbusData[22]<<3) & 0x07FF);

  // DigiChannel 1
  if (sbusData[23] & (1<<0)) {
    ADCData.channels[16] = 1;
  }
  else{
    ADCData.channels[16] = 0;
  }
  // DigiChannel 2
  if (sbusData[23] & (1<<1)) {
    ADCData.channels[17] = 1;
  }
  else{
    ADCData.channels[17] = 0;
  }
  // Failsafe
  ADCData.failsafe_status = SBUS_SIGNAL_OK;
  if (sbusData[23] & (1<<2)) {
    ADCData.failsafe_status = SBUS_SIGNAL_LOST;
  }
  if (sbusData[23] & (1<<3)) {
    ADCData.failsafe_status = SBUS_SIGNAL_FAILSAFE;
  }
	
	return ADCData;
}
