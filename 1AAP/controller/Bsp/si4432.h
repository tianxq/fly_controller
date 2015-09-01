/****************************************Copyright (c)****************************************************
**                               Guangzhou ZHIYUAN electronics Co.,LTD.
**
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:               Si43.h
** Last modified Date:      2013-11-22
** Last Version:            v1.0
** Description:             
**
**--------------------------------------------------------------------------------------------------------
** Created By:              yanghongyu
** Created date:            2013-6-10
** Version:                 v1.0
** Descriptions:             
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Description:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
#ifndef __SI4432_SET_h
#define __SI4432_SET_h


#include <stdbool.h>

#define  TX_NUMBER     20   


#define rf_recv_it
/************************************************************************************
************************  Si4432Ӳ��SPI�ӿ�IO����  **********************************
************************************************************************************/
#define CC11xx_SPI                       SPI2
#define CC11xx_SPI_CLK                   CLK_Peripheral_SPI1
#define CC11xx_SPI_SCK_PIN               GPIO_Pin_13                  /* PI.01 */
#define CC11xx_SPI_SCK_GPIO_PORT         GPIOB                       /* GPIOI */
#define CC11xx_SPI_MISO_PIN              GPIO_Pin_14                  /* PI.03 */
#define CC11xx_SPI_MISO_GPIO_PORT        GPIOB                       /* GPIOI */
#define CC11xx_SPI_MOSI_PIN              GPIO_Pin_15                  /* PI.02 */
#define CC11xx_SPI_MOSI_GPIO_PORT        GPIOB                       /* GPIOI */
#define CC11xx_CS_PIN                    GPIO_Pin_12                  /* PH.07 */
#define CC11xx_CS_GPIO_PORT              GPIOB                       /* GPIOH */
#define CC11xx_SPI_GPIO_PORT         GPIOB                       /* GPIOI */


#define Si4432_IRQ_Port                  GPIOB
#define Si4432_IRQ_Pin                   GPIO_Pin_0

#define Si4432_SDN_Port                  GPIOB
#define Si4432_SDN_Pin                   GPIO_Pin_1

#define SEL_L()        GPIO_ResetBits( CC11xx_CS_GPIO_PORT, CC11xx_CS_PIN)
#define SEL_H()        GPIO_SetBits( CC11xx_CS_GPIO_PORT, CC11xx_CS_PIN)

#define SDN_L()       GPIO_ResetBits( Si4432_SDN_Port, Si4432_SDN_Pin)
#define SDN_H()       GPIO_SetBits( Si4432_SDN_Port, Si4432_SDN_Pin) 
/* ��ȡģ���ж��������״̬     */
#define IRQ_READ()    (GPIO_ReadInputDataBit( Si4432_IRQ_Port, Si4432_IRQ_Pin) != 0) 

#define CLI()      __set_PRIMASK(1)  //���ж�
#define SEI()      __set_PRIMASK(0)  //���ж�

//Device status
#define SI44xx_RX_TX_FIFO_OVERFLOW			  0x80
#define SI44xx_RX_TX_FIFO_UNDERFLOW			  0x40
#define SI44xx_RX_FIFO_EMPTY				  0x20
#define SI44XX_HEADER_ERROR				  0x10
#define SI44xx_FREQUENCY_ERROR				  0x08
#define SI44xx_TX_STATE					  0x02
#define SI44xx_RX_STATE					  0x01
#define SI44xx_IDLE_STATE				  0x00


typedef enum _RF_REG_MAP					//revV2&B1
{
  DeviceType 						= 0x00,
  DeviceVersion						= 0x01,
  /* ��ǰ����״̬                        */ 
  DeviceStatus 						= 0x02,
  /* �ж�״̬��־�Ĵ���                  */ 
  InterruptStatus1 					= 0x03,
  InterruptStatus2 					= 0x04,
  /* �ж�ʹ�ܼĴ�������                  */ 
  InterruptEnable1 					= 0x05,          
  InterruptEnable2 					= 0x06,   
  /* ״̬���ƼĴ���                      */       
  OperatingFunctionControl1 				= 0x07,
  OperatingFunctionControl2 				= 0x08, 
  /* ���ص������üĴ���                  */ 
  CrystalOscillatorLoadCapacitance 			= 0x09,

  MicrocontrollerOutputClock 				= 0x0A,
  GPIO0Configuration 					= 0x0B,
  GPIO1Configuration 					= 0x0C,         
  GPIO2Configuration					= 0x0D,
  IOPortConfiguration					= 0x0E,
  ADCConfiguration					= 0x0F,
  ADCSensorAmplifierOffset				= 0x10,
  ADCValue						= 0x11,
  TemperatureSensorControl				= 0x12,
  TemperatureValueOffset				= 0x13,
  WakeUpTimerPeriod1 					= 0x14,          
  WakeUpTimerPeriod2 					= 0x15,         
  WakeUpTimerPeriod3 					= 0x16,         
  WakeUpTimerValue1					= 0x17,
  WakeUpTimerValue2					= 0x18,
  LowDutyCycleModeDuration 				= 0x19,       
  LowBatteryDetectorThreshold  				= 0x1A,
  BatteryVoltageLevel 					= 0x1B,                          
  IFFilterBandwidth  					= 0x1C,                           
  AFCLoopGearshiftOverride				= 0x1D,
  AFCTimingControl 					= 0x1E,                              
  ClockRecoveryGearshiftOverride 			= 0x1F,              
  ClockRecoveryOversamplingRatio 			= 0x20,              
  ClockRecoveryOffset2 					= 0x21,                       
  ClockRecoveryOffset1 					= 0x22,                       
  ClockRecoveryOffset0 					= 0x23,                     
  ClockRecoveryTimingLoopGain1 				= 0x24,              
  ClockRecoveryTimingLoopGain0 				= 0x25,             
  ReceivedSignalStrengthIndicator 			= 0x26,          
  RSSIThresholdForClearChannelIndicator 	        = 0x27,   
  AntennaDiversityRegister1				= 0x28,
  AntennaDiversityRegister2				= 0x29,
  AFCLimiter						= 0x2A,
  DataAccessControl 					= 0x30,                          
  EZmacStatus 						= 0x31,                                  
  HeaderControl1 					= 0x32,                               
  HeaderControl2 					= 0x33,                              
  PreambleLength 				        = 0x34,                               
  PreambleDetectionControl 				= 0x35,     
  /*  ͬ����              */               
  SyncWord3 						= 0x36,                                   
  SyncWord2 						= 0x37,                                   
  SyncWord1 						= 0x38,                               
  SyncWord0 						= 0x39,    
                              
  TransmitHeader3					= 0x3A,                       
  TransmitHeader2 					= 0x3B,                             
  TransmitHeader1 					= 0x3C,                              
  TransmitHeader0 					= 0x3D,  
  /* ���Ͱ�����            */                             
  TransmitPacketLength 					= 0x3E, 
   /* ��ͷУ��     */                        
  CheckHeader3 						= 0x3F,                                
  CheckHeader2 					        = 0x40,                              
  CheckHeader1 						= 0x41,                             
  CheckHeader0 						= 0x42,  
  /* ��ͷʹ�ܣ���λΪ��λ           */                            
  HeaderEnable3 					= 0x43,                               
  HeaderEnable2 					= 0x44,                                 
  HeaderEnable1 					= 0x45,                                
  HeaderEnable0 					= 0x46,  
                              
  ReceivedHeader3 					= 0x47,                          
  ReceivedHeader2 					= 0x48,                         
  ReceivedHeader1 					= 0x49,                           
  ReceivedHeader0 					= 0x4A,   
  /* ���հ�����            */                          
  ReceivedPacketLength				        = 0x4B,

  AnalogTestBus 					= 0x50,                              
  DigitalTestBus 					= 0x51,                          
  TXRampControl 					= 0x52,                             
  PLLTuneTime 						= 0x53,                            
  CalibrationControl 				        = 0x55,                     
  ModemTest 						= 0x56,                               
  ChargepumpTest 					= 0x57,                    
  ChargepumpCurrentTrimming_Override 		        = 0x58,         
  DividerCurrentTrimming				= 0x59,    
  VCOCurrentTrimming 					= 0x5A,                           
  VCOCalibration_Override 				= 0x5B,                    
  SynthesizerTest 					= 0x5C,                              
  BlockEnableOverride1 					= 0x5D,                        
  BlockEnableOverride2 					= 0x5E,                      
  BlockEnableOverride3 					= 0x5F,                       
  ChannelFilterCoefficientAddress 			= 0x60,             
  ChannelFilterCoefficientValue 			= 0x61,            
  CrystalOscillator_ControlTest 			= 0x62,               
  RCOscillatorCoarseCalibration_Override 	        = 0x63,    
  RCOscillatorFineCalibration_Override 		        = 0x64,      
  LDOControlOverride 					= 0x65,                          
  DeltasigmaADCTuning1			 		= 0x67,
  DeltasigmaADCTuning2			 		= 0x68,
  AGCOverride1					 	= 0x69,
  AGCOverride2 					        = 0x6A,
  GFSKFIRFilterCoefficientAddress 			= 0x6B,            
  GFSKFIRFilterCoefficientValue 			= 0x6C, 
  /* ���书��                           */              
  TXPower 						= 0x6D,  
  /* ���������趨�Ĵ���                 */                                  
  TXDataRate1 						= 0x6E,                            
  TXDataRate0 						= 0x6F, 
                               
  ModulationModeControl1 				= 0x70,                   
  ModulationModeControl2 				= 0x71,
  /* ��AFCʹ�ܣ����ڽ���״̬ʱ���趨����������Ƶ���ƶ�  */                    
  FrequencyDeviation 					= 0x72,   
  /* ��AFCʹ�ܣ���TX��RX���Ƶ��ƫ�����0x73��0x74��  */                         
  FrequencyOffset 					= 0x73,                            
  FrequencyChannelControl				= 0x74,
  FrequencyBandSelect 					= 0x75,                        
  NominalCarrierFrequency1	 			= 0x76,                    
  NominalCarrierFrequency0 				= 0x77,                    
  FrequencyHoppingChannelSelect 			= 0x79,               
  FrequencyHoppingStepSize 				= 0x7A,                     
  TXFIFOControl1 				        = 0x7C,                        
  TXFIFOControl2 				        = 0x7D,    
  RXFIFOControl 					= 0x7E,                               
  FIFOAccess						= 0x7F, 
} RF_REG_MAP;
    
typedef enum _RF_SAMPLE_SETTINGS
{
	DR2400BPS_DEV36KHZ			= 0,	
	DR4800BPS_DEV45KHZ			= 1,	
	DR9600BPS_DEV45KHZ			= 2,	
	DR19200BPS_DEV9_6KHZ		= 3,	
	DR38400BPS_DEV19_2KHZ		= 4,	
	DR57600BPS_DEV28_8KHZ		= 5,	
	DR115200BPS_DEV57_6KHZ		= 6,	
	
} RF_SAMPLE_SETTINGS;

/*      �ṩ���ⲿ���õĺ���         */ 
extern void dellayxms(unsigned int uiNum);                              /* ��ʱuiNum���룬����      */
extern unsigned char SpiReadRegister (unsigned char ucReg);             /* ��ȡ�Ĵ���ucReg��ֵ          */
extern void SpiWriteRegister(unsigned char ucReg,unsigned char ucData); /* ��Ĵ���ucReg��д��ucData    */

extern unsigned char IRQ_STATE(void);                                   /* ��ȡģ��IRQ���ŵĵ�ƽ        */

extern void Tx_Pwr_set(unsigned char upPwr);                            /* ���÷��书��                 */
extern void Data_Rate_set(unsigned char setting);                       /* �����������ʣ���η�Χ��0~7  */
extern void PreambleLength_Set(unsigned int ucBit);                     /* ���������볤��               */
extern void PreamblsheThreold_Set(unsigned int uiBit);                  /* �����������ż�               */

extern void RF_Init( void );                                            /* ��ʼ��ģ��                   */
extern unsigned char RFSendPacket(unsigned char *txBuffer, unsigned char size);   /* ����һ������       */
/* �������Ͷ�����ݣ�����ʱ�䳤��һ��LDC���ڣ�����LDCģʽ�µ�ģ�� */
extern void RF_SendWakeUp_Packet(unsigned int uiMs);          
extern unsigned char RFReacPacket(unsigned char *rxBuffer);             /* ��ȡģ���յ�������           */

extern void Rx_Mode_Entern (void);                                      /* �������ģʽ                 */
extern void RF_STANDBY_MODE(void);                                      /* ����ģʽ�������µ繦�����   */
extern void RF_SLEEP_MODE(void);                                        /* ����˯��ģʽ������1.4uA,     */
extern void RF_READY_MODE(void);                                        /* ����׼��ģʽ������1.5mA      */
extern void RF_LDC_MODE(void);                                          /* ����LDCģʽ�������߻���      */

 /* V1.01�������������������º�����date:2013-11-22      */
extern unsigned char DestinMac_Set(unsigned char ucByteNum,unsigned char *ucpDestin);   
extern unsigned char SourceMac_Set(unsigned char ucByteNum,unsigned char *ucpSource);
extern unsigned char Address_Lgth_Set(unsigned char ucByteNum,unsigned char EN_BroadCast); 
extern void Read_RecAddree(unsigned char *rxAddress); 
extern void RxFIFOReset(void);
extern void TxFIFOReset(void);
extern unsigned char Read_Rssi(void);
extern void Rssi_Offset_Set(unsigned char ucOffset);
void Set_CCA_Threshold(signed char rssi_threshold);

//-----------------------------------------------------------
//---------------------si4432��ʼ��--------------------------
void Si4432_Init(void);

unsigned char Set_RF_Speed(unsigned int uiTxSpeed);

void Si433_CHNL_STEP_Set(unsigned char ucChnl, unsigned int uiStep);

void Set_CCA_Threshold(signed char rssi_threshold);

void Rx_Mode_Entern(void);

//si4432�������ݰ�
unsigned char Si4432_RfReceivePacket(unsigned char *ucRxbuf, unsigned char *RxNumber);

bool CCA(unsigned char *rssi);

void dellayxms(unsigned int uiNum);

void RF_STANDBY_MODE(void);

void dellayx100us(unsigned int uiNum);

void Read_All_Registers(unsigned char *buffer);


void Errata(void);

#endif 
/*********************************************************************************************************
  End Of File
*********************************************************************************************************/
