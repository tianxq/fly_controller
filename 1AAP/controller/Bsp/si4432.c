/****************************************Copyright (c)****************************************************
**                            Guangzhou ZLGMCU Development Co., LTD
**
**                                 http://www.zlgmcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           si433.c
** Last modified Date:  2013-11-22
** Last Version:        V1.01
** Descriptions:        V1.01��2013-11-18����V1.00�Ļ��������ӵ�ַ���ܡ� 
**
**--------------------------------------------------------------------------------------------------------
** Created by:          yanghongyu
** Created date:        2013-06-08
** Version:             V1.00
** Descriptions:        ��д����
**
**--------------------------------------------------------------------------------------------------------
** Modified by:         
** Modified date:       
** Version:             
** Descriptions:        
**
**--------------------------------------------------------------------------------------------------------
** Modified by:        
** Modified date:      
** Version:            
** Descriptions:       
**
** Rechecked by:
*********************************************************************************************************/
#include "includes.h"                                                  /* LPC11xx����Ĵ���            */			
#include <stdbool.h>
//#include "FIFO_App.h"
//#include "gpio.h"

volatile uint8_t RF_Channel = 0;

/*  ��ֵʱ��Ҫ�޸ĵĺ궨��    */	
#define  ZM470S_ABC                                      /*  ��������ZM470S-ABC��ZM470S-D������ģ��   */

/* ���ڿ���SPI�������ʣ����ݲ�ͬ��MCU�ٶ�����������ǰΪMCU��ƵΪ48MHz    */
//#define TIME   1          

#define  PACKET_NUM         3                                           /* ���Ѱ������ݳ���             */
#define  PACKET_DATA        0xaa                                        /* ���Ѱ�����������             */
   
   
#define HEADER_DISABLE   0  
#define HEADER_ENABLE    1       

#define HEADER_STAT     HEADER_DISABLE // HEADER_ENABLE //

#if HEADER_ENABLE == HEADER_STAT 

unsigned char ucSourceMac[4] = {0x01,0x02,0x03,0x04};         /* ������ַ                     */
unsigned char ucDestinMac[4] = {0x05,0x06,0x07,0x08};					/* Ŀ�ĵ�ַ                     */

#endif



/*********************************************************************************************************
** Function name:        dellay
** Descriptions:         ��ʱһС��ʱ��
** input parameters:     i ��������
** output parameters:    ��
*********************************************************************************************************/
#if 0
void dellay(unsigned int i)
{
   unsigned int j,k;
   
   for (j = 0; j <1; j++)
	 {
	   for (k = 0; k < i; k++);
   }
}
#else
volatile void dellay(unsigned int i)
{
   unsigned int j,k;
   
   for (k = 0; k < i; k++)
	 {
	   for (j = 0; j<(72); j++);
   }
}
#endif

/*********************************************************************************************************
** Function name:        dellayms
** Descriptions:         ��ʱ���ɺ���
** input parameters:     uiNum: ��������
** output parameters:    ��
*********************************************************************************************************/
void dellayxms(unsigned int uiNum)
{
   unsigned int i = 0;
   unsigned int j = 0;
   while(i < uiNum) 
   {
     i++;
     for(j = 0; j < 8000; j++);//j < 1200
   }
}

void dellayx100us(unsigned int uiNum)
{
   unsigned int i = 0;
   unsigned int j = 0;
   while(i < uiNum) 
   {
     i++;
     for(j = 0; j < 800; j++);//j < 120
   }
}
/*********************************************************************************************************
** Function name:       IRQ_STATE
** Descriptions:        ����IRQ���ŵĵ�ƽ״̬
** input parameters:    ��
** output parameters:   ��
*********************************************************************************************************/
unsigned char IRQ_STATE(void)
{
	unsigned char state = 0;

	state = IRQ_READ();

    return ( state );
}
/*********************************************************************************************************
** Function name:       Si433_PinInit
** Descriptions:        ��ʼ����ZM470S���ӵ�����
** input parameters:    ��
** output parameters:   ��
*********************************************************************************************************/

/*************************************************************************************
*SPI IO��ʼ��
*************************************************************************************/
void SI4432SPI_LowLevel_Init(void)
{

//  /* sA7129_SPI Periph clock enable */
//  CLK_PeripheralClockConfig(CC11xx_SPI_CLK, ENABLE);

//  /* Set the MOSI,MISO and SCK at high level */
//  GPIO_ExternalPullUpConfig(CC11xx_SPI_SCK_GPIO_PORT, CC11xx_SPI_SCK_PIN | \
//                            CC11xx_SPI_MISO_PIN | CC11xx_SPI_MOSI_PIN, ENABLE);

//  /* SPI2 pin remap on Port I*/
//  //SYSCFG_REMAPPinConfig(REMAP_Pin_SPI2Full, ENABLE);

//  /* Configure FLASH_CS as Output push-pull, used as Flash Chip select */
//  GPIO_Init(CC11xx_CS_GPIO_PORT, CC11xx_CS_PIN,GPIO_Mode_Out_PP_High_Fast);//GPIO_Mode_Out_PP_High_Slow
}
/***************************************************************************************
*������:void SpiInit(void)
*���룺��
*�������
*����������SPI��ʼ������,����ʵ�ʵ�·���ӳ�ʼ��
***************************************************************************************/
void SI4432SpiInit(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* ʹ�� SPI2 & GPIOB ʱ�� */	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE); 

	/* Configure SPI1 pins: NSS, SCK, MISO and MOSI */ 
	GPIO_InitStructure.GPIO_Pin = CC11xx_SPI_SCK_PIN | CC11xx_SPI_MISO_PIN | CC11xx_SPI_MOSI_PIN; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(CC11xx_SPI_GPIO_PORT, &GPIO_InitStructure); 

    //SPI1 NSS  
	GPIO_InitStructure.GPIO_Pin = CC11xx_CS_PIN; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(CC11xx_CS_GPIO_PORT, &GPIO_InitStructure); 

	GPIO_SetBits(CC11xx_CS_GPIO_PORT, CC11xx_CS_PIN);   

	/* SPI configuration */ 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 					//SPI����Ϊ����ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	                   			//����SPIΪ��ģʽ
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                  	//SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	 		               		//����ʱ���ڲ�����ʱ��ʱ��Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;		               			//�ڶ���ʱ���ؿ�ʼ��������
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;			               		//NSS�ź��������ʹ��SSIλ������
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;	//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ8
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;				   		//���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 0x07;			//CRCֵ����Ķ���ʽ
	SPI_Init(CC11xx_SPI, &SPI_InitStructure);
	/* Enable SPI  */
	SPI_Cmd(CC11xx_SPI, ENABLE);	
	
}

/*****************************************************************************************
*��������SpisendByte(INT8U dat)
*���룺���͵�����
*�������
*����������SPI����һ���ֽ�
*****************************************************************************************/
unsigned char SpiTxRxByte(unsigned char dat)
{
  /* Loop while DR register in not emplty */
  //while (SPI_GetFlagStatus(CC11xx_SPI, SPI_FLAG_TXE) == RESET);
	while((SPI2->SR &SPI_I2S_FLAG_TXE)==RESET);  
  /* Send byte through the SPI peripheral */
  //SPI_SendData(CC11xx_SPI, dat);
	SPI2->DR = dat;
  
  /* Wait to receive a byte */
  //while (SPI_GetFlagStatus(CC11xx_SPI, SPI_FLAG_RXNE) == RESET);
	while((SPI2->SR &SPI_I2S_FLAG_RXNE)==RESET);
  /* Return the byte read from the SPI bus */
  //return SPI_ReceiveData(CC11xx_SPI);
	return(SPI2->DR);
}

//si4432 IRQ--SDN�ܽų�ʼ��
void Si4432_IO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
#ifdef rf_recv_it //�жϽ���ģʽ
	
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
 
    GPIO_InitStructure.GPIO_Pin = Si4432_IRQ_Pin; //
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //
    GPIO_Init(Si4432_IRQ_Port,&GPIO_InitStructure); 

	
    EXTI_ClearITPendingBit(EXTI_Line0);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

    EXTI_InitStructure.EXTI_Line = EXTI_Line0; 
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //�ж����󣬷��¼�����
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //�����ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;        
    EXTI_Init(&EXTI_InitStructure);	
	

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);          
        
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;     //ѡ���ж�ͨ��0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռʽͨ�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //��Ӧʽ�ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
	NVIC_Init(&NVIC_InitStructure);
#else
    //GPIO_Init( Si4432_IRQ_Port , Si4432_IRQ_Pin, GPIO_Mode_In_FL_No_IT);
	GPIO_InitStructure.GPIO_Pin = Si4432_IRQ_Pin; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(Si4432_IRQ_Port, &GPIO_InitStructure); 
#endif
  
  //  GPIO_Init( Si4432_SDN_Port , Si4432_SDN_Pin, GPIO_Mode_Out_PP_High_Slow);  
  	GPIO_InitStructure.GPIO_Pin = Si4432_SDN_Pin; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(Si4432_SDN_Port, &GPIO_InitStructure); 
}
/*********************************************************************************************************
** Function name:       SpiSendByte
** Descriptions:        �����������Ϸ���һ���ֽ�
** input parameters:    senddata:Ҫ���͵�����
** output parameters:   ��
*********************************************************************************************************/
void SpiSendByte(unsigned char senddata)
{
    SpiTxRxByte(senddata);  
}
/*********************************************************************************************************
** Function name:       SpiRCVByte
** Descriptions:        �����������Ͻ���һ���ֽ�
** input parameters:    ��
** output parameters:   ����������
*********************************************************************************************************/
unsigned char SpiRCVByte(void)
{
  return SpiTxRxByte(0);
}
/*********************************************************************************************************
** Function name:       SpiReadRegister
** Descriptions:        ��Si433�Ĵ�����ֵ
** input parameters:    reg:�Ĵ����ĵ�ַ
** output parameters:   ������ֵ
*********************************************************************************************************/
unsigned char SpiReadRegister (unsigned char ucReg)
{
	unsigned char ucTemp;
	
	SEL_L();
	SpiSendByte(ucReg);
  ucTemp = SpiRCVByte();
	SEL_H();
	return ucTemp;	
}
/*********************************************************************************************************
** Function name:       SpiWriteRegister
** Descriptions:        ��Si433�Ĵ�����д���ֽ�
** input parameters:    reg:��д��ļĴ�����ַ; data:д�������
** output parameters:   ��
*********************************************************************************************************/
void SpiWriteRegister(unsigned char ucReg,unsigned char ucData)
{	
	SEL_L();
	SpiSendByte(ucReg|0x80);
	SpiSendByte(ucData);
	SEL_H();
}
/*********************************************************************************************************
** Function name:       Si433_Reset
** Descriptions:        ��λSi4332оƬ
** input parameters:    ��
** output parameters:   ��
*********************************************************************************************************/
void Si433_Reset(void)
{
  unsigned char i=0;
  
#ifdef   ZM470S_ABC  	                                                /* ZM470S-Dģ��û��SDN����      */
        SDN_H();
		dellayxms(1);                                                        
		SDN_L();
#endif     
        dellayxms(20);                                                      /* ������15MS����               */	
        
	SpiReadRegister(InterruptStatus1);	                                /* ����жϱ�־                 */
	SpiReadRegister(InterruptStatus2);	
										 
        dellayxms(5);
        SpiWriteRegister(OperatingFunctionControl1, 0x80);				    /* �����λ                     */										 
	dellayxms(1); 
	while ( IRQ_READ() )
        {
          SpiWriteRegister(OperatingFunctionControl1, 0x80);
          dellayxms(1); 
          i++;
          if(i>10)break;
        };                                               /* �ȴ������λ�ɹ��ж�         */	
	SpiReadRegister(InterruptStatus1);	
	SpiReadRegister(InterruptStatus2);                                  /* ����жϱ�־                 */
   
        dellayxms(1);
}
/*********************************************************************************************************
** Function name:       Rx_Mode_Entern
** Descriptions:        �������״̬
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Rx_Mode_Entern(void)
{
	unsigned char i = 0;
    i = SpiReadRegister(InterruptStatus1);                              /* ����ж���ɱ�־             */
    i = SpiReadRegister(InterruptStatus2);        
    for(i = 0; i < 50;i++);
    SpiWriteRegister(InterruptEnable1, 0x03);                           /* �������յ�һ֡��Ч�����ж�   */   	                                                                    /* �Լ���������CRC�����ж�      */ 
    SpiWriteRegister(InterruptEnable2, 0x00);                            
   /* �������״̬��������18.5mA����   */   
    SpiWriteRegister(OperatingFunctionControl2, 0x00);
    SpiWriteRegister(OperatingFunctionControl1, 0x05);                  /* �������״̬                 */	
}
/*********************************************************************************************************
** Function name:       PreambleLength_Set
** Descriptions:        ���������볤��
** input parameters:    uiBit: ��bitΪ��λ������Ϊ4��������;��Χ4 ~ (4 �� 255)��һ��Ӧ����Ϊ 40��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void PreambleLength_Set(unsigned int uiBit)      
{
   if(uiBit < 4) {                 
        SpiWriteRegister(PreambleLength, 0x01);				            /* ��Ϊ��Сֵ                   */	
   } else if(uiBit < 8220){
      	SpiWriteRegister(PreambleLength, uiBit / 4);					/* ���������볤��               */
   } else {
        SpiWriteRegister(PreambleLength, 255);						    /* ����������Ϊ���ֵ           */
   }
}
/*********************************************************************************************************
** Function name:       PreambleThreshold_Set
** Descriptions:        �����������ż�����
** input parameters:    uiBit: ��bitΪ��λ������Ϊ4��������;��Χ4 ~ (4 �� 32)��һ��Ӧ����Ϊ 20��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void PreamblsheThreold_Set(unsigned int uiBit)      
{
   if(uiBit < 4) {                 
      SpiWriteRegister(PreambleDetectionControl, 0x08 | 0x02);	        /* �����������ż�               */	
   } else if(uiBit < 128){
        uiBit   = uiBit / 4;
        uiBit <<= 3;
      	SpiWriteRegister(PreambleDetectionControl, uiBit | 0x02);	    /* ���������볤��,λ��bit7~bit3 */
   } else {
        SpiWriteRegister(PreambleDetectionControl, 0xf8 | 0x02);		/* ����������Ϊ���ֵ           */
   }
}
/*********************************************************************************************************
** Function name:       Tx_Pwr_set
** Descriptions:        ���÷��书��
** input parameters:    upPwr����Χ0��7
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Tx_Pwr_set(unsigned char ucPwr)
{
   if(ucPwr < 8) {                 
      ucPwr = ucPwr &0x07;
      SpiWriteRegister(TXPower, 0x18|ucPwr);                            /* ���÷��书��                 */	
   } else {
      SpiWriteRegister(TXPower, 0x1f);                                  /* �����β������������     */
   }
}
/*********************************************************************************************************
** Function name:       Set_RF_Speed
** Descriptions:        �����ز�Ƶ��
** input parameters:    uiTxSpeed:�ز�Ƶ����MHzΪ��λ; 
** output parameters:   
*********************************************************************************************************/
unsigned char Set_RF_Speed(unsigned int uiTxSpeed)
{	
	unsigned char  hbsel = 0;
	unsigned char  fb    = 0;
	unsigned int   fc    = 0;

	if(uiTxSpeed < 480) {
	   hbsel = 0;
	   fb = (uiTxSpeed / 10) - 24;
       fc = (uiTxSpeed % 10) * 6400; 	   
	} else {
	   hbsel = 1 << 5;
	   fb = (uiTxSpeed / 20) - 24;
       fc = (uiTxSpeed % 20) * (6400 / 2); 	   		
	}

	SpiWriteRegister(FrequencyBandSelect, (0x40 | hbsel | fb));		    /* �ز��ߵ�Ƶ��ѡ����������   */	             
	SpiWriteRegister(NominalCarrierFrequency1, (fc/256));				/* �ز�С�����ֵ��ֽ�           */			 
	SpiWriteRegister(NominalCarrierFrequency0, (fc%256));  				/* �ز�С�����ָ��ֽ�           */				 
		
	return 0; 		   
}
/*********************************************************************************************************
** Function name:       Si433_CHNL_STEP_Set
** Descriptions:        ����Ƶ����Ƶ������
** input parameters:    Ƶ����ucChnl;Ƶ������uiStep(��KHzΪ��λ��Χ��0 - 2550 KHz)
** output parameters:   ��
*********************************************************************************************************/
void Si433_CHNL_STEP_Set(unsigned char ucChnl, unsigned int uiStep)
{	
	
/********************************************************************
   ���ʹ�ñ�����д�벽����д��ͨ����
   д��ͨ������������IC�����ز�Ƶ��  
*********************************************************************/
    SpiWriteRegister(FrequencyHoppingStepSize , (uiStep / 10));	        /* Ƶ����Ĳ�����               */
	SpiWriteRegister(FrequencyHoppingChannelSelect , ucChnl);			/* Ƶ��ѡ��                     */		
}
/*********************************************************************************************************
** Function name:       Data_Rate_set
** Descriptions:        ��������
** input parameters:    setting����Χ0��6
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Data_Rate_set(unsigned char setting)
{
	const unsigned char RMRfSettings[7][14] =		
	{
	//revB1
	//	 IFBW, COSR, CRO2, CRO1, CRO0, CTG1, CTG0, TDR1, TDR0, MMC1, FDEV,	AFC,	AFCTimingControl, AFCLimiter	
		{0x1D, 0x41, 0x60, 0x27, 0x52, 0x00, 0x05, 0x13, 0xa9, 0x20, 0x3a,  0x40,  0x0A, 0x20}, //DR: 2.4kbps, DEV: +-36kHz, BBBW: 75.2kHz (crystal:20 ppm)
		{0x1D, 0xA1, 0x20, 0x4E, 0xA5, 0x00, 0x0A, 0x27, 0x52, 0x20, 0x48, 	0x40,  0x0A, 0x20},	//DR: 4.8kbps, DEV: +-45kHz, BBBW: 95.3kHz (crystal:20 ppm)
		{0x1E, 0xD0, 0x00, 0x9D, 0x49, 0x00, 0x24, 0x4e, 0xa5, 0x20, 0x48, 	0x40,  0x0A, 0x20},	//DR: 9.6kbps, DEV: +-45kHz, BBBW: 112.8kHz (crystal:20 ppm)
		{0x03, 0xD0, 0x00, 0x9D, 0x49, 0x01, 0x3D, 0x9D, 0x49, 0x20, 0x0F,	0x40,  0x0A, 0x1E},	//DR: 19.2kbps, DEV: +-9.6kHz, BBBW: 75.2kHz (crystal:20 ppm)
		{0x03, 0x68, 0x01, 0x3A, 0x93, 0x02, 0x78, 0x09, 0xD5, 0x00, 0x1F,	0x40,  0x0A, 0x20},	//DR: 38.4kbps, DEV: +-19.2kHz, BBBW: 83.2kHz (crystal:20 ppm)  
		{0x06, 0x45, 0x01, 0xD7, 0xDC, 0x03, 0xB8, 0x0E, 0xBF, 0x00, 0x2E,	0x40,  0x0A, 0x2D},	//DR: 57.6kbps, DEV: +-28.8kHz, BBBW: 124.6kHz (crystal:20 ppm)
		{0x82, 0x68, 0x01, 0x3A, 0x93, 0x02, 0x78, 0x1D, 0x7E, 0x00, 0x5C,	0x40,  0x0A, 0x50}	//DR: 115.2kbps, DEV: +-57.6kHz, BBBW: 172.8kHz (crystal:20 ppm)
	 };

     if(setting > 7) {
        setting = 0;   /*  �����β�������Ϊ���    */
     }
		//set the registers according the selected RF settings in the range mode
		SpiWriteRegister( IFFilterBandwidth, RMRfSettings[setting][0] ); // IFFilterBandwidth  	= 0x1C, 
		SpiWriteRegister( ClockRecoveryOversamplingRatio, RMRfSettings[setting][1]);//ClockRecoveryOversamplingRatio  	= 0x20,      
		SpiWriteRegister( ClockRecoveryOffset2, RMRfSettings[setting][2]); //ClockRecoveryOffset2 	= 0x21, 
		SpiWriteRegister( ClockRecoveryOffset1, RMRfSettings[setting][3]); //ClockRecoveryOffset1 	= 0x22,  
		SpiWriteRegister( ClockRecoveryOffset0, RMRfSettings[setting][4]); //ClockRecoveryOffset0 	= 0x23,   
		SpiWriteRegister( ClockRecoveryTimingLoopGain1, RMRfSettings[setting][5]); //ClockRecoveryTimingLoopGain1  	= 0x24, 
		SpiWriteRegister( ClockRecoveryTimingLoopGain0, RMRfSettings[setting][6]); //ClockRecoveryTimingLoopGain0 	= 0x25,   
		SpiWriteRegister( TXDataRate1, RMRfSettings[setting][7]); //TXDataRate1  = 0x6E,
		SpiWriteRegister( TXDataRate0, RMRfSettings[setting][8]); //TXDataRate0  = 0x6F,       
		SpiWriteRegister( ModulationModeControl1, RMRfSettings[setting][9]); // ModulationModeControl1  = 0x70, 
		SpiWriteRegister( FrequencyDeviation, RMRfSettings[setting][10]); // FrequencyDeviation  = 0x72,  
		SpiWriteRegister( AFCLoopGearshiftOverride, RMRfSettings[setting][11]);	//AFCLoopGearshiftOverride 	= 0x1D,
		SpiWriteRegister( AFCTimingControl, RMRfSettings[setting][12]);	//AFCTimingControl  = 0x1E,  
		SpiWriteRegister( AFCLimiter, RMRfSettings[setting][13]); //AFCLimiter  = 0x2A,
}
/*********************************************************************************************************
** Function name:       RF_LDC_Set
** Descriptions:        ���ÿ����߻���˯��״̬
** input parameters:    uiWUT: ��������ʱ������0.1msΪ��λ������Ϊ1000ms,������10000��
                        uiTLCD��һ�������ڵĽ���ʱ��������С��uiWUT����0.1msΪ��λ��
                        uiWUT��Χ��15~76000��uiTLCD��Χ��15~290������ֵ�����м���Ĵ���ֵ��
** Returned value:      
*********************************************************************************************************/
void RF_LDC_Set(uint32_t uiWUT, uint32_t uiTLCD)
{
/**
           ģ�����RF_LDC_MODE�󣬻��Զ��ڽ���״̬������״̬��ѭ���л���
           ���ڽ����ڼ�(��uiTLCDʱ����)ģ����յ������룬�����������ݽ���������
           ���ڽ����ڼ�û�м�⵽�����룬���л�������״̬�����ߺ�uiWUT - uiTLCD)/10 ms���л�����״̬��
 ---------------------------------------------------------------------------------------------------------
 |                                             һ������ʾ��ͼ��                                           |
 |                                                                                                        |                                                       
 |  ����ʱ���� |�� uiTLCD ��|                                                                               |
 |  ����ʱ��   |��        uiWUT          ��|                                                                |
 |  ����״̬�� |��   ���� ��|��   ����     ��|��  ����  ��|��    ����    ��|                                      |
 |   ������������    |��        ���� n         ��|��       ���� (n +1)     ��|��     ���� (n +2)     ��|  ������������      |         
 ----------------------------------------------------------------------------------------------------------                                                                                               

     ����ʱ�����㷽���� WUT  = 4 �� WTM[15:0] �� ��2��R�η���/32.768  (����)
     ����ʱ�����㷽���� TLCD = 4 �� LCD[7:0]  �� ��2��R�η���/32.768  (����)
**/ 
/**
     ���Ѱ����������ʣ�9600bps��   �������ż���4bits������ʱ�䣺3.5���롣
     ���Ѱ����������ʣ�115200bps�� �������ż���8bits������ʱ�䣺2.0���롣
**/         
/**
    ƽ��������(1.3 �� 0.6 + 8.5 �� 0.2 + ��uiTLCD �� 10 - 0.6 - 0.2���� 18.5) ��  (uiWUT �� 10) + 0.001   (mA)
**/

    uint8_t   ucR   = 0;
    uint8_t   ucLcd = 0;
    uint16_t  uiM   = 0;
     
     uiWUT = uiWUT;
     uiTLCD =  uiTLCD;        
     
     if(uiTLCD < 300 && uiWUT <76001 && uiTLCD < uiWUT) {
       ucR    = 0;
       ucLcd  = (uiTLCD * 4) / 5;
       uiM    = (uiWUT  * 4) / 5;

    	SpiWriteRegister(WakeUpTimerPeriod1, ucR);	    	            /* Rֵ=WTR[4:0]�����ֵ0x14     */ 							
    	SpiWriteRegister(WakeUpTimerPeriod2, uiM / 255);		        /* Mֵ��8λ��WTM[15:8]          */						
    	SpiWriteRegister(WakeUpTimerPeriod3, uiM % 255);		        /* Mֵ��8λ��WTM[7:0]           */						
    
    	SpiWriteRegister(LowDutyCycleModeDuration, ucLcd);              /* RXʱ����LCD[7:0]             */ 
     } else {                                                                   

    	SpiWriteRegister(WakeUpTimerPeriod1, 0x00);		                /* Rֵ=WTR[4:0]�����ֵ0x14     */ 							
    	SpiWriteRegister(WakeUpTimerPeriod2, 0x20);		                /* Mֵ��8λ��WTM[15:8]          */						
    	SpiWriteRegister(WakeUpTimerPeriod3, 0x00);		                /* Mֵ��8λ��WTM[7:0]           */						
    
    	SpiWriteRegister(LowDutyCycleModeDuration, 0x56);               /* RXʱ����LCD[7:0]             */ 
    }
}
/*********************************************************************************************************
** Function name:       void RF_Init( void )
** Descriptions:        ����ģ���ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void RF_Init( void )
{	
    CLI();   //���ж�
    Si433_Reset();                                                      /* Ӳ����λ���������λ         */ 
    EXTI_ClearITPendingBit(EXTI_Line0);
    SEI();	  //���ж�
    
    Set_RF_Speed(467);							/* ���ô����ز�Ƶ��  470 MHz    */	             
    
	RF_Channel=4;
    Si433_CHNL_STEP_Set(RF_Channel, 600);										 											 
	
    Tx_Pwr_set(4);							/* ���÷��书��                 */

    Data_Rate_set(DR115200BPS_DEV57_6KHZ);			        /* �����������ʼ�Ƶƫ           */	    

    PreambleLength_Set(40);                                             /* ���������볤��               */
    PreamblsheThreold_Set(20);                                          /* �����������ż�               */
	
    SpiWriteRegister(HeaderControl2, 0x02);				/* ����ͬ���볤��               */  
    SpiWriteRegister(SyncWord3, 0x2D);					/* ����ͬ����                   */  						 
    SpiWriteRegister(SyncWord2, 0xD4);					/* ����ͬ����                   */  						 

    SpiWriteRegister(DataAccessControl, 0x8D);				/* ʹ���շ���CRCУ��            */  
 /* ���ݴ�FIFO�з��͵��Ʒ�ʽΪGFSK	  */
    SpiWriteRegister(ModulationModeControl2, 0x63);			                            
 /* ���÷���FIFO��ʣ��0x0b������ʱ��
   �жϱ�־λ(0x03,itxffeam)��λ	  */
    //SpiWriteRegister(TXFIFOControl2, 0x0B);	                                   
                                  
    SpiWriteRegister(AGCOverride1, 0x60); 	                        /* �Զ��������	                */	

    SpiWriteRegister(GPIO2Configuration, 0x14);                         /* ��GPIO2����ΪGND             */	
    SpiWriteRegister(GPIO1Configuration, 0x12);                         /* ��GPIO1����ΪRF���ؿ��ƶ� 12 */	
    SpiWriteRegister(GPIO0Configuration, 0x15);                         /* ��GPIO0����ΪRF���ؿ��ƶ� 15 */

#ifdef   ZM470S_ABC  /* ZM470S-ABC�õľ�����ZM470S-D�õĲ�һ��	*/
	SpiWriteRegister(CrystalOscillatorLoadCapacitance, 0xB5);  	        /* ZM470S-ABC���񲹳�����:10pF  */
#else
	SpiWriteRegister(CrystalOscillatorLoadCapacitance, 0x55);  	        /* ZM470S-D���񲹳�����:9pf     */
#endif
	SpiWriteRegister(RXFIFOControl, 0x30);                              /* �������ݳ�������	            */	
	//SpiWriteRegister(RSSIThresholdForClearChannelIndicator, 0x80);      /* RSSI�����ź�ǿ��	            */
 SpiWriteRegister(RSSIThresholdForClearChannelIndicator, 70);
#if HEADER_ENABLE == HEADER_STAT
    SourceMac_Set(4,&ucSourceMac[0]);                                   /* ���ñ�����ַ	                */
    DestinMac_Set(4,&ucDestinMac[0]);									/* ����Ŀ�ĵ�ַ	                */
    /**����ʵ��ʹ�õ�ַ�ĳ��ȣ�����Ϊ��ַ������0��رյ�ַ����
       ������رչ㲥��ַ���ܣ��ڶ������Ϊ��0����չ㲥��ַ�� **/
    Address_Lgth_Set(2,1); 												
#endif

    SpiReadRegister(InterruptStatus1);                                  /* ����жϱ�־                 */
    SpiReadRegister(InterruptStatus2);
 
    //RF_LDC_Set(10000, 45);                                              /* ����LDC����1000ms,����4.5ms  */

//	Rx_Mode_Entern();	                                                /*  ���н���״̬                */
}
/*********************************************************************************************************
** Function name:       RFSendPacket
** Descriptions:        ����ģ�鷢������
** input parameters:    txBuffer-���͵�����ָ��  size-���͵����ݳ���
** Returned value:      ���͵����ݳ��� 
*********************************************************************************************************/


void RF_TUNE_MODE(void)
{
    SpiWriteRegister(OperatingFunctionControl1, 0x03);
}


unsigned char rf_send_failCnt=0;
unsigned char rf_send_failCnt1=0;
#define RF_SEND_Fail_RESET_MAX  10    //����10�η��ͳ�ʱ�͸�λ
unsigned char status;
unsigned char IStatus1=0;
unsigned char IStatus2=0;

unsigned char RFSendPacket(unsigned char *txBuffer, unsigned char size)
{
        unsigned int   i = 0;
	char  unNum = size;
	unsigned char *p = txBuffer;
	
        CLI();	
        //�ر������ж�
        SpiWriteRegister(InterruptEnable1, 0x00);
        SpiWriteRegister(InterruptEnable2, 0x00);
        //���֮ǰ���ڵ��ж�
        IStatus1 = SpiReadRegister(InterruptStatus1);
        IStatus2 = SpiReadRegister(InterruptStatus2);
        
        status = SpiReadRegister(DeviceStatus);
        
        //if( status & SI44xx_IDLE_STATE )
        {
          
        }
        
	if (unNum > 32) 
	{
	    return 0;
	} 
	else if (p == 0) 
	{
	    return 0;
	}
	dellayx100us(4);
	//IStatus1 = SpiReadRegister(InterruptStatus1);                              /* ����ж���ɱ�־             */
        //IStatus2 = SpiReadRegister(InterruptStatus2);
	SpiWriteRegister(TransmitPacketLength, unNum);
	
        for (i = 0; i < unNum; i++) 
	{                                       /* ����Ҫ���͵�����             */                                   
          SpiWriteRegister(FIFOAccess, p[i]);
        }
        
        SpiWriteRegister(InterruptEnable1, 0x04);                           /* ���������ж�                 */
        SpiWriteRegister(InterruptEnable2, 0x00);                                                 
                         
        IStatus1 = SpiReadRegister(InterruptStatus1);                              /* ����ж���ɱ�־             */
        IStatus2 = SpiReadRegister(InterruptStatus2);                                         
    
        SpiWriteRegister(OperatingFunctionControl1, 0x09);                  /* ���뷢��״̬                 */
        dellayx100us(8);
        
        
        status = SpiReadRegister(DeviceStatus);
        if((status&0x02)!=0x02)
        {
          //rf_send_failCnt1++;
          //if(rf_send_failCnt1>RF_SEND_Fail_RESET_MAX)
          {
            //�ȴ���λ
            //while(1);
          }
          
        }
        else
        {
          rf_send_failCnt1=0;
        }
        //Ƶ�ʴ���
        if((status&0x08)==0x08)
        {
          //�ȴ���λ
            //while(1);
        }
        //-------------
        i = 100;
        while(--i)
        {
          if(!IRQ_READ())
          {
            if((SpiReadRegister(EZmacStatus)&0x01)) break;
              
            //if(!IRQ_READ()) break;//IRQ pin is low ,the chip is not in TX state
            
            dellayx100us(1);
          }
          else
            dellayx100us(1);
        }
        EXTI_ClearITPendingBit(EXTI_Line0);
        SEI();
        status = SpiReadRegister(DeviceStatus);
        if(i==0)
        {
          rf_send_failCnt++;
          if(rf_send_failCnt>RF_SEND_Fail_RESET_MAX)
          {
            //�ȴ���λ
            //while(1);
            CLI();   //���ж�
            RF_Init();
            //Errata();
            EXTI_ClearITPendingBit(EXTI_Line0);
            SEI();	  //���ж�
          }
        }
        else
        {
          rf_send_failCnt=0;
        }

        //-------------
#if 0
        i = 0;
        while(IRQ_READ()) 
	{                                                 /* �ȴ���������ж�             */	
            dellayxms(1);
            i++;
            if(i > 1000) 
            {
               return 0;
            } 
	}
#endif
        dellayx100us(4);
	IStatus1 = SpiReadRegister(InterruptStatus1); 
        IStatus2 = SpiReadRegister(InterruptStatus2); 
	if ((IStatus1 & 0x04) != 0x04)
	{   
		Rx_Mode_Entern();
		/* ��ֹĳЩ��������������ж����Ų��������ַ���ʧ״̬          */          
	    return 0; 
	}
	Rx_Mode_Entern();
	return (unNum);
}
/*********************************************************************************************************
** Function name:       RF_SendWakeUp_Packet
** Descriptions:        �������Ͷ�������Ѵ���LDCģʽ�µ�ģ��
** input parameters:     
** Returned value:       
*********************************************************************************************************/
void RF_SendWakeUp_Packet(unsigned int uiMs)
{
    unsigned int   j,i = 0;

    SpiWriteRegister(InterruptEnable1, 0x04);                           /* ���������ж�                 */
    SpiWriteRegister(InterruptEnable2, 0x00);    
        
    i = SpiReadRegister(InterruptStatus2);                              /* ����ж���ɱ�־             */
    SpiWriteRegister(TransmitPacketLength, PACKET_NUM);
    for (i = 0; i < PACKET_NUM; i++) {                                  /* ����Ҫ���͵�����             */                                      
         SpiWriteRegister(FIFOAccess, PACKET_DATA);
    }
    for(j = 0; j < uiMs; j++) {  
           	                                    
        SpiWriteRegister(OperatingFunctionControl1, 0x09);              /* ���뷢��״̬                 */
   
        SpiReadRegister(InterruptStatus1);                              /* ����ж���ɱ�־             */           
        dellay(1000);
        if(j < uiMs - 1){
            for (i = 0; i < PACKET_NUM; i++) {                          /* ����Ҫ���͵�����             */                                      
                SpiWriteRegister(FIFOAccess, PACKET_DATA);
            } 
        }    
        while(IRQ_READ()) {                                             /* �ȴ���������ж�             */	
        		dellay(100);
        		i++;
        		if(i > 65530) {
        		    break;
        		} 
       }
    }
        i = SpiReadRegister(InterruptStatus1);                          /* ����ж���ɱ�־             */
        i = SpiReadRegister(InterruptStatus2);  
}
/*********************************************************************************************************
** Function name:       RFSendPacket
** Descriptions:        ����ģ��������� 
** input parameters:    rxBuffer-���յ�����ָ��  
** output parameters:   
** Returned value:      ���յ����ݳ���
*********************************************************************************************************/
unsigned char RFReacPacket(unsigned char *rxBuffer)
{
    unsigned char i = 0;
	unsigned char ucPck_Lgth = 0;
	unsigned char ucIrqstat1 = 0;
	unsigned char *ucData    = rxBuffer;

    ucIrqstat1 = SpiReadRegister(InterruptStatus1);                     /* ����ж���ɱ�־             */
                 SpiReadRegister(InterruptStatus2);

	if((ucIrqstat1 & 0x01) == 1) 
	{			                /* �����CRC���ձ�־��λ��������ս���FIFO  */		    		
	   	RxFIFOReset();
	    return 0;
	} 
	else if(!(ucIrqstat1 & 0x02))
	{                                    /* ��������жϱ�־û��λ       */   
  	RxFIFOReset();
		return 0;
	}
	           
	ucPck_Lgth = SpiReadRegister( ReceivedPacketLength );                /* ���յ��������ֽ�����0x4B��   */    
        if(ucPck_Lgth>RF_rx_SIZE)return 0;       
	for (i = 0; i < ucPck_Lgth; i++) 
	{                                   
			ucData[i] = SpiReadRegister(FIFOAccess);  
	} 



	return ucPck_Lgth;
}
/*********************************************************************************************************
** Function name:       RF_STANDBY_MODE
** Descriptions:        �������״̬
** input parameters:     
** Returned value:       
*********************************************************************************************************/
void RF_STANDBY_MODE(void)
{
    /* �������״̬����ʱ���0.9uA,;��ʱ800us���л������ͻ����״̬ */
    /* �����Ҫ���͹��ģ��ɰ�ģ���SDN�������ߣ��ر�ģ��ĵ�Դ�����ٴ�����ģ��ʱ��Ҫ�س�����ģ�� */
    SpiWriteRegister(OperatingFunctionControl2, 0x00);
    SpiWriteRegister(OperatingFunctionControl1, 0x0);                  /* ����˯��״̬  enlbd = 1      */
     
}
/*********************************************************************************************************
** Function name:       RF_SLEEP_MODE
** Descriptions:        ����˯��״̬
** input parameters:     
** Returned value:       
*********************************************************************************************************/
void RF_SLEEP_MODE(void)
{
    /* ����˯��״̬  enlbd = 1;��ʱ���1.4uA����ʱ800us���л������ͻ����״̬      */
    SpiWriteRegister(OperatingFunctionControl2, 0x00);
    SpiWriteRegister(OperatingFunctionControl1, 0x40);                  /* ����˯��״̬  enlbd = 1      */
     
}
/*********************************************************************************************************
** Function name:       RF_READY_MODE
** Descriptions:        MCU��������ģ�黽������ 
** input parameters:    
** Returned value:      
*********************************************************************************************************/
void RF_READY_MODE(void)
{
    /*  �ϵ���Ĭ��״̬��xton = 1;��ʱ���1.5mA����ʱ200us���л������ͻ����״̬      */
    SpiWriteRegister(OperatingFunctionControl2, 0x00); 
    SpiWriteRegister(OperatingFunctionControl1, 0x01);                  /* ����READY״̬                */    
}
/*********************************************************************************************************
** Function name:       RF_LDC_MODE
** Descriptions:        ���ÿ����߻���˯��״̬
** input parameters:    
** Returned value:      
*********************************************************************************************************/
void RF_LDC_MODE(void)
{
    /*  Щģʽ�ĵ����ɺ���void RF_LDC_Set(uint32_t uiWUT, uint32_t uiTLCD)����ξ�����
        ��: RF_LDC_Set(10000, 20), ƽ��������29uA ;
            RF_LDC_Set(10000, 35), ƽ��������54uA ;
    */
 
    SpiWriteRegister(InterruptEnable1, 0x03);                           /* �������յ�һ֡��Ч�����ж�   */   	                                                                    /* �Լ���������CRC�����ж�      */ 
    SpiWriteRegister(InterruptEnable2, 0x40);                           /* �������յ���Ч�������ж�     */ 
                                                                                                       
	SpiWriteRegister(OperatingFunctionControl1, 0x00);
	SpiWriteRegister(OperatingFunctionControl2, 0x04);                  /* ����LDC״̬                  */                     
                   
    SpiReadRegister(InterruptStatus1);										
	SpiReadRegister(InterruptStatus2);	                                /* ����жϱ�־λ               */ 
}
/*********************************************************************************************************
** Function name:       RxFIFOReset
** Descriptions:        ��λ���ջ�����
** input parameters:    ��
** Returned value:      ��
*********************************************************************************************************/
void RxFIFOReset(void)
{
    unsigned char i = 0;
	    
	i = SpiReadRegister(OperatingFunctionControl2);
	SpiWriteRegister(OperatingFunctionControl2, (0x02 | i)); 
	SpiWriteRegister(OperatingFunctionControl2, (0xfd & i)); 		                                                                                            
	
}
/*********************************************************************************************************
** Function name:       TxFIFOReset
** Descriptions:        ��λ���ͻ�����
** input parameters:    ��
** Returned value:      ��
*********************************************************************************************************/
void TxFIFOReset(void)
{
    unsigned char i = 0;
	    
	i = SpiReadRegister(OperatingFunctionControl2);
	SpiWriteRegister(OperatingFunctionControl2, (0x01 | i)); 
	SpiWriteRegister(OperatingFunctionControl2, (0xfe & i));
	
}

#if HEADER_ENABLE == HEADER_STAT 
/*********************************************************************************************************
** Function name:       SourceMac_Set
** Descriptions:        ������ַ����
** input parameters:    ucByteNum: ��ַ���ֽ�����
                        ucpSource: ��ַ����ָ�롣 
** output parameters:   ��
** Returned value:      ʵ��д��ĵ�ַ�ֽ�����
*********************************************************************************************************/
unsigned char SourceMac_Set(unsigned char ucByteNum,unsigned char *ucpSource)      
{

   unsigned char i;
   unsigned char *ptemp;
   
   ptemp = ucpSource;

   if (ptemp == 0) {
       return 0;                                                        /* ָ��Ϊ��                     */ 
   }

   for (i = 0; i < ucByteNum && i < 4; i++) {
   /* У���ַΪ0x3F~0x42; CheckHeader3 = 0x3f */   
       SpiWriteRegister(CheckHeader3 + i, *ptemp++);	   
   }

   return i;
}
/*********************************************************************************************************
** Function name:       DestinMac_Set
** Descriptions:        Ŀ���ַ����
** input parameters:    ucByteNum: ��ַ���ֽ�����
                        ucpSource: ��ַ����ָ�롣 
** output parameters:   ��
** Returned value:      ʵ��д��ĵ�ַ�ֽ�����
*********************************************************************************************************/
unsigned char DestinMac_Set(unsigned char ucByteNum,unsigned char *ucpDestin)      
{

   unsigned char i;
   unsigned char *ptemp;
   
   ptemp = ucpDestin;

   if (ptemp == 0) {
       return 0;                                                        /* ָ��Ϊ��                     */ 
   }

   for (i = 0; i < ucByteNum && i < 4; i++) {
   /* У���ַΪ0x3A~0x3D; TransmitHeader3 = 0x3A */   
       SpiWriteRegister(TransmitHeader3 + i, *ptemp++);	        
   }

   return i;
}
/*********************************************************************************************************
** Function name:       Address_Lgth_Set
** Descriptions:        ����ʵ��ʹ�õĵ�ַ����
** input parameters:    ucByteNum: ���õĵ�ַ���ȣ����ֽ�Ϊ��λ��
						EN_BroadCast��Ϊ0ʱ�������չ㲥������0ʱ���չ㲥����                 
** output parameters:   ��
** Returned value:      ʵ�������˵ĵ�ַ���ȣ����ֽ�Ϊ��λ��
*********************************************************************************************************/
unsigned char Address_Lgth_Set(unsigned char ucByteNum,unsigned char EN_BroadCast)      
{

    unsigned char hdlen = 0;		                        /* ���ͺͽ��հ�ͷ����hdlen[2:0]���ֽ�Ϊ��λ */ 
    unsigned char hdch  = 0;		                        /* ��ͷУ���ֽڣ���ӦλΪ1ʱʹ�ܣ�hdch[3:0] */ 
   	unsigned char bcen  = 0;								/* ���չ㲥��ʹ�ܣ�Ϊ1ʱ����Ӧ���ֽ�ʹ��    */ 

    unsigned char header3  = 0;
    unsigned char header2  = 0;
    unsigned char header1  = 0;
    unsigned char header0  = 0;

	unsigned ucCtrl2Temp   = 0;
    unsigned uctemp        = ucByteNum;
	

    if (uctemp > 4) {						                            /* ��γ�����Χ����Ϊ4�ֽ�      */ 
   	    uctemp = 4;
    }

	switch (uctemp) {
		case 0: hdch = 0; break;
	    case 1: header3 = 0xff; hdch = 0x08; break;	
	    case 2: header3 = 0xff, header2 = 0xff ; hdch = 0x0c; break;
	    case 3: header3 = 0xff, header2 = 0xff , header1 = 0xff ; hdch = 0x0e; break;
	    case 4: header3 = 0xff, header2 = 0xff , header1 = 0xff , header0 = 0xff; hdch = 0x0f; break;
	    default:  break;
	}

	if (0 != EN_BroadCast) {                                            /* �ж��Ƿ���չ㲥��ַ�İ�     */
		bcen = hdch;
	} else {
		bcen = 0;
	}

	hdlen = uctemp << 4;

	ucCtrl2Temp  = SpiReadRegister(HeaderControl2); 				    /* �ȶ�������Ϊ�˲��ı�����λ   */
	ucCtrl2Temp &= 0x8f;
	ucCtrl2Temp |= hdlen;
	SpiWriteRegister(HeaderControl2, ucCtrl2Temp);                      /* ���ð�ͷ����                 */
	 
	SpiWriteRegister(HeaderControl1, bcen << 4 | hdch);	                /* �㲥ʹ�ܼ�У��ʹ���ֽڳ���   */

    /* ���ã�Ϊ1ʱʹ�ܣ�У��ʹ���ֽڶ�Ӧ��λ */
    SpiWriteRegister(HeaderEnable3, header3);	                        /* hden[31:24]                  */   
	SpiWriteRegister(HeaderEnable2, header2);	                        /* hden[23:16]                  */
    SpiWriteRegister(HeaderEnable1, header1);	                        /* hden[15:8]                   */
    SpiWriteRegister(HeaderEnable0, header0);	                        /* hden[7:0]                    */

    return uctemp;
}
/*********************************************************************************************************
** Function name:       Read_RecAddree
** Descriptions:        ��ȡ�������յ��ĵ�ַ
** input parameters:    rxAddress����Ž��յ�ַ��ָ���׵�ַ���ֻ�����ֽڡ� 
** Returned value:       
*********************************************************************************************************/
void Read_RecAddree(unsigned char *rxAddress)
{
	*rxAddress = SpiReadRegister(ReceivedHeader3); 						/* ��ַ���׵��ֽ�               */
	rxAddress++;
	*rxAddress = SpiReadRegister(ReceivedHeader2); 
	rxAddress++;
	*rxAddress = SpiReadRegister(ReceivedHeader1); 
	rxAddress++;
 	*rxAddress = SpiReadRegister(ReceivedHeader0); 
   	    
}
#endif

/*********************************************************************************************************
** Function name:       Read_Rssi
** Descriptions:        ��ȡ�ź�ǿ�ȼĴ���
** input parameters:    ��
** Returned value:      ʵʱ�ź�ǿ�ȵ�ֵ 
*********************************************************************************************************/
unsigned char Read_Rssi(void)
{	
	return (SpiReadRegister(ReceivedSignalStrengthIndicator)); 
}
/*********************************************************************************************************
** Function name:       Rssi_Offset_Set
** Descriptions:        �����ź�ǿ��ƫ��
** input parameters:    ucOffset: ��Χ:0 ~ 7; ÿ����һRSSI�ͻ�����4dB,��RSSI�Ĵ���������ֵ������4*2=8��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Rssi_Offset_Set(unsigned char ucOffset)      
{
   unsigned char preath;

   preath  = SpiReadRegister(PreambleDetectionControl);
   preath   &=	0xf8;
   ucOffset &=  0x07;

   SpiWriteRegister(PreambleDetectionControl, preath | ucOffset);		/* �����ź�ǿ��ƫ�Ƽ��������ż� */  
}



/*********************************************************************************************************
** Function name:       Set_CCA_Threshold
** Descriptions:        ��������ŵ���������ֵ
** input parameters:    
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Set_CCA_Threshold(signed char rssi_threshold)
{
  SpiWriteRegister(RSSIThresholdForClearChannelIndicator, rssi_threshold);
}


/*********************************************************************************************************
** Function name:       CCA
** Descriptions:        ����ŵ�����
** input parameters:    
** output parameters:   ��
** Returned value:      true:  �ŵ����
**                      false: �ŵ���ռ��
*********************************************************************************************************/
bool CCA(unsigned char *rssi)
{
  uint8_t cca;
  
  Rx_Mode_Entern();
  
  dellay(5);
  
  cca = SpiReadRegister(InterruptStatus2);
  
  *rssi=Read_Rssi();
  /*If the signal strength is above the programmed threshold then the irssi bit 
    in the InterruptStatus2 regiter will set to 1.
  */
  cca &= 0x10; //get the irssi bit .
  
  return (!(bool)cca); 
}
//-----------------------------------------------------------
//---------------------si4432��ʼ��--------------------------
void Si4432_Init(void)
{
  //si4432 spi ��ʼ��
  SI4432SpiInit();
  //si4432 IO ��ʼ��
  Si4432_IO_Init();
  //si4432�������ó�ʼ��
  CLI();   //���ж�
  RF_Init();
  //Errata();
  EXTI_ClearITPendingBit(EXTI_Line0);
  SEI();	  //���ж�
}


/*

*/
uint8_t hex2Dec_Express(uint8_t hex)
{
  uint8_t dec;
  uint8_t mod;
  
  mod = (hex&0x80)? (~hex):hex; //??
  
  if( mod < 100 )
  {
    dec = (mod/10)*0x10;
    dec += (mod%10);
  }
  
  else if(mod<116)
  {
     dec = 0xA0+(mod%100); //??100?0xAx??
  }
  
  else
    dec = 0xff;
  
  return dec;
}

//si4432�������ݰ�
unsigned char Si4432_RfReceivePacket(unsigned char *ucRxbuf, unsigned char *RxNumber)
{
	unsigned char rssi;
	
	if(!IRQ_STATE())				/*  ?D??��y??��?��-�̨���?��?��?��y?Y    */ 				
	{		
		rssi = Read_Rssi(); 
	
		*RxNumber=RFReacPacket(ucRxbuf);					/*  �䨮ZM470S?��?��?��3?��y?Y        */

		Rx_Mode_Entern();	                    		/*  ??DD?����?���䨬?  */  
		
		if(*RxNumber > 0)
		{
			RxFIFOReset();
			//rssi = hex2Dec_Express(rssi);
		}
		
		rssi = 0;
		return 1;
	}
	return 0;
}




/****************************************************************************************
 *  @Func  : SpiReadRegister 
 *  
 *  @Brief : burst read mode, read sequential registers
 *  
 *  @param : reg_addr  ::
 *  
 *  @return: None
 *  
 ****************************************************************************************/
void SpiBurstReadRegister(unsigned char reg_addr, uint8_t len, uint8_t *buffer)
{
	int i;
	
	SEL_L(); //CS = 0
	
	SpiSendByte(reg_addr); //RW = 0,read
	
	for(i=0;i<len;i++)
	{
		buffer[i] = SpiRCVByte();
	}
    
	SEL_H(); //CS=1	
}



void Read_All_Registers(uint8_t *buffer)
{
  SpiBurstReadRegister( 0x00,0x80, buffer);
}


void Errata(void)
{
  SpiWriteRegister(DividerCurrentTrimming, 0x00); //59h
  SpiWriteRegister(VCOCurrentTrimming, 0x03);     //5ah
  SpiWriteRegister(0x66, 0x02);     //66h
  SpiWriteRegister(ChargepumpCurrentTrimming_Override, 0x11);   //58h
}

void RSSI_Errata(void)
{
  //SpiWriteRegister(AGCOverride2, 0x02);
  
}
/*********************************************************************************************************
  End Of File
*********************************************************************************************************/
