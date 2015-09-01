/****************************************Copyright (c)****************************************************
**                            Guangzhou ZLGMCU Development Co., LTD
**
**                                 http://www.zlgmcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           si433.c
** Last modified Date:  2013-11-22
** Last Version:        V1.01
** Descriptions:        V1.01：2013-11-18，在V1.00的基础上增加地址功能。 
**
**--------------------------------------------------------------------------------------------------------
** Created by:          yanghongyu
** Created date:        2013-06-08
** Version:             V1.00
** Descriptions:        编写驱动
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
#include "includes.h"                                                  /* LPC11xx外设寄存器            */			
#include <stdbool.h>
//#include "FIFO_App.h"
//#include "gpio.h"

volatile uint8_t RF_Channel = 0;

/*  移值时需要修改的宏定义    */	
#define  ZM470S_ABC                                      /*  用于区别ZM470S-ABC和ZM470S-D这两种模块   */

/* 用于控制SPI总线速率，根据不同的MCU速度来调整；当前为MCU主频为48MHz    */
//#define TIME   1          

#define  PACKET_NUM         3                                           /* 唤醒包的数据长度             */
#define  PACKET_DATA        0xaa                                        /* 唤醒包的数据内容             */
   
   
#define HEADER_DISABLE   0  
#define HEADER_ENABLE    1       

#define HEADER_STAT     HEADER_DISABLE // HEADER_ENABLE //

#if HEADER_ENABLE == HEADER_STAT 

unsigned char ucSourceMac[4] = {0x01,0x02,0x03,0x04};         /* 本机地址                     */
unsigned char ucDestinMac[4] = {0x05,0x06,0x07,0x08};					/* 目的地址                     */

#endif



/*********************************************************************************************************
** Function name:        dellay
** Descriptions:         延时一小段时间
** input parameters:     i 毫秒数据
** output parameters:    无
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
** Descriptions:         延时若干毫秒
** input parameters:     uiNum: 毫秒数据
** output parameters:    无
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
** Descriptions:        返回IRQ引脚的电平状态
** input parameters:    无
** output parameters:   无
*********************************************************************************************************/
unsigned char IRQ_STATE(void)
{
	unsigned char state = 0;

	state = IRQ_READ();

    return ( state );
}
/*********************************************************************************************************
** Function name:       Si433_PinInit
** Descriptions:        初始化与ZM470S连接的引脚
** input parameters:    无
** output parameters:   无
*********************************************************************************************************/

/*************************************************************************************
*SPI IO初始化
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
*函数名:void SpiInit(void)
*输入：无
*输出：无
*功能描述：SPI初始化程序,根据实际电路连接初始化
***************************************************************************************/
void SI4432SpiInit(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* 使能 SPI2 & GPIOB 时钟 */	
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
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 					//SPI设置为两线全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	                   			//设置SPI为主模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                  	//SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	 		               		//串行时钟在不操作时，时钟为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;		               			//第二个时钟沿开始采样数据
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;			               		//NSS信号由软件（使用SSI位）管理
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;	//定义波特率预分频的值:波特率预分频值为8
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;				   		//数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 0x07;			//CRC值计算的多项式
	SPI_Init(CC11xx_SPI, &SPI_InitStructure);
	/* Enable SPI  */
	SPI_Cmd(CC11xx_SPI, ENABLE);	
	
}

/*****************************************************************************************
*函数名：SpisendByte(INT8U dat)
*输入：发送的数据
*输出：无
*功能描述：SPI发送一个字节
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

//si4432 IRQ--SDN管脚初始化
void Si4432_IO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
#ifdef rf_recv_it //中断接收模式
	
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
 
    GPIO_InitStructure.GPIO_Pin = Si4432_IRQ_Pin; //
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //
    GPIO_Init(Si4432_IRQ_Port,&GPIO_InitStructure); 

	
    EXTI_ClearITPendingBit(EXTI_Line0);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

    EXTI_InitStructure.EXTI_Line = EXTI_Line0; 
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //中断请求，非事件请求
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //上下沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;        
    EXTI_Init(&EXTI_InitStructure);	
	

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);          
        
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;     //选择中断通道0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占式通道优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //响应式中断优先级
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
** Descriptions:        向数据总线上发送一个字节
** input parameters:    senddata:要发送的数据
** output parameters:   无
*********************************************************************************************************/
void SpiSendByte(unsigned char senddata)
{
    SpiTxRxByte(senddata);  
}
/*********************************************************************************************************
** Function name:       SpiRCVByte
** Descriptions:        从数据总线上接收一个字节
** input parameters:    无
** output parameters:   读到的数据
*********************************************************************************************************/
unsigned char SpiRCVByte(void)
{
  return SpiTxRxByte(0);
}
/*********************************************************************************************************
** Function name:       SpiReadRegister
** Descriptions:        读Si433寄存器的值
** input parameters:    reg:寄存器的地址
** output parameters:   读到的值
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
** Descriptions:        向Si433寄存器中写入字节
** input parameters:    reg:被写入的寄存器地址; data:写入的数据
** output parameters:   无
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
** Descriptions:        复位Si4332芯片
** input parameters:    无
** output parameters:   无
*********************************************************************************************************/
void Si433_Reset(void)
{
  unsigned char i=0;
  
#ifdef   ZM470S_ABC  	                                                /* ZM470S-D模块没有SDN引脚      */
        SDN_H();
		dellayxms(1);                                                        
		SDN_L();
#endif     
        dellayxms(20);                                                      /* 理论上15MS以上               */	
        
	SpiReadRegister(InterruptStatus1);	                                /* 清除中断标志                 */
	SpiReadRegister(InterruptStatus2);	
										 
        dellayxms(5);
        SpiWriteRegister(OperatingFunctionControl1, 0x80);				    /* 软件复位                     */										 
	dellayxms(1); 
	while ( IRQ_READ() )
        {
          SpiWriteRegister(OperatingFunctionControl1, 0x80);
          dellayxms(1); 
          i++;
          if(i>10)break;
        };                                               /* 等待软件复位成功中断         */	
	SpiReadRegister(InterruptStatus1);	
	SpiReadRegister(InterruptStatus2);                                  /* 清除中断标志                 */
   
        dellayxms(1);
}
/*********************************************************************************************************
** Function name:       Rx_Mode_Entern
** Descriptions:        进入接收状态
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Rx_Mode_Entern(void)
{
	unsigned char i = 0;
    i = SpiReadRegister(InterruptStatus1);                              /* 清除中断完成标志             */
    i = SpiReadRegister(InterruptStatus2);        
    for(i = 0; i < 50;i++);
    SpiWriteRegister(InterruptEnable1, 0x03);                           /* 开启接收到一帧有效数据中断   */   	                                                                    /* 以及开启接收CRC错误中断      */ 
    SpiWriteRegister(InterruptEnable2, 0x00);                            
   /* 进入接收状态；电流在18.5mA左右   */   
    SpiWriteRegister(OperatingFunctionControl2, 0x00);
    SpiWriteRegister(OperatingFunctionControl1, 0x05);                  /* 进入接收状态                 */	
}
/*********************************************************************************************************
** Function name:       PreambleLength_Set
** Descriptions:        设置引导码长度
** input parameters:    uiBit: 以bit为单位，必须为4的整数倍;范围4 ~ (4 × 255)；一般应用设为 40。
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void PreambleLength_Set(unsigned int uiBit)      
{
   if(uiBit < 4) {                 
        SpiWriteRegister(PreambleLength, 0x01);				            /* 设为最小值                   */	
   } else if(uiBit < 8220){
      	SpiWriteRegister(PreambleLength, uiBit / 4);					/* 设置引导码长度               */
   } else {
        SpiWriteRegister(PreambleLength, 255);						    /* 设置引导码为最大值           */
   }
}
/*********************************************************************************************************
** Function name:       PreambleThreshold_Set
** Descriptions:        设置引导码门槛长度
** input parameters:    uiBit: 以bit为单位，必须为4的整数倍;范围4 ~ (4 × 32)；一般应用设为 20。
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void PreamblsheThreold_Set(unsigned int uiBit)      
{
   if(uiBit < 4) {                 
      SpiWriteRegister(PreambleDetectionControl, 0x08 | 0x02);	        /* 设置引导码门槛               */	
   } else if(uiBit < 128){
        uiBit   = uiBit / 4;
        uiBit <<= 3;
      	SpiWriteRegister(PreambleDetectionControl, uiBit | 0x02);	    /* 设置引导码长度,位于bit7~bit3 */
   } else {
        SpiWriteRegister(PreambleDetectionControl, 0xf8 | 0x02);		/* 设置引导码为最大值           */
   }
}
/*********************************************************************************************************
** Function name:       Tx_Pwr_set
** Descriptions:        设置发射功率
** input parameters:    upPwr，范围0到7
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Tx_Pwr_set(unsigned char ucPwr)
{
   if(ucPwr < 8) {                 
      ucPwr = ucPwr &0x07;
      SpiWriteRegister(TXPower, 0x18|ucPwr);                            /* 设置发射功率                 */	
   } else {
      SpiWriteRegister(TXPower, 0x1f);                                  /* 如果入参不合理，设置最大     */
   }
}
/*********************************************************************************************************
** Function name:       Set_RF_Speed
** Descriptions:        设置载波频率
** input parameters:    uiTxSpeed:载波频率以MHz为单位; 
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

	SpiWriteRegister(FrequencyBandSelect, (0x40 | hbsel | fb));		    /* 载波高低频段选择及整数部分   */	             
	SpiWriteRegister(NominalCarrierFrequency1, (fc/256));				/* 载波小数部分低字节           */			 
	SpiWriteRegister(NominalCarrierFrequency0, (fc%256));  				/* 载波小数部分高字节           */				 
		
	return 0; 		   
}
/*********************************************************************************************************
** Function name:       Si433_CHNL_STEP_Set
** Descriptions:        设置频道及频道步进
** input parameters:    频道号ucChnl;频道步进uiStep(以KHz为单位范围：0 - 2550 KHz)
** output parameters:   无
*********************************************************************************************************/
void Si433_CHNL_STEP_Set(unsigned char ucChnl, unsigned int uiStep)
{	
	
/********************************************************************
   如果使用必需先写入步进所写入通道，
   写入通道结束会引起IC调整载波频率  
*********************************************************************/
    SpiWriteRegister(FrequencyHoppingStepSize , (uiStep / 10));	        /* 频道间的步进，               */
	SpiWriteRegister(FrequencyHoppingChannelSelect , ucChnl);			/* 频道选择                     */		
}
/*********************************************************************************************************
** Function name:       Data_Rate_set
** Descriptions:        数据速率
** input parameters:    setting：范围0到6
** output parameters:   无
** Returned value:      无
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
        setting = 0;   /*  如果入参不合理，设为最低    */
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
** Descriptions:        设置可无线唤醒睡眠状态
** input parameters:    uiWUT: 整个周期时长，以0.1ms为单位，如设为1000ms,则输入10000。
                        uiTLCD：一个周期内的接收时长，必须小于uiWUT，以0.1ms为单位。
                        uiWUT范围：15~76000；uiTLCD范围：15~290；其它值可自行计算寄存器值。
** Returned value:      
*********************************************************************************************************/
void RF_LDC_Set(uint32_t uiWUT, uint32_t uiTLCD)
{
/**
           模块进入RF_LDC_MODE后，会自动在接收状态和休眠状态间循环切换；
           若在接收期间(即uiTLCD时段内)模块接收到引导码，则会把整包数据接收下来；
           若在接收期间没有检测到引导码，则切换到休眠状态，休眠后（uiWUT - uiTLCD)/10 ms再切换接收状态；
 ---------------------------------------------------------------------------------------------------------
 |                                             一个周期示意图：                                           |
 |                                                                                                        |                                                       
 |  接收时长： |← uiTLCD →|                                                                               |
 |  周期时长   |←        uiWUT          →|                                                                |
 |  工作状态： |←   接收 →|←   休眠     →|←  接收  →|←    休眠    →|                                      |
 |   ………………    |←        周期 n         →|←       周期 (n +1)     →|←     周期 (n +2)     →|  ………………      |         
 ----------------------------------------------------------------------------------------------------------                                                                                               

     周期时长计算方法： WUT  = 4 × WTM[15:0] × （2的R次方）/32.768  (毫秒)
     接收时长计算方法： TLCD = 4 × LCD[7:0]  × （2的R次方）/32.768  (毫秒)
**/ 
/**
     唤醒包的数据速率：9600bps，   引导码门槛：4bits，接收时间：3.5毫秒。
     唤醒包的数据速率：115200bps， 引导码门槛：8bits，接收时间：2.0毫秒。
**/         
/**
    平均电流：(1.3 × 0.6 + 8.5 × 0.2 + （uiTLCD ÷ 10 - 0.6 - 0.2）× 18.5) ÷  (uiWUT ÷ 10) + 0.001   (mA)
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

    	SpiWriteRegister(WakeUpTimerPeriod1, ucR);	    	            /* R值=WTR[4:0]且最大值0x14     */ 							
    	SpiWriteRegister(WakeUpTimerPeriod2, uiM / 255);		        /* M值高8位，WTM[15:8]          */						
    	SpiWriteRegister(WakeUpTimerPeriod3, uiM % 255);		        /* M值低8位，WTM[7:0]           */						
    
    	SpiWriteRegister(LowDutyCycleModeDuration, ucLcd);              /* RX时长，LCD[7:0]             */ 
     } else {                                                                   

    	SpiWriteRegister(WakeUpTimerPeriod1, 0x00);		                /* R值=WTR[4:0]且最大值0x14     */ 							
    	SpiWriteRegister(WakeUpTimerPeriod2, 0x20);		                /* M值高8位，WTM[15:8]          */						
    	SpiWriteRegister(WakeUpTimerPeriod3, 0x00);		                /* M值低8位，WTM[7:0]           */						
    
    	SpiWriteRegister(LowDutyCycleModeDuration, 0x56);               /* RX时长，LCD[7:0]             */ 
    }
}
/*********************************************************************************************************
** Function name:       void RF_Init( void )
** Descriptions:        无线模块初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void RF_Init( void )
{	
    CLI();   //关中断
    Si433_Reset();                                                      /* 硬件复位，再软件复位         */ 
    EXTI_ClearITPendingBit(EXTI_Line0);
    SEI();	  //开中断
    
    Set_RF_Speed(467);							/* 设置传输载波频率  470 MHz    */	             
    
	RF_Channel=4;
    Si433_CHNL_STEP_Set(RF_Channel, 600);										 											 
	
    Tx_Pwr_set(4);							/* 设置发射功率                 */

    Data_Rate_set(DR115200BPS_DEV57_6KHZ);			        /* 设置数据速率及频偏           */	    

    PreambleLength_Set(40);                                             /* 设置引导码长度               */
    PreamblsheThreold_Set(20);                                          /* 设置引导码门槛               */
	
    SpiWriteRegister(HeaderControl2, 0x02);				/* 设置同步码长度               */  
    SpiWriteRegister(SyncWord3, 0x2D);					/* 设置同步码                   */  						 
    SpiWriteRegister(SyncWord2, 0xD4);					/* 设置同步码                   */  						 

    SpiWriteRegister(DataAccessControl, 0x8D);				/* 使能收发包CRC校验            */  
 /* 数据从FIFO中发送调制方式为GFSK	  */
    SpiWriteRegister(ModulationModeControl2, 0x63);			                            
 /* 设置发送FIFO中剩下0x0b个数据时，
   中断标志位(0x03,itxffeam)置位	  */
    //SpiWriteRegister(TXFIFOControl2, 0x0B);	                                   
                                  
    SpiWriteRegister(AGCOverride1, 0x60); 	                        /* 自动增益调整	                */	

    SpiWriteRegister(GPIO2Configuration, 0x14);                         /* 把GPIO2设置为GND             */	
    SpiWriteRegister(GPIO1Configuration, 0x12);                         /* 把GPIO1设置为RF开关控制端 12 */	
    SpiWriteRegister(GPIO0Configuration, 0x15);                         /* 把GPIO0设置为RF开关控制端 15 */

#ifdef   ZM470S_ABC  /* ZM470S-ABC用的晶体与ZM470S-D用的不一样	*/
	SpiWriteRegister(CrystalOscillatorLoadCapacitance, 0xB5);  	        /* ZM470S-ABC晶振补尝电容:10pF  */
#else
	SpiWriteRegister(CrystalOscillatorLoadCapacitance, 0x55);  	        /* ZM470S-D晶振补尝电容:9pf     */
#endif
	SpiWriteRegister(RXFIFOControl, 0x30);                              /* 接收数据长度门限	            */	
	//SpiWriteRegister(RSSIThresholdForClearChannelIndicator, 0x80);      /* RSSI接收信号强度	            */
 SpiWriteRegister(RSSIThresholdForClearChannelIndicator, 70);
#if HEADER_ENABLE == HEADER_STAT
    SourceMac_Set(4,&ucSourceMac[0]);                                   /* 设置本机地址	                */
    DestinMac_Set(4,&ucDestinMac[0]);									/* 设置目的地址	                */
    /**设置实际使用地址的长度，若设为地址长度设0则关闭地址功能
       开启或关闭广播地址功能，第二个入参为非0则接收广播地址包 **/
    Address_Lgth_Set(2,1); 												
#endif

    SpiReadRegister(InterruptStatus1);                                  /* 清除中断标志                 */
    SpiReadRegister(InterruptStatus2);
 
    //RF_LDC_Set(10000, 45);                                              /* 设置LDC周期1000ms,接收4.5ms  */

//	Rx_Mode_Entern();	                                                /*  进行接收状态                */
}
/*********************************************************************************************************
** Function name:       RFSendPacket
** Descriptions:        无线模块发送数据
** input parameters:    txBuffer-发送的数据指针  size-发送的数据长度
** Returned value:      发送的数据长度 
*********************************************************************************************************/


void RF_TUNE_MODE(void)
{
    SpiWriteRegister(OperatingFunctionControl1, 0x03);
}


unsigned char rf_send_failCnt=0;
unsigned char rf_send_failCnt1=0;
#define RF_SEND_Fail_RESET_MAX  10    //连续10次发送超时就复位
unsigned char status;
unsigned char IStatus1=0;
unsigned char IStatus2=0;

unsigned char RFSendPacket(unsigned char *txBuffer, unsigned char size)
{
        unsigned int   i = 0;
	char  unNum = size;
	unsigned char *p = txBuffer;
	
        CLI();	
        //关闭所有中断
        SpiWriteRegister(InterruptEnable1, 0x00);
        SpiWriteRegister(InterruptEnable2, 0x00);
        //清除之前存在的中断
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
	//IStatus1 = SpiReadRegister(InterruptStatus1);                              /* 清除中断完成标志             */
        //IStatus2 = SpiReadRegister(InterruptStatus2);
	SpiWriteRegister(TransmitPacketLength, unNum);
	
        for (i = 0; i < unNum; i++) 
	{                                       /* 填入要发送的数据             */                                   
          SpiWriteRegister(FIFOAccess, p[i]);
        }
        
        SpiWriteRegister(InterruptEnable1, 0x04);                           /* 开户发送中断                 */
        SpiWriteRegister(InterruptEnable2, 0x00);                                                 
                         
        IStatus1 = SpiReadRegister(InterruptStatus1);                              /* 清除中断完成标志             */
        IStatus2 = SpiReadRegister(InterruptStatus2);                                         
    
        SpiWriteRegister(OperatingFunctionControl1, 0x09);                  /* 进入发送状态                 */
        dellayx100us(8);
        
        
        status = SpiReadRegister(DeviceStatus);
        if((status&0x02)!=0x02)
        {
          //rf_send_failCnt1++;
          //if(rf_send_failCnt1>RF_SEND_Fail_RESET_MAX)
          {
            //等待复位
            //while(1);
          }
          
        }
        else
        {
          rf_send_failCnt1=0;
        }
        //频率错误
        if((status&0x08)==0x08)
        {
          //等待复位
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
            //等待复位
            //while(1);
            CLI();   //关中断
            RF_Init();
            //Errata();
            EXTI_ClearITPendingBit(EXTI_Line0);
            SEI();	  //开中断
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
	{                                                 /* 等待发送完成中断             */	
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
		/* 防止某些特殊情况拉低了中断引脚并且数据又发送失状态          */          
	    return 0; 
	}
	Rx_Mode_Entern();
	return (unNum);
}
/*********************************************************************************************************
** Function name:       RF_SendWakeUp_Packet
** Descriptions:        连续发送多个包唤醒处于LDC模式下的模块
** input parameters:     
** Returned value:       
*********************************************************************************************************/
void RF_SendWakeUp_Packet(unsigned int uiMs)
{
    unsigned int   j,i = 0;

    SpiWriteRegister(InterruptEnable1, 0x04);                           /* 开户发送中断                 */
    SpiWriteRegister(InterruptEnable2, 0x00);    
        
    i = SpiReadRegister(InterruptStatus2);                              /* 清除中断完成标志             */
    SpiWriteRegister(TransmitPacketLength, PACKET_NUM);
    for (i = 0; i < PACKET_NUM; i++) {                                  /* 填入要发送的数据             */                                      
         SpiWriteRegister(FIFOAccess, PACKET_DATA);
    }
    for(j = 0; j < uiMs; j++) {  
           	                                    
        SpiWriteRegister(OperatingFunctionControl1, 0x09);              /* 进入发送状态                 */
   
        SpiReadRegister(InterruptStatus1);                              /* 清除中断完成标志             */           
        dellay(1000);
        if(j < uiMs - 1){
            for (i = 0; i < PACKET_NUM; i++) {                          /* 填入要发送的数据             */                                      
                SpiWriteRegister(FIFOAccess, PACKET_DATA);
            } 
        }    
        while(IRQ_READ()) {                                             /* 等待发送完成中断             */	
        		dellay(100);
        		i++;
        		if(i > 65530) {
        		    break;
        		} 
       }
    }
        i = SpiReadRegister(InterruptStatus1);                          /* 清除中断完成标志             */
        i = SpiReadRegister(InterruptStatus2);  
}
/*********************************************************************************************************
** Function name:       RFSendPacket
** Descriptions:        无线模块接收数据 
** input parameters:    rxBuffer-接收的数据指针  
** output parameters:   
** Returned value:      接收的数据长度
*********************************************************************************************************/
unsigned char RFReacPacket(unsigned char *rxBuffer)
{
    unsigned char i = 0;
	unsigned char ucPck_Lgth = 0;
	unsigned char ucIrqstat1 = 0;
	unsigned char *ucData    = rxBuffer;

    ucIrqstat1 = SpiReadRegister(InterruptStatus1);                     /* 清除中断完成标志             */
                 SpiReadRegister(InterruptStatus2);

	if((ucIrqstat1 & 0x01) == 1) 
	{			                /* 如果是CRC接收标志置位，建议清空接收FIFO  */		    		
	   	RxFIFOReset();
	    return 0;
	} 
	else if(!(ucIrqstat1 & 0x02))
	{                                    /* 如果接收中断标志没置位       */   
  	RxFIFOReset();
		return 0;
	}
	           
	ucPck_Lgth = SpiReadRegister( ReceivedPacketLength );                /* 接收到的数据字节数在0x4B中   */    
        if(ucPck_Lgth>RF_rx_SIZE)return 0;       
	for (i = 0; i < ucPck_Lgth; i++) 
	{                                   
			ucData[i] = SpiReadRegister(FIFOAccess);  
	} 



	return ucPck_Lgth;
}
/*********************************************************************************************************
** Function name:       RF_STANDBY_MODE
** Descriptions:        进入待机状态
** input parameters:     
** Returned value:       
*********************************************************************************************************/
void RF_STANDBY_MODE(void)
{
    /* 进入待机状态，此时电充0.9uA,;用时800us可切换到发送或接收状态 */
    /* 如果需要更低功耗，可把模块的SDN引脚拉高，关闭模块的电源，但再次启动模块时需要重初即化模块 */
    SpiWriteRegister(OperatingFunctionControl2, 0x00);
    SpiWriteRegister(OperatingFunctionControl1, 0x0);                  /* 进入睡眠状态  enlbd = 1      */
     
}
/*********************************************************************************************************
** Function name:       RF_SLEEP_MODE
** Descriptions:        进入睡眠状态
** input parameters:     
** Returned value:       
*********************************************************************************************************/
void RF_SLEEP_MODE(void)
{
    /* 进入睡眠状态  enlbd = 1;此时电充1.4uA；用时800us可切换到发送或接收状态      */
    SpiWriteRegister(OperatingFunctionControl2, 0x00);
    SpiWriteRegister(OperatingFunctionControl1, 0x40);                  /* 进入睡眠状态  enlbd = 1      */
     
}
/*********************************************************************************************************
** Function name:       RF_READY_MODE
** Descriptions:        MCU控制无线模块唤醒休眠 
** input parameters:    
** Returned value:      
*********************************************************************************************************/
void RF_READY_MODE(void)
{
    /*  上电后的默认状态，xton = 1;此时电充1.5mA；用时200us可切换到发送或接收状态      */
    SpiWriteRegister(OperatingFunctionControl2, 0x00); 
    SpiWriteRegister(OperatingFunctionControl1, 0x01);                  /* 进入READY状态                */    
}
/*********************************************************************************************************
** Function name:       RF_LDC_MODE
** Descriptions:        设置可无线唤醒睡眠状态
** input parameters:    
** Returned value:      
*********************************************************************************************************/
void RF_LDC_MODE(void)
{
    /*  些模式的电流由函数void RF_LDC_Set(uint32_t uiWUT, uint32_t uiTLCD)的入参决定；
        如: RF_LDC_Set(10000, 20), 平均电流：29uA ;
            RF_LDC_Set(10000, 35), 平均电流：54uA ;
    */
 
    SpiWriteRegister(InterruptEnable1, 0x03);                           /* 开启接收到一帧有效数据中断   */   	                                                                    /* 以及开启接收CRC错误中断      */ 
    SpiWriteRegister(InterruptEnable2, 0x40);                           /* 开启接收到有效引导码中断     */ 
                                                                                                       
	SpiWriteRegister(OperatingFunctionControl1, 0x00);
	SpiWriteRegister(OperatingFunctionControl2, 0x04);                  /* 进入LDC状态                  */                     
                   
    SpiReadRegister(InterruptStatus1);										
	SpiReadRegister(InterruptStatus2);	                                /* 清除中断标志位               */ 
}
/*********************************************************************************************************
** Function name:       RxFIFOReset
** Descriptions:        复位接收缓冲区
** input parameters:    无
** Returned value:      无
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
** Descriptions:        复位发送缓冲区
** input parameters:    无
** Returned value:      无
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
** Descriptions:        本机地址设置
** input parameters:    ucByteNum: 地址的字节数。
                        ucpSource: 地址的首指针。 
** output parameters:   无
** Returned value:      实际写入的地址字节数。
*********************************************************************************************************/
unsigned char SourceMac_Set(unsigned char ucByteNum,unsigned char *ucpSource)      
{

   unsigned char i;
   unsigned char *ptemp;
   
   ptemp = ucpSource;

   if (ptemp == 0) {
       return 0;                                                        /* 指针为空                     */ 
   }

   for (i = 0; i < ucByteNum && i < 4; i++) {
   /* 校验地址为0x3F~0x42; CheckHeader3 = 0x3f */   
       SpiWriteRegister(CheckHeader3 + i, *ptemp++);	   
   }

   return i;
}
/*********************************************************************************************************
** Function name:       DestinMac_Set
** Descriptions:        目标地址设置
** input parameters:    ucByteNum: 地址的字节数。
                        ucpSource: 地址的首指针。 
** output parameters:   无
** Returned value:      实际写入的地址字节数。
*********************************************************************************************************/
unsigned char DestinMac_Set(unsigned char ucByteNum,unsigned char *ucpDestin)      
{

   unsigned char i;
   unsigned char *ptemp;
   
   ptemp = ucpDestin;

   if (ptemp == 0) {
       return 0;                                                        /* 指针为空                     */ 
   }

   for (i = 0; i < ucByteNum && i < 4; i++) {
   /* 校验地址为0x3A~0x3D; TransmitHeader3 = 0x3A */   
       SpiWriteRegister(TransmitHeader3 + i, *ptemp++);	        
   }

   return i;
}
/*********************************************************************************************************
** Function name:       Address_Lgth_Set
** Descriptions:        设置实际使用的地址长度
** input parameters:    ucByteNum: 启用的地址长度，以字节为单位。
						EN_BroadCast：为0时，不接收广播包；非0时接收广播包。                 
** output parameters:   无
** Returned value:      实际设置了的地址长度，以字节为单位。
*********************************************************************************************************/
unsigned char Address_Lgth_Set(unsigned char ucByteNum,unsigned char EN_BroadCast)      
{

    unsigned char hdlen = 0;		                        /* 发送和接收包头长度hdlen[2:0]，字节为单位 */ 
    unsigned char hdch  = 0;		                        /* 包头校验字节，对应位为1时使能，hdch[3:0] */ 
   	unsigned char bcen  = 0;								/* 接收广播包使能，为1时，对应的字节使能    */ 

    unsigned char header3  = 0;
    unsigned char header2  = 0;
    unsigned char header1  = 0;
    unsigned char header0  = 0;

	unsigned ucCtrl2Temp   = 0;
    unsigned uctemp        = ucByteNum;
	

    if (uctemp > 4) {						                            /* 入参超出范围则设为4字节      */ 
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

	if (0 != EN_BroadCast) {                                            /* 判断是否接收广播地址的包     */
		bcen = hdch;
	} else {
		bcen = 0;
	}

	hdlen = uctemp << 4;

	ucCtrl2Temp  = SpiReadRegister(HeaderControl2); 				    /* 先读出来，为了不改变其它位   */
	ucCtrl2Temp &= 0x8f;
	ucCtrl2Temp |= hdlen;
	SpiWriteRegister(HeaderControl2, ucCtrl2Temp);                      /* 设置包头长度                 */
	 
	SpiWriteRegister(HeaderControl1, bcen << 4 | hdch);	                /* 广播使能及校验使能字节长度   */

    /* 设置（为1时使能）校验使能字节对应的位 */
    SpiWriteRegister(HeaderEnable3, header3);	                        /* hden[31:24]                  */   
	SpiWriteRegister(HeaderEnable2, header2);	                        /* hden[23:16]                  */
    SpiWriteRegister(HeaderEnable1, header1);	                        /* hden[15:8]                   */
    SpiWriteRegister(HeaderEnable0, header0);	                        /* hden[7:0]                    */

    return uctemp;
}
/*********************************************************************************************************
** Function name:       Read_RecAddree
** Descriptions:        读取本机接收到的地址
** input parameters:    rxAddress：存放接收地址的指针首地址，最长只有四字节。 
** Returned value:       
*********************************************************************************************************/
void Read_RecAddree(unsigned char *rxAddress)
{
	*rxAddress = SpiReadRegister(ReceivedHeader3); 						/* 地址的首地字节               */
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
** Descriptions:        读取信号强度寄存器
** input parameters:    无
** Returned value:      实时信号强度的值 
*********************************************************************************************************/
unsigned char Read_Rssi(void)
{	
	return (SpiReadRegister(ReceivedSignalStrengthIndicator)); 
}
/*********************************************************************************************************
** Function name:       Rssi_Offset_Set
** Descriptions:        设置信号强度偏移
** input parameters:    ucOffset: 范围:0 ~ 7; 每增加一RSSI就会增加4dB,即RSSI寄存器读到的值会增加4*2=8。
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Rssi_Offset_Set(unsigned char ucOffset)      
{
   unsigned char preath;

   preath  = SpiReadRegister(PreambleDetectionControl);
   preath   &=	0xf8;
   ucOffset &=  0x07;

   SpiWriteRegister(PreambleDetectionControl, preath | ucOffset);		/* 设置信号强度偏移及引导码门槛 */  
}



/*********************************************************************************************************
** Function name:       Set_CCA_Threshold
** Descriptions:        设置清洁信道评估门限值
** input parameters:    
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Set_CCA_Threshold(signed char rssi_threshold)
{
  SpiWriteRegister(RSSIThresholdForClearChannelIndicator, rssi_threshold);
}


/*********************************************************************************************************
** Function name:       CCA
** Descriptions:        清洁信道评估
** input parameters:    
** output parameters:   无
** Returned value:      true:  信道清洁
**                      false: 信道被占用
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
//---------------------si4432初始化--------------------------
void Si4432_Init(void)
{
  //si4432 spi 初始化
  SI4432SpiInit();
  //si4432 IO 初始化
  Si4432_IO_Init();
  //si4432无线配置初始化
  CLI();   //关中断
  RF_Init();
  //Errata();
  EXTI_ClearITPendingBit(EXTI_Line0);
  SEI();	  //开中断
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

//si4432接收数据包
unsigned char Si4432_RfReceivePacket(unsigned char *ucRxbuf, unsigned char *RxNumber)
{
	unsigned char rssi;
	
	if(!IRQ_STATE())				/*  ?D??òy??±?à-μí￡?ê?μ?êy?Y    */ 				
	{		
		rssi = Read_Rssi(); 
	
		*RxNumber=RFReacPacket(ucRxbuf);					/*  ′óZM470S?￡?é?á3?êy?Y        */

		Rx_Mode_Entern();	                    		/*  ??DD?óê?×′ì?  */  
		
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
