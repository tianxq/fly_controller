/******************** (C) COPYRIGHT 2012 基于uCOS的STM32外设驱动库 ***************
 * 文件名  ：main.c
 * 描述    ：uCOS框架，用于测试外设驱动         
 * 实验平台：STM32F10X
 * 库版本  ：ST3.5.0
 * uCOS版本：Ver2.91
 * 作者    ：
**********************************************************************************/	
/**********************包含头文件*******************************/
#include "includes.h"

/********************** 宏定义 *******************************/

//定义uCOS任务堆栈大小
#define		Task_Start_Size		100
#define   Task_LedBeep_Stk_Size 50
#define   Task_ADC_Stk_Size     100
#define   Task_Wlan_Stk_Size    1000
#define   Task_USBHID_Stk_Size    100

//定义uCOS任务优先级
#define		Task_Start_Prio			15
#define   Task_LedBeep_Prio   12
#define   Task_ADC_Prio       2
#define   Task_Wlan_Prio      1
#define   Task_USBHID_Prio    3

//定义uCOS任务堆栈
OS_STK		Task_Start[Task_Start_Size];
OS_STK		Task_LedBeep_Stk[Task_LedBeep_Stk_Size];
OS_STK		Task_ADC_Stk[Task_ADC_Stk_Size];
OS_STK		Task_Wlan_Stk[Task_Wlan_Stk_Size];
OS_STK		Task_USBHID_Stk[Task_USBHID_Stk_Size];


//uCOS任务函数声明
void TaskStart(void *pdata);
void TaskLedBeep(void *pdata);
void TaskADC(void *pdata);
void TaskWlan(void *pdata);
void TaskUSBHID(void *pdata);

/////////////////////////////////////////////////////////////////////////
/**********************中断向量初始化*******************************/
static void NVIC_Configuration(void) 
{
#ifdef JKB_SW_L
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x4000);   //使用BOOTLOADER要加上这句   L_ADDR
#endif

#ifdef JKB_SW_H
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x20000);   //使用BOOTLOADER要加上这句	H_ADDR
#endif
	/* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);	
}

/**********************所有GPIO初始化为默认设置*******************************/
static void GPIO_DeIntConfiguration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /* Configure all unused GPIO port pins in Analog Input mode (floating input
    trigger OFF), this will reduce the power consumption and increase the device
    immunity against EMI/EMC *************************************************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_GPIOE, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_GPIOE, DISABLE);
}

/********************************* uCOS启动任务 *************************************/
void TaskStart(void *pdata)
{
	
    OSTaskCreateExt(TaskLedBeep,
                    (void *)0,
                    &Task_LedBeep_Stk[Task_LedBeep_Stk_Size - 1],
                    Task_LedBeep_Prio,
                    Task_LedBeep_Prio,
                    &Task_LedBeep_Stk[0],
                    Task_LedBeep_Stk_Size,
                    (void *)0,
                    OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
    OSTaskCreateExt(TaskADC,
                    (void *)0,
                    &Task_ADC_Stk[Task_ADC_Stk_Size - 1],
                    Task_ADC_Prio,
                    Task_ADC_Prio,
                    &Task_ADC_Stk[0],
                    Task_ADC_Stk_Size,
                    (void *)0,
                    OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	    OSTaskCreateExt(TaskWlan,
                    (void *)0,
                    &Task_Wlan_Stk[Task_Wlan_Stk_Size - 1],
                    Task_Wlan_Prio,
                    Task_Wlan_Prio,
                    &Task_Wlan_Stk[0],
                    Task_Wlan_Stk_Size,
                    (void *)0,
                    OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
			OSTaskCreateExt(TaskUSBHID,
                    (void *)0,
                    &Task_USBHID_Stk[Task_USBHID_Stk_Size - 1],
                    Task_USBHID_Prio,
                    Task_USBHID_Prio,
                    &Task_USBHID_Stk[0],
                    Task_USBHID_Stk_Size,
                    (void *)0,
                    OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	
	
	while(1)
	{
		OSTimeDly(OS_TICKS_PER_SEC/5);
	}
}

/********************************* LED初始化 *************************************/
#define LED_GPIO    GPIOB
#define BEEP_GPIO   GPIOB
#define LED_RCC     RCC_APB2Periph_GPIOB
#define LED_X       GPIO_Pin_10
#define LED_GPS     GPIO_Pin_11
#define LED_V       GPIO_Pin_12
#define BEEP        GPIO_Pin_13

#define LedOn(X)     GPIO_SetBits(LED_GPIO, X);
#define LedOff(X)    GPIO_ResetBits(LED_GPIO, X);

#define BeepOn(X)     GPIO_SetBits(BEEP_GPIO, X);
#define BeepOff(X)    GPIO_ResetBits(BEEP_GPIO, X);

void LedBeepInit()
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Enable the GPIO_LED Clock */
    RCC_APB2PeriphClockCmd(LED_RCC, ENABLE);

    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = LED_X|LED_GPS|LED_V;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/********************************* uCOS任务Led *************************************/

void TaskLedBeep(void *pdata)
{

	while(1)
	{
		LedOn(LED_X);
		LedOff(LED_X);
		BeepOn(BEEP);
		BeepOff(BEEP);
		//检测信号量 开关灯
		OSTimeDly(OS_TICKS_PER_SEC/2);
	}
}

/********************************* uCOS任务ADC *************************************/
void TaskADC(void *pdata)
{
	
	while(1)
	{

		OSTimeDly(OS_TICKS_PER_SEC/20);
	}
}

/******************************* uCOS任务Wlan ***********************************/
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

void TaskWlan(void *pdata)
{
	
	while(1)
	{
		OSTimeDly(OS_TICKS_PER_SEC/10);
	}
}

/********************************* uCOS任务	USBHID *************************************/
void TaskUSBHID(void *pdata)
{
	
	while(1)
	{
		//读取缓冲区HID指令
		//1,updata
		//2,japan,america,china
		//3,摇杆校准
		OSTimeDly(OS_TICKS_PER_SEC/5);
	}
}


/********************************* Main函数 *************************************/
int main(void)
{
	/* 配置系统时钟为72M */ 
	SystemInit();
	/******************************** 中断向量初始化 ****************************/	
	NVIC_Configuration();
	/****************************** GPIO恢复到默认配置 **************************/	
	GPIO_DeIntConfiguration();
  /******************************** uCOS-II初始化 *****************************/
  OSInit();
  /******************************** LED初始化 *********************************/
	LedBeepInit();
	//rs485_2_Init(9600,UART_CONFIG_PAR_NONE);
  /******************************** 创建启动任务 ******************************/
    OSTaskCreateExt(TaskStart,
                    (void *)0,
                    &Task_Start[Task_Start_Size - 1],
                    Task_Start_Prio,
                    Task_Start_Prio,
                    &Task_Start[0],
                    Task_Start_Size,
                    (void *)0,
                    OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
    /******************************** 使能系统节拍 ******************************/
    SysTick_Config(SystemCoreClock / 1000 *5);
    /******************************** 开始多任务 ******************************/
    OSStart();

	return 1;
}

/******************* (C) COPYRIGHT 2011 野火嵌入式开发工作室 *****END OF FILE****/
