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
#define   Task_Key_Stk_Size    1000
#define   Task_USBHID_Stk_Size    100

//定义uCOS任务优先级
#define		Task_Start_Prio			15
#define   Task_LedBeep_Prio   12
#define   Task_ADC_Prio       2
#define   Task_Key_Prio      1
#define   Task_USBHID_Prio    3

//sbus数据
uint8_t  sbusData[17]={0x5a,0xa5};
uint16_t BatteryVoltage;//The battery voltage


//定义uCOS任务堆栈
OS_STK		Task_Start[Task_Start_Size];
OS_STK		Task_LedBeep_Stk[Task_LedBeep_Stk_Size];
OS_STK		Task_ADC_Stk[Task_ADC_Stk_Size];
OS_STK		Task_Key_Stk[Task_Key_Stk_Size];
OS_STK		Task_USBHID_Stk[Task_USBHID_Stk_Size];


//uCOS任务函数声明
void TaskStart(void *pdata);
void TaskLedBeep(void *pdata);
void TaskADC(void *pdata);
void TaskKey(void *pdata);
void TaskUSBHID(void *pdata);

/////////////////////////////////////////////////////////////////////////
/**********************中断向量初始化*******************************/
static void NVIC_Configuration(void) 
{
#ifdef JKB_SW_L
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x4000);   //使用BOOTLOADER要加上这句   L_ADDR
#endif

#ifdef JKB_SW_H
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x10000);   //使用BOOTLOADER要加上这句	H_ADDR
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
	    OSTaskCreateExt(TaskKey,
                    (void *)0,
                    &Task_Key_Stk[Task_Key_Stk_Size - 1],
                    Task_Key_Prio,
                    Task_Key_Prio,
                    &Task_Key_Stk[0],
                    Task_Key_Stk_Size,
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
		OSTimeDly(OS_TICKS_PER_SEC*5);
	}
}


/********************************* uCOS任务Led *************************************/
INT8U ledXstate=0,ledGPSstate=0,ledCHstate=0;//灯状态
void TaskLedBeep(void *pdata)
{

	INT32U testtime;
	while(1)
	{
		//判断显示灯状态
		if(ledXstate)
		{
			LedOn(LED_X_G);
			LedOff(LED_X_R);
		}
		else
		{
			LedOn(LED_X_R);
			LedOff(LED_X_G);
		}
		if(ledGPSstate)
		{
			LedOn(LED_GPS_G);
			LedOff(LED_GPS_R);
		}
		else
		{
			LedOn(LED_GPS_R);
			LedOff(LED_GPS_G);
		}
		
		if(!GPIO_ReadInputDataBit(CHG_GPIO,CHG_ERR) 
			&& GPIO_ReadInputDataBit(CHG_GPIO,CHG_STA))
		{
			LedOn(LED_CH_R);
			LedOff(LED_CH_G);
		}
		else
		{
			LedOn(LED_CH_G);
			LedOff(LED_CH_R);
		}

		//蜂鸣器
//		BeepOn();
//		OSTimeDly(OS_TICKS_PER_SEC/50);
//		BeepOff();
		
		//马达
		MotoOn(MOTO);
		
		OSTimeDly(OS_TICKS_PER_SEC/2);
	}
}

/********************************* uCOS任务ADC *************************************/
void TaskADC(void *pdata)
{
	uint16_t i,j;
	uint16_t ADResults[CHANNEL_NUM];
	uint16_t ADtoFilter[CHANNEL_NUM][AVERAGE_FILTER_BUFFER_SIZE];
	
	while(1)
	{

		for(i=0;i<CHANNEL_NUM;i++)
		{
			for(j=0;j<AVERAGE_FILTER_BUFFER_SIZE;j++)
			{
				ADtoFilter[i][j]=AD_Value[j][i];
			}
		}
		for(i=0;i<CHANNEL_NUM;i++)
		{
			ADResults[i]=average_filter(&ADtoFilter[i][0]);
		}
				
		for(i=0;i<CHANNEL_NUM;i++)
		{
			ADResults[i]=ADResults[i]>>1;
		}
		sbusData[2]=1;
		sbusData[3]=12;
		memcpy(sbusData+4,ADResults,12);
		
		sbusData[16]=checksum8(sbusData,16);
		//RS485Send(1,sbusData,17);
		
		BatteryVoltage=ADResults[6];
		
		if(BatteryVoltage< 0x700)
		{
			ledCHstate=0;
	  }
		else
		{
			ledCHstate=1;
		}
		
		OSTimeDly(OS_TICKS_PER_SEC/100);
	}
}

/******************************* uCOS任务Key ***********************************/
int keythreeSt1=0,keythreeSt2=0;

void TaskKey(void *pdata)
{
	KeyStatus st;
		
	while(1)
	{
		st=ReadKeyStatus(KEY_A_GPIO,KEY_A,&keyAst);
		if(st==ContiousKeyDownStatus)
		{
			sbusData[2]=7;
			sbusData[3]=3;
			sbusData[4]=checksum8(sbusData,4);
			RS485Send(1,sbusData,5);
		}
		else if(st==OnceKeyDownStatus)
		{
			sbusData[2]=7;
			sbusData[3]=2;
			sbusData[4]=checksum8(sbusData,4);
			RS485Send(1,sbusData,5);
		}
		st=ReadKeyStatus(KEY_B_GPIO,KEY_B,&keyBst);
		if(st==OnceKeyDownStatus)
		{
			sbusData[2]=8;
			sbusData[3]=2;
			sbusData[4]=checksum8(sbusData,4);
			RS485Send(1,sbusData,5);
		}
		else if(st==ContiousKeyDownStatus)
		{
			sbusData[2]=8;
			sbusData[3]=3;
			sbusData[4]=checksum8(sbusData,4);
			RS485Send(1,sbusData,5);
		}
		st=ReadKeyStatus(KEY_OKF_GPIO,KEY_OKF,&keyOKFst);
		if(st==OnceKeyDownStatus)
		{
			sbusData[2]=4;
			sbusData[3]=2;
			sbusData[4]=checksum8(sbusData,4);
			RS485Send(1,sbusData,5);
		}
		else if(st==ContiousKeyDownStatus)
		{
			sbusData[2]=4;
			sbusData[3]=3;
			sbusData[4]=checksum8(sbusData,4);
			RS485Send(1,sbusData,5);
		}
		st=ReadKeyStatus(KEY_HOME_GPIO,KEY_HOME,&keyHOMEst);
		if(st==OnceKeyDownStatus)
		{
			sbusData[2]=3;
			sbusData[3]=2;
			sbusData[4]=checksum8(sbusData,4);
			RS485Send(1,sbusData,5);
		}
		else if(st==ContiousKeyDownStatus)
		{
			sbusData[2]=3;
			sbusData[3]=3;
			sbusData[4]=checksum8(sbusData,4);
			RS485Send(1,sbusData,5);
		}
		st=ReadKeyStatus(KEY_PHOTO_GPIO,KEY_PHOTO,&keyPHOTOst);
		if(st==OnceKeyDownStatus)
		{
			sbusData[2]=5;
			sbusData[3]=2;
			sbusData[4]=checksum8(sbusData,4);
			RS485Send(1,sbusData,5);
		}
		else if(st==ContiousKeyDownStatus)
		{
			sbusData[2]=5;
			sbusData[3]=3;
			sbusData[4]=checksum8(sbusData,4);
			RS485Send(1,sbusData,5);
		}
		st=ReadKeyStatus(KEY_VIDEO_GPIO,KEY_VIDEO,&keyVIDEOst);
		if(st==OnceKeyDownStatus)
		{
			sbusData[2]=6;
			sbusData[3]=2;
			sbusData[4]=checksum8(sbusData,4);
			RS485Send(1,sbusData,5);
		}
		else if(st==ContiousKeyDownStatus)
		{
			sbusData[2]=6;
			sbusData[3]=3;
			sbusData[4]=checksum8(sbusData,4);
			RS485Send(1,sbusData,5);
		}
		
		if(GPIO_ReadInputDataBit(KEY_THREE_GPIO,KEY_NEWHAND)==0)
		{
			keythreeSt2=1;
		}
		else if(GPIO_ReadInputDataBit(KEY_THREE_GPIO,KEY_AE)==0)
		{
			keythreeSt2=2;
		}
		else if(GPIO_ReadInputDataBit(KEY_THREE_GPIO,KEY_VIO)==0)
		{
			keythreeSt2=3;
		}
		if(keythreeSt2!= keythreeSt1)
		{
			sbusData[2]=2;
			sbusData[3]=keythreeSt2;
			sbusData[4]=checksum8(sbusData,4);
			RS485Send(1,sbusData,5);
			keythreeSt1 = keythreeSt2;
		}
		OSTimeDly(OS_TICKS_PER_SEC/200);
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
		OSTimeDly(OS_TICKS_PER_SEC);
	}
}


/********************************* Main函数 *************************************/
int main(void)
{
#ifdef JKB_SW_H
	SCB->VTOR = FLASH_BASE | 0x10000; /* Vector Table Relocation in Internal FLASH. */
#endif
	/* 配置系统时钟为72M */ 
//	SystemInit();
	/******************************** 中断向量初始化 ****************************/	
	NVIC_Configuration();//boot要修改flash地址
	/****************************** GPIO恢复到默认配置 **************************/	
	GPIO_DeIntConfiguration();
  /******************************** uCOS-II初始化 *****************************/
  OSInit();
  /******************************** 硬件初始化 *********************************/
	LedBeepInit();
	beepInit(0x3800,0);
	key_IO_Init();
	//sbus传输初始化
	rs485_1_Init(9600,USART_Parity_No);
	stm32_adc1_init();
		
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
    SysTick_Config(SystemCoreClock / 1000);
    /******************************** 开始多任务 ******************************/
    OSStart();

	return 1;
}

/******************* (C) COPYRIGHT 2011 野火嵌入式开发工作室 *****END OF FILE****/
