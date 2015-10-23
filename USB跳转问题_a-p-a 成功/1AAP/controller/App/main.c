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
#define   Task_Key_Stk_Size    100
#define   Task_Uart_Stk_Size    100
#define   Task_USBHID_Stk_Size    500

//定义uCOS任务优先级
#define		Task_Start_Prio			15
#define   Task_LedBeep_Prio   12
#define   Task_ADC_Prio       9
#define   Task_Key_Prio      8
#define   Task_Uart_Prio      11
#define   Task_USBHID_Prio    7


uint16_t BatteryVoltage;//The battery voltage
KeyStatus keyst[6];

		
INT8U err_timer;
OS_TMR *timekeyHome,*timekeyOkf,*timekeyPhoto,*timekeyA,*timekeyB;

//程序跳转
#define ApplicationAddress 0x08000000	//IAP程序起始地址=0

typedef  void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;

//定义uCOS任务堆栈
OS_STK		Task_Start[Task_Start_Size];
OS_STK		Task_LedBeep_Stk[Task_LedBeep_Stk_Size];
OS_STK		Task_ADC_Stk[Task_ADC_Stk_Size];
OS_STK		Task_Key_Stk[Task_Key_Stk_Size];
OS_STK		Task_Uart_Stk[Task_Uart_Stk_Size];
OS_STK		Task_USBHID_Stk[Task_USBHID_Stk_Size];


//uCOS任务函数声明
void TaskStart(void *pdata);
void TaskLedBeep(void *pdata);
void TaskADC(void *pdata);
void TaskKey(void *pdata);
void TaskUart(void *pdata);
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

volatile void stm32_delay_ms(unsigned int x_ms)
{
	int i,j;

	for(i=0;i<x_ms;i++)
		for(j=0;j<7200;j++)
				__NOP;
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
//	    OSTaskCreateExt(TaskKey,
//                    (void *)0,
//                    &Task_Key_Stk[Task_Key_Stk_Size - 1],
//                    Task_Key_Prio,
//                    Task_Key_Prio,
//                    &Task_Key_Stk[0],
//                    Task_Key_Stk_Size,
//                    (void *)0,
//                    OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
			OSTaskCreateExt(TaskUart,
                    (void *)0,
                    &Task_Uart_Stk[Task_Uart_Stk_Size - 1],
                    Task_Uart_Prio,
                    Task_Uart_Prio,
                    &Task_Uart_Stk[0],
                    Task_Uart_Stk_Size,
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
		OSTimeDly(OS_TICKS_PER_SEC);
	}
}


/********************************* uCOS任务Led *************************************/
void TaskLedBeep(void *pdata)
{	
	INT32U testtime;
	ledInit();
	motoInit(14400,0);
	key_Init();

	while(1)
	{
		//马达
		if(motostate==1)
		{
			MotoOn();
			motostate=0;
		}
		else
			MotoOff();
		
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
			if(ledCHstate==1)
			{
				LedOn(LED_CH_G);
				LedOff(LED_CH_R);
			}
			else 
			{
				LedOn(LED_CH_R);
				LedOff(LED_CH_G);
				OSTimeDly(OS_TICKS_PER_SEC/20);
				LedOff(LED_CH_R);
			}

		}
		
		OSTimeDly(OS_TICKS_PER_SEC/20);
	}
}

/********************************* uCOS任务ADC *************************************/
void TaskADC(void *pdata)
{
	uint16_t i,j;
	uint16_t ADResults[CHANNEL_NUM];
	uint16_t ADtoFilter[CHANNEL_NUM][AVERAGE_FILTER_BUFFER_SIZE];
	rc_pro_com_in_pkt_t sbusData;
	
	sbusData.head = 0x5AA5;
	
		//sbus传输初始化
	rs485_1_Init(115200,UART_CONFIG_PAR_EVEN);
	stm32_adc1_init();
	
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
				
		for(i=0;i<CHANNEL_NUM-1;i++)
		{
			ADResults[i]=ADResults[i]>>1;
		}
		
		//2通道反向
		ADResults[1]=0x800 - ADResults[1];
		//3通道反向
		ADResults[2]=0x800 - ADResults[2];
		
		sbusData.type=1;
		sbusData.length=14;
		memcpy(sbusData.adc,ADResults,12);
		
		/*key*/
		if(GPIO_ReadInputDataBit(KEY_THREE_GPIO,KEY_NEWHAND)==0)
		{
			keyst[0]=1;
		}
		else if(GPIO_ReadInputDataBit(KEY_THREE_GPIO,KEY_AE)==0)
		{
			keyst[0]=2;
		}
		else if(GPIO_ReadInputDataBit(KEY_THREE_GPIO,KEY_VIO)==0)
		{
			keyst[0]=3;
		}
		sbusData.key[0]=0;
		sbusData.key[1]=0;
		sbusData.key[0] |= keyst[0]<<6;
		sbusData.key[0] |= keyst[1]<<4;
		sbusData.key[0] |= keyst[2]<<2;
		sbusData.key[0] |= keyst[3];
		sbusData.key[1] |= keyst[4]<<6;
		sbusData.key[1] |= keyst[5]<<4;
		/**/
		
		sbusData.chk=checksum8((uint8_t *)&sbusData,18);
		//每10ms发送一次AD
		RS485Send(1,(uint8_t *)&sbusData,19);
//		RS485Send(1,(uint8_t *)&sbusData.key,2);
//		RS485Send(1,(uint8_t *)keyst+1,1);

		memset(keyst,1,6);
		
		BatteryVoltage=ADResults[6];
		
		if(BatteryVoltage< 0x8c0)//低于3.6v
		{
			ledCHstate=0;
	  }
		else
		{
			ledCHstate=1;
		}
		
		OSTimeDly(OS_TICKS_PER_SEC/200);
	}
}

/******************************* uCOS任务Key ***********************************/
void TaskKey(void *pdata)
{
		
	while(1)
	{
		keyst[4]=ReadKeyStatus(KEY_A_GPIO,KEY_A,&keyAst);

		keyst[5]=ReadKeyStatus(KEY_B_GPIO,KEY_B,&keyBst);

		keyst[2]=ReadKeyStatus(KEY_OKF_GPIO,KEY_OKF,&keyOKFst);

		keyst[1]=ReadKeyStatus(KEY_HOME_GPIO,KEY_HOME,&keyHOMEst);

		keyst[3]=ReadKeyStatus(KEY_PHOTO_GPIO,KEY_PHOTO,&keyPHOTOst);

		//keyst[3]=ReadKeyStatus(KEY_VIDEO_GPIO,KEY_VIDEO,&keyVIDEOst);
		
		if(GPIO_ReadInputDataBit(KEY_THREE_GPIO,KEY_NEWHAND)==0)
		{
			keyst[0]=1;
		}
		else if(GPIO_ReadInputDataBit(KEY_THREE_GPIO,KEY_AE)==0)
		{
			keyst[0]=2;
		}
		else if(GPIO_ReadInputDataBit(KEY_THREE_GPIO,KEY_VIO)==0)
		{
			keyst[0]=3;
		}
		
		OSTimeDly(OS_TICKS_PER_SEC/20);
	}
}

/********************************* uCOS任务	Uart接收**********************************/
void TaskUart(void *pdata)
{
	uint8_t test[16],len;
	while(1)
	{
//		if(nCom1.nBuff.UsartRxBuff[0]==0x11)
//		{
//				ledXstate=1;
//				nCom1.nBuff.UsartRxBuff[0]=0;
//		}
//		else if(nCom1.nBuff.UsartRxBuff[0]==0x10)
//		{
//				ledXstate=0;
//				nCom1.nBuff.UsartRxBuff[0]=0;
//		}
		
		OSTimeDly(OS_TICKS_PER_SEC/20);
	}
}

/********************************* uCOS任务	USBHID *************************************/
void TaskUSBHID(void *pdata)
{
	uint8_t InBuffer[8]={1,2,3,4,5,6,7,8};
	uint8_t wFlashBuffer[4]={0},rFlashBuffer[4];

//	USB_IO_PullDown();
//stm32_delay_ms(100);
////	
//	USB_IO_PullUp();
//stm32_delay_ms(10);
//	//----------------------
//	usbHIDInit();	
	while(1)
	{
		//读取缓冲区HID指令
		//1,摇杆校准
		//2,japan,america,china
		//3,updata
		//4,数据上传
				
		  if (HIDReceive_Buffer[0] == 0x01) //handStyle=0;
			{
				wFlashBuffer[0]=HIDReceive_Buffer[1];
				WriteFlashNBtye(4,wFlashBuffer,4);
				ReadFlashNBtye(4, rFlashBuffer, 4);
				if(rFlashBuffer[0]!=wFlashBuffer[0])
				{ 
					//设置失败
				}
			}
			else if (HIDReceive_Buffer[0] == 0x02)//calibration=0;摇杆校准=1
			{
				//校准
			}
			else if (HIDReceive_Buffer[0] == 0x03)//升级
			{
				wFlashBuffer[0]=HIDReceive_Buffer[1];
				WriteFlashNBtye(0,wFlashBuffer,4);
				ReadFlashNBtye(0, rFlashBuffer, 4);
				if(rFlashBuffer[0]!=wFlashBuffer[0])
				{ 
					//升级位标记错误,IAP无法识别标志位升级
				}
				else
				{
					//回到IAP
					__set_FAULTMASK(1);      
					NVIC_SystemReset();// 
				}
			}
			else if (HIDReceive_Buffer[0] == 0x04)
			{
					//上传数据
					/*********
					UserToPMABufferCopy(InBuffer, GetEPTxAddr(ENDP1), 6);
					SetEPTxCount(ENDP1, 6);                    
					SetEPTxValid(ENDP1);
					*********/
			}
			
			HIDReceive_Buffer[0]=0;

		OSTimeDly(OS_TICKS_PER_SEC/10);
	}
}

/********************************* Main函数 *************************************/
int main(void)
{

		//SystemInit();//usb中断必须要？？这个
#ifdef JKB_SW_H
	SCB->VTOR = FLASH_BASE | 0x10000; /* Vector Table Relocation in Internal FLASH. */
#endif
	/* 配置系统时钟为72M */ 

	/******************************** 中断向量初始化 ****************************/	
	NVIC_Configuration();//boot要修改flash地址

	/****************************** GPIO恢复到默认配置 **************************/	
	//GPIO_DeIntConfiguration();
	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);
	GPIO_AFIODeInit();
	/******************************** uCOS-II初始化 *****************************/
  OSInit();
  /******************************** 硬件初始化 *********************************/
	__set_PRIMASK(0);//
	//----------------------

	USB_IO_PullDown();
	stm32_delay_ms(1000);
	
	USB_IO_PullUp();
	stm32_delay_ms(100);
	//----------------------
	usbHIDInit();	
	

	
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
