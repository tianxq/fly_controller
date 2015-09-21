/******************** (C) COPYRIGHT 2012 ����uCOS��STM32���������� ***************
 * �ļ���  ��main.c
 * ����    ��uCOS��ܣ����ڲ�����������         
 * ʵ��ƽ̨��STM32F10X
 * ��汾  ��ST3.5.0
 * uCOS�汾��Ver2.91
 * ����    ��
**********************************************************************************/	
/**********************����ͷ�ļ�*******************************/
#include "includes.h"

/********************** �궨�� *******************************/

//����uCOS�����ջ��С
#define		Task_Start_Size		100
#define   Task_LedBeep_Stk_Size 50
#define   Task_ADC_Stk_Size     100
#define   Task_Key_Stk_Size    1000
#define   Task_USBHID_Stk_Size    100

//����uCOS�������ȼ�
#define		Task_Start_Prio			15
#define   Task_LedBeep_Prio   12
#define   Task_ADC_Prio       2
#define   Task_Key_Prio      1
#define   Task_USBHID_Prio    3

//sbus����
uint8_t  sbusData[17]={0x5a,0xa5};
uint16_t BatteryVoltage;//The battery voltage


//����uCOS�����ջ
OS_STK		Task_Start[Task_Start_Size];
OS_STK		Task_LedBeep_Stk[Task_LedBeep_Stk_Size];
OS_STK		Task_ADC_Stk[Task_ADC_Stk_Size];
OS_STK		Task_Key_Stk[Task_Key_Stk_Size];
OS_STK		Task_USBHID_Stk[Task_USBHID_Stk_Size];


//uCOS����������
void TaskStart(void *pdata);
void TaskLedBeep(void *pdata);
void TaskADC(void *pdata);
void TaskKey(void *pdata);
void TaskUSBHID(void *pdata);

/////////////////////////////////////////////////////////////////////////
/**********************�ж�������ʼ��*******************************/
static void NVIC_Configuration(void) 
{
#ifdef JKB_SW_L
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x4000);   //ʹ��BOOTLOADERҪ�������   L_ADDR
#endif

#ifdef JKB_SW_H
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x10000);   //ʹ��BOOTLOADERҪ�������	H_ADDR
#endif
	/* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);	
}

/**********************����GPIO��ʼ��ΪĬ������*******************************/
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

/********************************* uCOS�������� *************************************/
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


/********************************* uCOS����Led *************************************/
INT8U ledXstate=0,ledGPSstate=0,ledCHstate=0;//��״̬
void TaskLedBeep(void *pdata)
{

	INT32U testtime;
	while(1)
	{
		//�ж���ʾ��״̬
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

		//������
//		BeepOn();
//		OSTimeDly(OS_TICKS_PER_SEC/50);
//		BeepOff();
		
		//���
		MotoOn(MOTO);
		
		OSTimeDly(OS_TICKS_PER_SEC/2);
	}
}

/********************************* uCOS����ADC *************************************/
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

/******************************* uCOS����Key ***********************************/
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

/********************************* uCOS����	USBHID *************************************/
void TaskUSBHID(void *pdata)
{
	
	while(1)
	{
		//��ȡ������HIDָ��
		//1,updata
		//2,japan,america,china
		//3,ҡ��У׼
		OSTimeDly(OS_TICKS_PER_SEC);
	}
}


/********************************* Main���� *************************************/
int main(void)
{
#ifdef JKB_SW_H
	SCB->VTOR = FLASH_BASE | 0x10000; /* Vector Table Relocation in Internal FLASH. */
#endif
	/* ����ϵͳʱ��Ϊ72M */ 
//	SystemInit();
	/******************************** �ж�������ʼ�� ****************************/	
	NVIC_Configuration();//bootҪ�޸�flash��ַ
	/****************************** GPIO�ָ���Ĭ������ **************************/	
	GPIO_DeIntConfiguration();
  /******************************** uCOS-II��ʼ�� *****************************/
  OSInit();
  /******************************** Ӳ����ʼ�� *********************************/
	LedBeepInit();
	beepInit(0x3800,0);
	key_IO_Init();
	//sbus�����ʼ��
	rs485_1_Init(9600,USART_Parity_No);
	stm32_adc1_init();
		
  /******************************** ������������ ******************************/
    OSTaskCreateExt(TaskStart,
                    (void *)0,
                    &Task_Start[Task_Start_Size - 1],
                    Task_Start_Prio,
                    Task_Start_Prio,
                    &Task_Start[0],
                    Task_Start_Size,
                    (void *)0,
                    OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
    /******************************** ʹ��ϵͳ���� ******************************/
    SysTick_Config(SystemCoreClock / 1000);
    /******************************** ��ʼ������ ******************************/
    OSStart();

	return 1;
}

/******************* (C) COPYRIGHT 2011 Ұ��Ƕ��ʽ���������� *****END OF FILE****/
