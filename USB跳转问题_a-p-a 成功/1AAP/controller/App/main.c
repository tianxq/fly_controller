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
#define   Task_Key_Stk_Size    100
#define   Task_Uart_Stk_Size    100
#define   Task_USBHID_Stk_Size    500

//����uCOS�������ȼ�
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

//������ת
#define ApplicationAddress 0x08000000	//IAP������ʼ��ַ=0

typedef  void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;

//����uCOS�����ջ
OS_STK		Task_Start[Task_Start_Size];
OS_STK		Task_LedBeep_Stk[Task_LedBeep_Stk_Size];
OS_STK		Task_ADC_Stk[Task_ADC_Stk_Size];
OS_STK		Task_Key_Stk[Task_Key_Stk_Size];
OS_STK		Task_Uart_Stk[Task_Uart_Stk_Size];
OS_STK		Task_USBHID_Stk[Task_USBHID_Stk_Size];


//uCOS����������
void TaskStart(void *pdata);
void TaskLedBeep(void *pdata);
void TaskADC(void *pdata);
void TaskKey(void *pdata);
void TaskUart(void *pdata);
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

volatile void stm32_delay_ms(unsigned int x_ms)
{
	int i,j;

	for(i=0;i<x_ms;i++)
		for(j=0;j<7200;j++)
				__NOP;
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


/********************************* uCOS����Led *************************************/
void TaskLedBeep(void *pdata)
{	
	INT32U testtime;
	ledInit();
	motoInit(14400,0);
	key_Init();

	while(1)
	{
		//���
		if(motostate==1)
		{
			MotoOn();
			motostate=0;
		}
		else
			MotoOff();
		
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

/********************************* uCOS����ADC *************************************/
void TaskADC(void *pdata)
{
	uint16_t i,j;
	uint16_t ADResults[CHANNEL_NUM];
	uint16_t ADtoFilter[CHANNEL_NUM][AVERAGE_FILTER_BUFFER_SIZE];
	rc_pro_com_in_pkt_t sbusData;
	
	sbusData.head = 0x5AA5;
	
		//sbus�����ʼ��
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
		
		//2ͨ������
		ADResults[1]=0x800 - ADResults[1];
		//3ͨ������
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
		//ÿ10ms����һ��AD
		RS485Send(1,(uint8_t *)&sbusData,19);
//		RS485Send(1,(uint8_t *)&sbusData.key,2);
//		RS485Send(1,(uint8_t *)keyst+1,1);

		memset(keyst,1,6);
		
		BatteryVoltage=ADResults[6];
		
		if(BatteryVoltage< 0x8c0)//����3.6v
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

/******************************* uCOS����Key ***********************************/
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

/********************************* uCOS����	Uart����**********************************/
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

/********************************* uCOS����	USBHID *************************************/
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
		//��ȡ������HIDָ��
		//1,ҡ��У׼
		//2,japan,america,china
		//3,updata
		//4,�����ϴ�
				
		  if (HIDReceive_Buffer[0] == 0x01) //handStyle=0;
			{
				wFlashBuffer[0]=HIDReceive_Buffer[1];
				WriteFlashNBtye(4,wFlashBuffer,4);
				ReadFlashNBtye(4, rFlashBuffer, 4);
				if(rFlashBuffer[0]!=wFlashBuffer[0])
				{ 
					//����ʧ��
				}
			}
			else if (HIDReceive_Buffer[0] == 0x02)//calibration=0;ҡ��У׼=1
			{
				//У׼
			}
			else if (HIDReceive_Buffer[0] == 0x03)//����
			{
				wFlashBuffer[0]=HIDReceive_Buffer[1];
				WriteFlashNBtye(0,wFlashBuffer,4);
				ReadFlashNBtye(0, rFlashBuffer, 4);
				if(rFlashBuffer[0]!=wFlashBuffer[0])
				{ 
					//����λ��Ǵ���,IAP�޷�ʶ���־λ����
				}
				else
				{
					//�ص�IAP
					__set_FAULTMASK(1);      
					NVIC_SystemReset();// 
				}
			}
			else if (HIDReceive_Buffer[0] == 0x04)
			{
					//�ϴ�����
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

/********************************* Main���� *************************************/
int main(void)
{

		//SystemInit();//usb�жϱ���Ҫ�������
#ifdef JKB_SW_H
	SCB->VTOR = FLASH_BASE | 0x10000; /* Vector Table Relocation in Internal FLASH. */
#endif
	/* ����ϵͳʱ��Ϊ72M */ 

	/******************************** �ж�������ʼ�� ****************************/	
	NVIC_Configuration();//bootҪ�޸�flash��ַ

	/****************************** GPIO�ָ���Ĭ������ **************************/	
	//GPIO_DeIntConfiguration();
	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);
	GPIO_AFIODeInit();
	/******************************** uCOS-II��ʼ�� *****************************/
  OSInit();
  /******************************** Ӳ����ʼ�� *********************************/
	__set_PRIMASK(0);//
	//----------------------

	USB_IO_PullDown();
	stm32_delay_ms(1000);
	
	USB_IO_PullUp();
	stm32_delay_ms(100);
	//----------------------
	usbHIDInit();	
	

	
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
