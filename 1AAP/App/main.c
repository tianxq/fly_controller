/********************************************************************************
 * @file    RC_Bus_Profile.c
 * @author  TianXinQi
 * @version V1.0
 * @date
 * @brief
 *******************************************************************************
 * @modify: Andy.Zhang
 * @date:	2015-10-21
 * *****************************************************************************
 * @attention
 *
 * All Right reserved by JYU CO,.LTD
 *
 *******************************************************************************/


/**********************����ͷ�ļ�*******************************/
#include "includes.h"

/*********************************************************************************
 *									Declare
 ********************************************************************************/
 extern uint16_t ADResults[CHANNEL_NUM];
 extern uint16_t BatteryVoltage;//The battery voltage


void TaskUSBHID(void *pdata);
void IWDG_Init(void);
extern void TaskADC(void *pdata);
extern void Keyboard_Task(void *pdata);
extern void Display_Task(void *pdata);
extern void RC_Bus_Task(void *pdata);
extern void TaskUSBHID(void *pdata);

typedef  void (*pFunction)(void);

#define HAL_RESET_SYSTEM()			do{IWDG_Init();while(1);}while(0)

#define ApplicationAddress 0x08000000	//IAP
/*********************************************************************************
 *									Variable
 ********************************************************************************/
// uCOS task stacks
OS_STK		Task_Start_Stk[Task_Start_Size];
OS_STK		Task_Display_Stk[Task_Display_Stk_Size];
OS_STK		Task_ADC_Stk[Task_ADC_Stk_Size];
OS_STK		Task_Key_Stk[Task_Key_Stk_Size];
OS_STK		Task_RCBus_Stk[Task_RCBus_Stk_Size];
OS_STK		Task_USBHID_Stk[Task_USBHID_Stk_Size];

pFunction Jump_To_Application;
uint32_t JumpAddress;




/*********************************************************************************
 *									Function
 ********************************************************************************/
volatile void stm32_delay_ms(unsigned int x_ms)
{
	int i,j;

	for(i=0;i<x_ms;i++)
		for(j=0;j<7200;j++)
				__NOP;
}


void stm32_crc_init(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
}


/********************************* uCOS�������� *************************************/
void TaskStart(void *pdata)
{
    stm32_crc_init();

    OSTaskCreateExt(Display_Task,
                    (void *)0,
                    &Task_Display_Stk[Task_Display_Stk_Size - 1],
                    Task_Display_Prio,
                    Task_Display_Prio,
                    &Task_Display_Stk[0],
                    Task_Display_Stk_Size,
                    (void *)0,
                    OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

    OSTaskCreateExt(TaskADC,
                    (void *)0,
                    &Task_ADC_Stk[Task_ADC_Stk_Size - 1],
                    Task_ADC_Prio,
                    Task_ADC_Prio,
                    Task_ADC_Stk,
                    Task_ADC_Stk_Size,
                    (void *)0,
                    OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

	OSTaskCreateExt(Keyboard_Task,
					(void *)0,
					&Task_Key_Stk[Task_Key_Stk_Size - 1],
					Task_Key_Prio,
					Task_Key_Prio,
					&Task_Key_Stk[0],
					Task_Key_Stk_Size,
					(void *)0,
					OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);


	OSTaskCreateExt(RC_Bus_Task,
					(void *)0,
					&Task_RCBus_Stk[Task_RCBus_Stk_Size - 1],
					Task_RCBus_Prio,
					Task_RCBus_Prio,
					&Task_RCBus_Stk[0],
					Task_RCBus_Stk_Size,
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


	OSTaskDel(OS_PRIO_SELF);
}




void IWDG_Init(void)
{

	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	IWDG_SetPrescaler(IWDG_Prescaler_256);

	IWDG_SetReload(10);

  IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);

	IWDG_ReloadCounter();

	IWDG_Enable();
}


/********************************* uCOS����	USBHID *************************************/



/********************************* Main���� *************************************/
int main(void)
{
#ifdef JKB_SW_H
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x10000);
#endif

	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);
	GPIO_AFIODeInit();

	/******************************** �ж�������ʼ�� ****************************/

	/****************************** GPIO�ָ���Ĭ������ **************************/

	/******************************** uCOS-II��ʼ�� *****************************/
  __set_PRIMASK(0);	//enable interrupts
	
	USB_IO_PullDown();
	stm32_delay_ms(200);
	USB_IO_PullUp();
	stm32_delay_ms(10);
	usbHIDInit();	
	stm32_delay_ms(500);
	

	OSInit();
  	/******************************** Create uCOS Task ******************************/
	OSTaskCreateExt(TaskStart,
					(void *)0,
					&Task_Start_Stk[Task_Start_Size - 1],
					Task_Start_Prio,
					Task_Start_Prio,
					Task_Start_Stk,
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
