/********************************************************************************
 * @file    Keyboard_Task.c
 * @author  TianXinqi
 * @version V1.0
 * @date    2015-10-21
 * @brief
 ******************************************************************************
 * @attention
 *
 * All Right reserved by JYU CO,.LTD
 *
 *******************************************************************************/
#include "includes.h"
#include "Keyboard_Task.h"

//F#include "SEGGER_RTT.h"
/*********************************************************************************
 *                                  Declare
 ********************************************************************************/
//Extern Variables



//Local functions
void Keyboard_ISR(void);
void Keyboard_OperateTimeout_Evt_CB(OS_TMR *ptmr, void *parg);
void Keyboard_Evt_Handler(uint16_t key);
void hal_key_board_init(void);
/*********************************************************************************
 *                                  Variable
 ********************************************************************************/
KeyStatus_t KeyState[6];
static OS_TMR *timekeyHome,*timekeyOkf,*timekeyPhoto,*timekeyA,*timekeyB;
static OS_EVENT * KeyBoard_MsgBox;
static uint16_t Key_Event;
static INT8U err_timer;
/*********************************************************************************
 *                                  Function
 ********************************************************************************/


/**
 * A uCOS Task to handle events occured on the keyboard.
 * This task create a message box, and pend message, if there is a rising edge
 * triggered a interrupt on one of the EXTI_Line related to the keyboard, the
 * ISR will post a message to the msgbox, and this task will accept and handle it.
 * @param pdata [uCOS input param]
 */
void Keyboard_Task(void *pdata)
{
    INT8U err;
    uint16_t * msg;

//	SEGGER_RTT_Init();
	
    hal_key_board_init();//Initialize the keyboard

    KeyBoard_MsgBox = OSMboxCreate(0);

    while(1)
    {
        msg = (uint16_t *)OSMboxPend(KeyBoard_MsgBox, 0, &err);

        if (err == OS_ERR_NONE && msg != NULL)
        {
            Keyboard_Evt_Handler(*msg);
			
			//SEGGER_RTT_printf(0, "check done.\r\n");
        }
    }
}



void hal_key_board_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd( KEY_RCC_CLK | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = KEY_OKF_PIN;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;      
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = KEY_A_PIN | KEY_NEWHAND | KEY_AE|KEY_VIO; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     
	GPIO_Init(GPIOB,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = KEY_HOME_PIN | KEY_PHOTO_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     //
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	GPIO_InitStructure.GPIO_Pin = KEY_B_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     
	GPIO_Init(GPIOB,&GPIO_InitStructure); 
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);          
        
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;     
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;        
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;       
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;    
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;       
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
	NVIC_Init(&NVIC_InitStructure);
	
	EXTI_ClearITPendingBit(EXTI_Line2);
	EXTI_ClearITPendingBit(EXTI_Line3);     
	EXTI_ClearITPendingBit(EXTI_Line13);
	EXTI_ClearITPendingBit(EXTI_Line14);
	EXTI_ClearITPendingBit(EXTI_Line15);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource2);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);

	EXTI_InitStructure.EXTI_Line = EXTI_Line2|EXTI_Line3|EXTI_Line13|EXTI_Line14|EXTI_Line15; 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;        
	EXTI_Init(&EXTI_InitStructure);	
	
	
	for( int i=0;i<(sizeof(KeyState)/sizeof(KeyState[0]));i++)
	{
		KeyState[i] = NORMAL;
	}
}
/**
 * Invoke this function in the EXTI_Line IRQ Handler function,
 * and this func will check what event occured on the keyboard,
 * and then post a msg to the KeyBoard_MsgQue, the Keyboard_Task
 * process will accept this msg and handle that.
 */
void Keyboard_ISR(void)
{
    if (EXTI_GetITStatus(EXTI_Line2) == SET) //keyA
    {
        Key_Event = KEY_A_NO;

        EXTI_ClearITPendingBit(EXTI_Line2);
    }
    else if (EXTI_GetITStatus(EXTI_Line3) == SET) //keyA
    {
        Key_Event = KEY_B_NO;

        EXTI_ClearITPendingBit(EXTI_Line3);
    }
    else if (EXTI_GetITStatus(EXTI_Line13) == SET)
    {
        Key_Event = KEY_HOME_NO;

        EXTI_ClearITPendingBit(EXTI_Line13);
    }
    else if (EXTI_GetITStatus(EXTI_Line14) == SET)
    {
        Key_Event = KEY_PHOTO_NO;

        EXTI_ClearITPendingBit(EXTI_Line14);
    }
    else if (EXTI_GetITStatus(EXTI_Line15) == SET)
    {
        Key_Event = KEY_OKF_NO;

        EXTI_ClearITPendingBit(EXTI_Line15);
    }

    OSMboxPost(KeyBoard_MsgBox, &Key_Event);
}


/**
 * The call back function of the key-timers, if the ucos timer called this function,
 * that means the operat on the keyboard is invalid, the status of the key should be
 * NoKeyDownStatus.
 * @param ptmr [the relative timer]
 * @param parg [the key]
 */
void Keyboard_OperateTimeout_Evt_CB(OS_TMR *ptmr, void *parg)
{
	uint16_t key = *(uint16_t *)parg;
	
    //Timering over, reset the key status
	KeyState[key] = NORMAL;

    OSTmrDel(ptmr, &err_timer);
		
//	SEGGER_RTT_printf(0,"Operate is invalid!\r\n", key);
}


void Keyboard_Evt_Handler(uint16_t key)
{
#define TIMER_OUT		(OS_TMR_CFG_TICKS_PER_SEC*5)
	
    INT32U time_left;
    OS_TMR ** pTimer;
    GPIO_TypeDef * port;
    uint16_t pin;
	  INT8U state;
	
	

    switch(key)
    {
        case KEY_A_NO:
            pTimer = &timekeyA;
            port = KEY_A_GPIO_PORT;
            pin = KEY_A_PIN;
            break;

        case KEY_B_NO:
            pTimer = &timekeyB;
            port = KEY_B_GPIO_PORT;
            pin = KEY_B_PIN;
            break;

        case KEY_HOME_NO:
            pTimer = &timekeyHome;
            port = KEY_HOME_GPIO_PORT;
            pin = KEY_HOME_PIN;
            break;

        case KEY_PHOTO_NO:
            pTimer = &timekeyPhoto;
            port = KEY_PHOTO_GPIO_PORT;
            pin = KEY_PHOTO_PIN;
            break;

        case KEY_OKF_NO:
            pTimer = &timekeyOkf;
            port = KEY_OKF_GPIO_PORT;
            pin = KEY_OKF_PIN;
            break;

        default:    break;
    }


	state = OSTmrStateGet(*pTimer, &err_timer);
	

	if (  (state == OS_TMR_STATE_UNUSED) 
       && (GPIO_ReadInputDataBit(port, pin) == Bit_RESET)
       )
	{ 	//The key was been just pushed
		//Start a timer to caculate th time of the key been hold
		(*pTimer) = OSTmrCreate(TIMER_OUT, 
								0, 
								OS_TMR_OPT_ONE_SHOT, 
								(OS_TMR_CALLBACK)Keyboard_OperateTimeout_Evt_CB, 
								(void *)&key, 
								NULL, 
								&err_timer);

		OSTmrStart (*pTimer, &err_timer);
		
	}
	else if ( state == OS_TMR_STATE_RUNNING /*&& GPIO_ReadInputDataBit(port, pin) == Bit_SET*/ )
	{ /* happen while the rising edge of the key input pin,
		 the key was released just now.
		 check the timer */
		time_left = OSTmrRemainGet(*pTimer, &err_timer);

		/* if the press keeping time is too short, that maybe a shake, ignore it */
		if (time_left > (TIMER_OUT - OS_TMR_CFG_TICKS_PER_SEC/2)) 
			KeyState[key] = NORMAL;
		
		if ( time_left < (TIMER_OUT - OS_TMR_CFG_TICKS_PER_SEC) )//50ms~
		{
			KeyState[key] = PRESSED_AND_HOLD;
			motostate = 1;//moto start;
//			SEGGER_RTT_printf(0,"Pressed and hold Key-%d\r\n", key);			
		}
		else
		{
		  KeyState[key] = CLICKED_ONCE;
			
//			SEGGER_RTT_printf(0,"Clicked Key-%d\r\n", key);
		}
		

		/* Key check routine end Stop the timer */
		OSTmrDel(*pTimer, &err_timer);
	}
}
