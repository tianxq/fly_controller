/********************************************************************************
 * @file    Keyboard_Task.c
 * @author  TianXinqi
 * @version V1.0
 * @date    2015-10-21
 * @brief   This file mainly contain the routines that handle the led and beep
 *          display on the board.
 ******************************************************************************
 * @attention
 *
 * All Right reserved by JYU CO,.LTD
 *
 *******************************************************************************/
#include "includes.h"

/*********************************************************************************
 *                                  Declare
 ********************************************************************************/


/*********************************************************************************
 *                                  Variable
 ********************************************************************************/
u8 DisplayTimecount=0;

/*********************************************************************************
 *                                  Function
 ********************************************************************************/

void Display_OperateTimeout_Evt_CB(OS_TMR *ptmr, void *parg)
{
	if(DisplayTimecount==0)
	{
		LedOn(LED_CH_R);
		DisplayTimecount=1;
	}
	else
	{
		LedOff(LED_CH_R);
		DisplayTimecount=0;
	}

}



void Display_Task(void *pdata)
{
	OS_TMR * dTimer, *mTimer;
	u8 err_timer;
	
	ledInit();
	motoInit(14400,0);
	dTimer = OSTmrCreate(1, 
						OS_TMR_CFG_TICKS_PER_SEC/10, 
						OS_TMR_OPT_PERIODIC, 
						(OS_TMR_CALLBACK)Display_OperateTimeout_Evt_CB, 
						NULL, 
						NULL, 
						&err_timer);
	OSTmrStart (dTimer, &err_timer);
	
    while(1)
    {
			if (motostate == 1)
			{
				MotoOn();
				motostate=0;
			}
			else
				MotoOff();


			if (ledXstate)
			{
				LedOn(LED_X_G);
				LedOff(LED_X_R);
			}
			else
			{
				LedOn(LED_X_R);
				LedOff(LED_X_G);
			}
			if (ledGPSstate)
			{
				LedOn(LED_GPS_G);
				LedOff(LED_GPS_R);
			}
			else
			{
				LedOn(LED_GPS_R);
				LedOff(LED_GPS_G);
			}

			if (!GPIO_ReadInputDataBit(CHG_GPIO,CHG_ERR)
				&& GPIO_ReadInputDataBit(CHG_GPIO,CHG_STA))
			{
					LedOn(LED_CH_R);
					LedOff(LED_CH_G);
				  OSTmrStop(dTimer, OS_TMR_OPT_NONE,NULL,&err_timer);
			}
			else
			{
				if(ledCHstate==1)
				{
					LedOn(LED_CH_G);
					LedOff(LED_CH_R);
					OSTmrStop(dTimer, OS_TMR_OPT_NONE,NULL,&err_timer);
				}
				else
				{

					OSTmrStart (dTimer, &err_timer);
				}

			}

			OSTimeDlyHMSM(0,0,0,50);
    }
}
