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


/*********************************************************************************
 *                                  Function
 ********************************************************************************/

void TaskUSBHID(void *pdata)
{
	//uint8_t InBuffer[8]={1,2,3,4,5,6,7,8};
	uint8_t wFlashBuffer[4]={0},rFlashBuffer[4];

//	USB_IO_PullDown();
//	OSTimeDly(OS_TICKS_PER_SEC);

//	USB_IO_PullUp();
//	OSTimeDly(OS_TICKS_PER_SEC);

//	usbHIDInit();


	while(1)
	{
		//1,校准
		//2,japan,america,china
		//3,updata
		//4,数据上传
		//5,通道反相
		  if (HIDReceive_Buffer[0] == 0x01) //calibration
			{
				/* Create a new task to do the adc calibration */
        OSTaskCreateExt(RC_Channel_Calibrate_Process,
    					(void *)0,
    					&Task_Start_Stk[Task_Start_Size - 1],
    					Task_Start_Prio,
    					Task_Start_Prio,
    					Task_Start_Stk,
    					Task_Start_Size,
    					(void *)0,
    					OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
			}
			else if (HIDReceive_Buffer[0] == 0x02)//handStyle
			{
				if(writeMask(HIDmask_address_amjp,HIDReceive_Buffer+1,1) ==0)
				{ 
					
				}

			}
			else if (HIDReceive_Buffer[0] == 0x03)//updata
			{
				if(writeMask(HIDmask_address_boot,HIDReceive_Buffer+1,1) ==0)
				{ 
					//
				}
				else
				{
					__set_FAULTMASK(1);      
					NVIC_SystemReset();// 
				}
			}
			else if (HIDReceive_Buffer[0] == 0x04)
			{
					//???????
					/*********
					UserToPMABufferCopy(InBuffer, GetEPTxAddr(ENDP1), 6);
					SetEPTxCount(ENDP1, 6);
					SetEPTxValid(ENDP1);
					*********/
			}
			else if (HIDReceive_Buffer[0] == 0x05)
			{
				//adc反向
				if(writeMask(HIDmask_address_adcinversion,HIDReceive_Buffer+1,6) ==0)
				{ 
					
				}

			}

			HIDReceive_Buffer[0]=0;

		OSTimeDly(OS_TICKS_PER_SEC/10);
	}
}
