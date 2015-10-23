/**
  ******************************************************************************
  * @file    Project/Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.0.0
  * @date    04/06/2009
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "includes.h"
#include "stm32f10x_it.h"
#include "usb_istr.h"
//#include "LogicTimer.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
	//ledInit();
	while(1)
    {
      //ledToggle(1);    
    }
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval : None
  */
void SysTick_Handler(void)
{
    OS_CPU_SR  cpu_sr;

    OS_ENTER_CRITICAL();
    OSIntNesting++;
    OS_EXIT_CRITICAL();
	OSTimeTick();
	//LogicTimerThread();
    OSIntExit();
	
//	if(gSysTimout > 0)gSysTimout--;//5m????
//	dectimers();
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
#if 1	//´®¿Ú1
/***************************** USART1ÖÐ¶Ï·þÎñ *********************************/
void USART1_IRQHandler(void)
{

	uint8_t NewData = 0;
    OS_CPU_SR  cpu_sr;

	NewData = NewData;
	OS_ENTER_CRITICAL();
	OSIntNesting++;
	OS_EXIT_CRITICAL();


	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		NewData = USART1->CR1;
		NewData = USART1->SR;
		NewData = USART1->DR;
		NewData = 1;

		UsartDMARx(&nCom1);
	}
    OSIntExit();

		/*********
	uint8_t NewData = 0;
    OS_CPU_SR  cpu_sr;

	NewData = NewData;
    OS_ENTER_CRITICAL();
    OSIntNesting++;
    OS_EXIT_CRITICAL();
		
     if(USART_GetITStatus(USART1, USART_IT_RXNE )==SET)//????
     {
             USART_ClearITPendingBit(USART1,USART_IT_RXNE);
             NewData = USART_ReceiveData(USART1); 
             if(NewData!=0)
             {
                    //Usart1RXArray[Usart1RXLen++]=RxData;
             }
     }
		
    OSIntExit();	
		********************/
} 

#else	//

#endif

/***************************** USART2ÖÐ¶Ï·þÎñ *********************************/
void USART2_IRQHandler(void)
{
	uint8_t NewData = 0;
    OS_CPU_SR  cpu_sr;

	NewData = NewData;
    OS_ENTER_CRITICAL();
    OSIntNesting++;
    OS_EXIT_CRITICAL();

	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		NewData = USART2->CR1;
		NewData = USART2->SR;
		NewData = USART2->DR;
		NewData = 1;

		UsartDMARx(&nCom2);		
	}	
	
    OSIntExit();
}

/***************************** USART3ÖÐ¶Ï·þÎñ *********************************/
void USART3_IRQHandler(void)
{
	uint8_t NewData = 0;
    OS_CPU_SR  cpu_sr;

	NewData = NewData;
    OS_ENTER_CRITICAL();
    OSIntNesting++;
    OS_EXIT_CRITICAL();

	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{
		NewData = USART3->CR1;
		NewData = USART3->SR;
		NewData = USART3->DR;
		NewData = 1;

		UsartDMARx(&nCom3);	
		//ledToggle(1);	
	}	
	
    OSIntExit();
} 

/******************************************************************************
**´®¿Ú1 DMA½ÓÊÕÍê³ÉÖÐ¶Ï
******************************************************************************/
void DMA1_Channel5_IRQHandler(void)
{
	volatile uint32_t SR,DR;
	
	//ÊÇ·ñÍêÈ«½ÓÊÜÖÐ¶Ï
	if(DMA_GetITStatus(DMA1_FLAG_TC5)!= RESET)
	{
	    //¹Ø±ÕDMAÍ¨µÀ
	    DMA_Cmd(DMA1_Channel5, DISABLE);
	    //Çå³ýÖÐ¶Ï±êÖ¾
	    DMA_ClearFlag(DMA1_FLAG_TC5);
    }
	else if(DMA_GetITStatus(DMA1_FLAG_TE5)!= RESET)//ÖØÊÔ
	{
	    //¹Ø±ÕDMAÍ¨µÀ
	    DMA_Cmd(DMA1_Channel5, DISABLE);
	    //Çå³ýÖÐ¶Ï±êÖ¾
	    DMA_ClearFlag(DMA1_FLAG_TE5);
	}
	
	//DAM½ÓÊÕÂúÖÐ¶Ïºó ´®¿Ú¿ÉÄÜÒç³ö(¹ýÔØ)£¬ËùÒÔÒªÒ»Ö±¶ÁSR ºÍCR ½«¹ýÔØ±êÖ¾ÇåÁã
	SR = USART1->SR; 
	while(SR & 0x08)
	{
		DR = USART1->CR1;
		DR = USART1->CR2;
		DR = USART1->CR3;
		DR = USART1->DR;
		SR = USART1->SR;
		DR = DR;
		SR = SR;
	}
	//È¡DMAÊý¾Ý	
}

/*****************************************************************************
**´®¿Ú1 DMA·¢ËÍÍê³ÉÖÐ¶Ï  
******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_FLAG_TC4)!= RESET)
	{
	    //¹Ø±ÕDMA
	    DMA_Cmd(DMA1_Channel4, DISABLE);
	    //Çå³ýÖÐ¶Ï±êÖ¾
	    DMA_ClearFlag(DMA1_FLAG_TC4);
		
		nCom1.nUsartSt.IsTxEnd = 1;
    }
}

/*****************************************************************************
**´®¿Ú2 DMA½ÓÊÕÍê³ÉÖÐ¶Ï
******************************************************************************/
void DMA1_Channel6_IRQHandler(void)
{
	volatile uint32_t SR,DR;
	
	//ÊÇ·ñÍêÈ«½ÓÊÜÖÐ¶Ï
	if(DMA_GetITStatus(DMA1_FLAG_TC6)!= RESET)
	{
	    //¹Ø±ÕDMAÍ¨µÀ
	    DMA_Cmd(DMA1_Channel6, DISABLE);
	    //Çå³ýÖÐ¶Ï±êÖ¾
	    DMA_ClearFlag(DMA1_FLAG_TC6);
    }
	else if(DMA_GetITStatus(DMA1_FLAG_TE6)!= RESET)//ÖØÊÔ
	{
	    //¹Ø±ÕDMAÍ¨µÀ
	    DMA_Cmd(DMA1_Channel6, DISABLE);
	    //Çå³ýÖÐ¶Ï±êÖ¾
	    DMA_ClearFlag(DMA1_FLAG_TE6);
	}
	
	//DAM½ÓÊÕÂúÖÐ¶Ïºó ´®¿Ú¿ÉÄÜÒç³ö(¹ýÔØ)£¬ËùÒÔÒªÒ»Ö±¶ÁSR ºÍCR ½«¹ýÔØ±êÖ¾ÇåÁã
	SR = USART2->SR; 
	while(SR & 0x08)
	{
		DR = USART2->CR1;
		DR = USART2->CR2;
		DR = USART2->CR3;
		DR = USART2->DR;
		SR = USART2->SR;
		DR = DR;
		SR = SR;
	}
	//È¡DMAÊý¾Ý
}
	
/****************************************************************************
**´®¿Ú2 DMA·¢ËÍÍê³ÉÖÐ¶Ï  
*****************************************************************************/
void DMA1_Channel7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_FLAG_TC7)!= RESET)
	{
	    //¹Ø±ÕDMA
	    DMA_Cmd(DMA1_Channel7, DISABLE);
	    //Çå³ýÖÐ¶Ï±êÖ¾
	    DMA_ClearFlag(DMA1_FLAG_TC7);
		
		nCom2.nUsartSt.IsTxEnd = 1;
//		while (!(nCom2.nCom->SR & USART_FLAG_TXE));			//µÈ´ý·¢ËÍ»º³åÆ÷¿Õ
//		while (USART_GetFlagStatus(nCom2.nCom, USART_FLAG_TC) == RESET);//µÈ´ý·¢ËÍÍê³É
    }	
}

/****************************************************************************
**´®¿Ú3 DMA½ÓÊÕÍê³ÉÖÐ¶Ï  
*****************************************************************************/
void DMA1_Channel3_IRQHandler(void)
{
	volatile uint32_t SR,DR;
	
	//ÊÇ·ñÍêÈ«½ÓÊÜÖÐ¶Ï
	if(DMA_GetITStatus(DMA1_FLAG_TC3)!= RESET)
	{
	    //¹Ø±ÕDMAÍ¨µÀ
	    DMA_Cmd(DMA1_Channel3, DISABLE);
	    //Çå³ýÖÐ¶Ï±êÖ¾
	    DMA_ClearFlag(DMA1_FLAG_TC3);
    }
	else if(DMA_GetITStatus(DMA1_FLAG_TE3)!= RESET)//ÖØÊÔ
	{
	    //¹Ø±ÕDMAÍ¨µÀ
	    DMA_Cmd(DMA1_Channel3, DISABLE);
	    //Çå³ýÖÐ¶Ï±êÖ¾
	    DMA_ClearFlag(DMA1_FLAG_TE3);
	}
	
	//DAM½ÓÊÕÂúÖÐ¶Ïºó ´®¿Ú¿ÉÄÜÒç³ö(¹ýÔØ)£¬ËùÒÔÒªÒ»Ö±¶ÁSR ºÍCR ½«¹ýÔØ±êÖ¾ÇåÁã
	SR = USART3->SR; 
	while(SR & 0x08)
	{
		DR = USART3->CR1;
		DR = USART3->CR2;
		DR = USART3->CR3;
		DR = USART3->DR;
		SR = USART3->SR;
		DR = DR;
		SR = SR;
	}
	//È¡DMAÊý¾Ý
}

/****************************************************************************
**´®¿Ú3 ·¢ËÍÍê³ÉÖÐ¶Ï
*****************************************************************************/
void DMA1_Channel2_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_FLAG_TC2)!= RESET)
	{
	    //¹Ø±ÕDMA
	    DMA_Cmd(DMA1_Channel2, DISABLE);
	    //Çå³ýÖÐ¶Ï±êÖ¾
	    DMA_ClearFlag(DMA1_FLAG_TC2);
		
		nCom3.nUsartSt.IsTxEnd = 1;
    }		
}

/****************************************************************************
**Íâ²¿ÖÐ¶ÏÏ
*****************************************************************************/
INT8U state;
INT32U time_left;
void EXTI2_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line2)!=RESET) //keyA
  {
    EXTI_ClearITPendingBit(EXTI_Line2);
		state = OSTmrStateGet(timekeyA, &err_timer);
		if(state == OS_TMR_STATE_UNUSED || state == OS_TMR_STATE_STOPPED)
		{
			if(GPIO_ReadInputDataBit(KEY_A_GPIO,KEY_A)==0)
			{
				//Æô¶¯¶¨Ê±Æ÷
				timekeyA = OSTmrCreate (80, 0, OS_TMR_OPT_ONE_SHOT, timekey_callback, NULL, "timekeyA", &err_timer);//2s
				OSTmrStart (timekeyA, &err_timer);
			}
		}
		else if(state == OS_TMR_STATE_COMPLETED)//the timer is in ONE-SHOT mode and has completed it's timeout
		{
			OSTmrDel(timekeyA, &err_timer);
			keyst[4]=3;
		}
		else if(state == OS_TMR_STATE_RUNNING)
		{
			time_left=OSTmrRemainGet(timekeyA, &err_timer);
			if(time_left<198 )//20msÒÔÉÏ
			{
				keyst[4]=2;
			}
			OSTmrDel(timekeyA, &err_timer);
		}
  }
  EXTI_ClearITPendingBit(EXTI_Line2);

}

void EXTI3_IRQHandler(void)
{
	
  if(EXTI_GetITStatus(EXTI_Line3)!=RESET)//keyB
  {
    EXTI_ClearITPendingBit(EXTI_Line3);
		state = OSTmrStateGet(timekeyB, &err_timer);
		if(state == OS_TMR_STATE_UNUSED || state == OS_TMR_STATE_STOPPED)
		{
			if(GPIO_ReadInputDataBit(KEY_B_GPIO,KEY_B)==0)
			{
				//Æô¶¯¶¨Ê±Æ÷
				timekeyB = OSTmrCreate (80, 0, OS_TMR_OPT_ONE_SHOT, timekey_callback, NULL, "timekeyB", &err_timer);//2s
				OSTmrStart (timekeyB, &err_timer);
			}
		}
		else if(state == OS_TMR_STATE_COMPLETED)//the timer is in ONE-SHOT mode and has completed it's timeout
		{
			OSTmrDel(timekeyB, &err_timer);
			keyst[5]=3;
		}
		else if(state == OS_TMR_STATE_RUNNING)
		{
			time_left=OSTmrRemainGet(timekeyB, &err_timer);
			if(time_left<198 )//20msÒÔÉÏ
			{
				keyst[5]=2;
			}
			OSTmrDel(timekeyB, &err_timer);
		}
  }
  EXTI_ClearITPendingBit(EXTI_Line3);

}

void EXTI15_10_IRQHandler(void)
{

  if(EXTI_GetITStatus(EXTI_Line13)!=RESET) //Ò»¼þ·µº½
  {
    EXTI_ClearITPendingBit(EXTI_Line13);
		state = OSTmrStateGet(timekeyHome, &err_timer);
		if(state == OS_TMR_STATE_UNUSED || state == OS_TMR_STATE_STOPPED)
		{
			if(GPIO_ReadInputDataBit(KEY_HOME_GPIO,KEY_HOME)==0)
			{
				OSTmrDel(timekeyHome, &err_timer);
				//Æô¶¯¶¨Ê±Æ÷
				timekeyHome = OSTmrCreate (80, 0, OS_TMR_OPT_ONE_SHOT, timekey_callback, NULL, "timekeyHome", &err_timer);//2s
				OSTmrStart (timekeyHome, &err_timer);
			}
		}
		else if(state == OS_TMR_STATE_COMPLETED)//the timer is in ONE-SHOT mode and has completed it's timeout
		{
			OSTmrDel(timekeyHome, &err_timer);
			keyst[1]=3;
		}
		else if(state == OS_TMR_STATE_RUNNING)
		{
			time_left=OSTmrRemainGet(timekeyHome, &err_timer);
			if(time_left<198 )//20msÒÔÉÏ
			{
				keyst[1]=2;
			}
			OSTmrDel(timekeyHome, &err_timer);
		}
  }
	else if(EXTI_GetITStatus(EXTI_Line14)!=RESET)  //ÅÄÕÕ
  {
    EXTI_ClearITPendingBit(EXTI_Line14);
		state = OSTmrStateGet(timekeyPhoto, &err_timer);
		if(state == OS_TMR_STATE_UNUSED || state == OS_TMR_STATE_STOPPED)
		{
			if(GPIO_ReadInputDataBit(KEY_PHOTO_GPIO,KEY_PHOTO)==0)
			{
			//Æô¶¯¶¨Ê±Æ÷
				timekeyPhoto = OSTmrCreate (80, 0, OS_TMR_OPT_ONE_SHOT, timekey_callback, NULL, "timekeyPhoto", &err_timer);//2s
				OSTmrStart (timekeyPhoto, &err_timer);
			}
		}
		else if(state == OS_TMR_STATE_COMPLETED)//the timer is in ONE-SHOT mode and has completed it's timeout
		{
			OSTmrDel(timekeyPhoto, &err_timer);
			keyst[3]=3;
		}
		else if(state == OS_TMR_STATE_RUNNING)
		{
			time_left=OSTmrRemainGet(timekeyPhoto, &err_timer);
			if(time_left<198 )//20msÒÔÉÏ
			{
				keyst[3]=2;
			}
			OSTmrDel(timekeyPhoto, &err_timer);
		}

  }
	else if(EXTI_GetITStatus(EXTI_Line15)!=RESET) //OKF
  {
    EXTI_ClearITPendingBit(EXTI_Line15);
		state = OSTmrStateGet(timekeyOkf, &err_timer);
		if(state == OS_TMR_STATE_UNUSED || state == OS_TMR_STATE_STOPPED)
		{
			if(GPIO_ReadInputDataBit(KEY_OKF_GPIO,KEY_OKF)==0)
			{
				//Æô¶¯¶¨Ê±Æ÷
				timekeyOkf = OSTmrCreate (80, 0, OS_TMR_OPT_ONE_SHOT, timekey_callback, NULL, "timekeyOkf", &err_timer);//2s
				OSTmrStart (timekeyOkf, &err_timer);
			}
		}
		else if(state == OS_TMR_STATE_COMPLETED)//the timer is in ONE-SHOT mode and has completed it's timeout
		{
			OSTmrDel(timekeyOkf, &err_timer);
			keyst[2]=3;
		}
		else if(state == OS_TMR_STATE_RUNNING)
		{
			time_left=OSTmrRemainGet(timekeyOkf, &err_timer);
			if(time_left<198 )//10msÒÔÉÏ
			{
				keyst[2]=2;
			}
			OSTmrDel(timekeyOkf, &err_timer);
		}
  }

}

/****************************************************************************
**usb
*****************************************************************************/

void USB_LP_CAN1_RX0_IRQHandler(void)
{
	LedOn(LED_GPS_G);
	LedOff(LED_GPS_R);
  USB_Istr();

}
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
