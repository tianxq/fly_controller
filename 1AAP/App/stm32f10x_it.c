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

    OSIntExit();
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
#if 1
/***************************** USART1�жϷ��� *********************************/
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
}
#endif

/******************************************************************************
**����1 DMA���������ж�
******************************************************************************/
void DMA1_Channel5_IRQHandler(void)
{
		volatile uint32_t SR,DR;
	if(DMA_GetITStatus(DMA1_FLAG_TC5)!= RESET)
	{
	   
	    DMA_Cmd(DMA1_Channel5, DISABLE);
	  
	    DMA_ClearFlag(DMA1_FLAG_TC5);
  }
	else if(DMA_GetITStatus(DMA1_FLAG_TE5)!= RESET)//����
	{	    
	    DMA_Cmd(DMA1_Channel5, DISABLE);
	   
	    DMA_ClearFlag(DMA1_FLAG_TE5);
	}
	
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

}

/*****************************************************************************
**����1 DMA���������ж�
******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_FLAG_TC4)!= RESET)
	{

	    DMA_Cmd(DMA1_Channel4, DISABLE);

	    DMA_ClearFlag(DMA1_FLAG_TC4);
		
		nCom1.nUsartSt.IsTxEnd = 1;
    }
}


/****************************************************************************
**�ⲿ�ж��
*****************************************************************************/

extern void Keyboard_ISR(void);

void EXTI2_IRQHandler(void)
{
	uint8_t NewData = 0;
  OS_CPU_SR  cpu_sr;
	
	NewData = NewData;
	OS_ENTER_CRITICAL();
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	
	Keyboard_ISR();
	
	OSIntExit();
}


void EXTI3_IRQHandler(void)
{
	uint8_t NewData = 0;
  OS_CPU_SR  cpu_sr;
	
	NewData = NewData;
	OS_ENTER_CRITICAL();
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	
	Keyboard_ISR();
	
	OSIntExit();
}

void EXTI15_10_IRQHandler(void)
{
	uint8_t NewData = 0;
  OS_CPU_SR  cpu_sr;
	
	NewData = NewData;
	OS_ENTER_CRITICAL();
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	
	Keyboard_ISR();
	
	OSIntExit();
}

/****************************************************************************
**usb
*****************************************************************************/
void USBWakeUp_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line18);
}
void USB_LP_CAN1_RX0_IRQHandler(void)
{
//	LedOn(LED_GPS_G);
//	LedOff(LED_X_G);

		USB_Istr();


}
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
