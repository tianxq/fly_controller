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
#if 1	//串口1用作PSAM
/***************************** USART1中断服务 *********************************/
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


#else	//
void SC_USART_IRQHandler(void)
{
    /* If a Frame error is signaled by the card */
    if(USART_GetITStatus(SC_USART, USART_IT_FE) != RESET)
    {
        USART_ReceiveData(SC_USART);

        /* Resend the byte that failed to be received (by the Smartcard) correctly */
        SC_ParityErrorHandler();
    }

    /* If the SC_USART detects a parity error */
    if(USART_GetITStatus(SC_USART, USART_IT_PE) != RESET)
    {
        /* Enable SC_USART RXNE Interrupt (until receiving the corrupted byte) */
        USART_ITConfig(SC_USART, USART_IT_RXNE, ENABLE);
        /* Flush the SC_USART DR register */
        USART_ReceiveData(SC_USART);
    }

    if(USART_GetITStatus(SC_USART, USART_IT_RXNE) != RESET)
    {
        /* Disable SC_USART RXNE Interrupt */
        USART_ITConfig(SC_USART, USART_IT_RXNE, DISABLE);
        USART_ReceiveData(SC_USART);
    }

    /* If a Overrun error is signaled by the card */
    if(USART_GetITStatus(SC_USART, USART_IT_ORE) != RESET)
    {
        USART_ReceiveData(SC_USART);
    }
    /* If a Noise error is signaled by the card */
    if(USART_GetITStatus(SC_USART, USART_IT_NE) != RESET)
    {
        USART_ReceiveData(SC_USART);
    }
}
#endif

/***************************** USART2中断服务 *********************************/
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

/***************************** USART3中断服务 *********************************/
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
**串口1 DMA接收完成中断
******************************************************************************/
void DMA1_Channel5_IRQHandler(void)
{
	volatile uint32_t SR,DR;
	
	//是否完全接受中断
	if(DMA_GetITStatus(DMA1_FLAG_TC5)!= RESET)
	{
	    //关闭DMA通道
	    DMA_Cmd(DMA1_Channel5, DISABLE);
	    //清除中断标志
	    DMA_ClearFlag(DMA1_FLAG_TC5);
    }
	else if(DMA_GetITStatus(DMA1_FLAG_TE5)!= RESET)//重试
	{
	    //关闭DMA通道
	    DMA_Cmd(DMA1_Channel5, DISABLE);
	    //清除中断标志
	    DMA_ClearFlag(DMA1_FLAG_TE5);
	}
	
	//DAM接收满中断后 串口可能溢出(过载)，所以要一直读SR 和CR 将过载标志清零
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
	//取DMA数据	
}

/*****************************************************************************
**串口1 DMA发送完成中断  
******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_FLAG_TC4)!= RESET)
	{
	    //关闭DMA
	    DMA_Cmd(DMA1_Channel4, DISABLE);
	    //清除中断标志
	    DMA_ClearFlag(DMA1_FLAG_TC4);
		
		nCom1.nUsartSt.IsTxEnd = 1;
    }
}

/*****************************************************************************
**串口2 DMA接收完成中断
******************************************************************************/
void DMA1_Channel6_IRQHandler(void)
{
	volatile uint32_t SR,DR;
	
	//是否完全接受中断
	if(DMA_GetITStatus(DMA1_FLAG_TC6)!= RESET)
	{
	    //关闭DMA通道
	    DMA_Cmd(DMA1_Channel6, DISABLE);
	    //清除中断标志
	    DMA_ClearFlag(DMA1_FLAG_TC6);
    }
	else if(DMA_GetITStatus(DMA1_FLAG_TE6)!= RESET)//重试
	{
	    //关闭DMA通道
	    DMA_Cmd(DMA1_Channel6, DISABLE);
	    //清除中断标志
	    DMA_ClearFlag(DMA1_FLAG_TE6);
	}
	
	//DAM接收满中断后 串口可能溢出(过载)，所以要一直读SR 和CR 将过载标志清零
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
	//取DMA数据
}
	
/****************************************************************************
**串口2 DMA发送完成中断  
*****************************************************************************/
void DMA1_Channel7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_FLAG_TC7)!= RESET)
	{
	    //关闭DMA
	    DMA_Cmd(DMA1_Channel7, DISABLE);
	    //清除中断标志
	    DMA_ClearFlag(DMA1_FLAG_TC7);
		
		nCom2.nUsartSt.IsTxEnd = 1;
//		while (!(nCom2.nCom->SR & USART_FLAG_TXE));			//等待发送缓冲器空
//		while (USART_GetFlagStatus(nCom2.nCom, USART_FLAG_TC) == RESET);//等待发送完成
    }	
}

/****************************************************************************
**串口3 DMA接收完成中断  
*****************************************************************************/
void DMA1_Channel3_IRQHandler(void)
{
	volatile uint32_t SR,DR;
	
	//是否完全接受中断
	if(DMA_GetITStatus(DMA1_FLAG_TC3)!= RESET)
	{
	    //关闭DMA通道
	    DMA_Cmd(DMA1_Channel3, DISABLE);
	    //清除中断标志
	    DMA_ClearFlag(DMA1_FLAG_TC3);
    }
	else if(DMA_GetITStatus(DMA1_FLAG_TE3)!= RESET)//重试
	{
	    //关闭DMA通道
	    DMA_Cmd(DMA1_Channel3, DISABLE);
	    //清除中断标志
	    DMA_ClearFlag(DMA1_FLAG_TE3);
	}
	
	//DAM接收满中断后 串口可能溢出(过载)，所以要一直读SR 和CR 将过载标志清零
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
	//取DMA数据
}

/****************************************************************************
**串口3 发送完成中断
*****************************************************************************/
void DMA1_Channel2_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_FLAG_TC2)!= RESET)
	{
	    //关闭DMA
	    DMA_Cmd(DMA1_Channel2, DISABLE);
	    //清除中断标志
	    DMA_ClearFlag(DMA1_FLAG_TC2);
		
		nCom3.nUsartSt.IsTxEnd = 1;
    }		
}

/****************************************************************************
**外部中断�
*****************************************************************************/
uint8_t  sbusData2[6]={0x5a,0xa5};
void EXTI2_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line2)!=RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line2);
		RS485Send(1,sbusData2,6);
  }
  EXTI_ClearITPendingBit(EXTI_Line2);
}

//无线数据接收FIFO
#if 0
Queue QueueRFrxFIFO;
void EXTI0_IRQHandler(void)
{

       /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  RFrxMsg rfrxMsg;
  unsigned char D_Status;
  
  if(EXTI_GetITStatus(EXTI_Line0)!=RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line0);
    CLI();   //关中断
    //LED_1_T;
    
    D_Status=SpiReadRegister(DeviceStatus);
    
    if((D_Status&0x02)!=0x02)
    {
      rfrxMsg.rx_len=RFReacPacket(rfrxMsg.rx_data);
      if(rfrxMsg.rx_len > 0)
      {
        InsertQueue(&QueueRFrxFIFO, &rfrxMsg, CopyQueue_RFrxFIFO);
		//RS485Send(2,rfrxMsg.rx_data,rfrxMsg.rx_len);
      }
    }
    Rx_Mode_Entern();
    RxFIFOReset();
    //LED_1_T;
    SEI();	  //开中断
  }
  EXTI_ClearITPendingBit(EXTI_Line0);
}

#endif
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
