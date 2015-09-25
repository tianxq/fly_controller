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
#if 1	//����1
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

/***************************** USART2�жϷ��� *********************************/
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

/***************************** USART3�жϷ��� *********************************/
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
**����1 DMA��������ж�
******************************************************************************/
void DMA1_Channel5_IRQHandler(void)
{
	volatile uint32_t SR,DR;
	
	//�Ƿ���ȫ�����ж�
	if(DMA_GetITStatus(DMA1_FLAG_TC5)!= RESET)
	{
	    //�ر�DMAͨ��
	    DMA_Cmd(DMA1_Channel5, DISABLE);
	    //����жϱ�־
	    DMA_ClearFlag(DMA1_FLAG_TC5);
    }
	else if(DMA_GetITStatus(DMA1_FLAG_TE5)!= RESET)//����
	{
	    //�ر�DMAͨ��
	    DMA_Cmd(DMA1_Channel5, DISABLE);
	    //����жϱ�־
	    DMA_ClearFlag(DMA1_FLAG_TE5);
	}
	
	//DAM�������жϺ� ���ڿ������(����)������Ҫһֱ��SR ��CR �����ر�־����
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
	//ȡDMA����	
}

/*****************************************************************************
**����1 DMA��������ж�  
******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_FLAG_TC4)!= RESET)
	{
	    //�ر�DMA
	    DMA_Cmd(DMA1_Channel4, DISABLE);
	    //����жϱ�־
	    DMA_ClearFlag(DMA1_FLAG_TC4);
		
		nCom1.nUsartSt.IsTxEnd = 1;
    }
}

/*****************************************************************************
**����2 DMA��������ж�
******************************************************************************/
void DMA1_Channel6_IRQHandler(void)
{
	volatile uint32_t SR,DR;
	
	//�Ƿ���ȫ�����ж�
	if(DMA_GetITStatus(DMA1_FLAG_TC6)!= RESET)
	{
	    //�ر�DMAͨ��
	    DMA_Cmd(DMA1_Channel6, DISABLE);
	    //����жϱ�־
	    DMA_ClearFlag(DMA1_FLAG_TC6);
    }
	else if(DMA_GetITStatus(DMA1_FLAG_TE6)!= RESET)//����
	{
	    //�ر�DMAͨ��
	    DMA_Cmd(DMA1_Channel6, DISABLE);
	    //����жϱ�־
	    DMA_ClearFlag(DMA1_FLAG_TE6);
	}
	
	//DAM�������жϺ� ���ڿ������(����)������Ҫһֱ��SR ��CR �����ر�־����
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
	//ȡDMA����
}
	
/****************************************************************************
**����2 DMA��������ж�  
*****************************************************************************/
void DMA1_Channel7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_FLAG_TC7)!= RESET)
	{
	    //�ر�DMA
	    DMA_Cmd(DMA1_Channel7, DISABLE);
	    //����жϱ�־
	    DMA_ClearFlag(DMA1_FLAG_TC7);
		
		nCom2.nUsartSt.IsTxEnd = 1;
//		while (!(nCom2.nCom->SR & USART_FLAG_TXE));			//�ȴ����ͻ�������
//		while (USART_GetFlagStatus(nCom2.nCom, USART_FLAG_TC) == RESET);//�ȴ��������
    }	
}

/****************************************************************************
**����3 DMA��������ж�  
*****************************************************************************/
void DMA1_Channel3_IRQHandler(void)
{
	volatile uint32_t SR,DR;
	
	//�Ƿ���ȫ�����ж�
	if(DMA_GetITStatus(DMA1_FLAG_TC3)!= RESET)
	{
	    //�ر�DMAͨ��
	    DMA_Cmd(DMA1_Channel3, DISABLE);
	    //����жϱ�־
	    DMA_ClearFlag(DMA1_FLAG_TC3);
    }
	else if(DMA_GetITStatus(DMA1_FLAG_TE3)!= RESET)//����
	{
	    //�ر�DMAͨ��
	    DMA_Cmd(DMA1_Channel3, DISABLE);
	    //����жϱ�־
	    DMA_ClearFlag(DMA1_FLAG_TE3);
	}
	
	//DAM�������жϺ� ���ڿ������(����)������Ҫһֱ��SR ��CR �����ر�־����
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
	//ȡDMA����
}

/****************************************************************************
**����3 ��������ж�
*****************************************************************************/
void DMA1_Channel2_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_FLAG_TC2)!= RESET)
	{
	    //�ر�DMA
	    DMA_Cmd(DMA1_Channel2, DISABLE);
	    //����жϱ�־
	    DMA_ClearFlag(DMA1_FLAG_TC2);
		
		nCom3.nUsartSt.IsTxEnd = 1;
    }		
}

/****************************************************************************
**�ⲿ�ж��
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

//�������ݽ���FIFO
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
    CLI();   //���ж�
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
    SEI();	  //���ж�
  }
  EXTI_ClearITPendingBit(EXTI_Line0);
}

#endif
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
