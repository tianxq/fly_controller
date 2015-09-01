/**
  ******************************************************************************
  * @file    Smartcard/src/smartcard.c
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    07/27/2009
  * @brief   This file provides all the Smartcard firmware functions.
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

/** @addtogroup Smartcard
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "includes.h"

/* Private typedef -----------------------------------------------------------*/


/* ATR structure - Answer To Reset -------------------------------------------*/
/*应答复位*/
/*格式：TS|T0[TAi|TBi|TCi|TDi] [T1...TK]|TCK*/
/*
T0:格式字符，必选。其意义是：假设T0（byte） = b8b7b6b5b4b3b2b1(b1~b8为1bit)。其中T0的高四个比特b8b7b6b5分别指示的是TDi|TCi|TBi|TAi。
当b8=1时，复位序列中有TDi,当b8=0时，复位序列中没有TDi；
当b7=1时，复位序列中有TCi,当b7=0时，复位序列中没有TCi；
当b6=1时，复位序列中有TBi,当b6=0时，复位序列中没有TBi；
当b5=1时，复位序列中有TAi,当b5=0时，复位序列中没有TAi。
T0的低四个比特b4b3b2b1指示历史比特T1...TK的长度，如b4b3b2b1 = 0110，则序列中会有6个历史比特。

T=0 is the asynchronous half duplex character transmission protocol.
T=1 is the asynchronous half duplex block transmission protocol.
TCK:校验字符，可选。在T0这种情况下将不出现。（T0:异步半双工字符传输协议，T1:异步半双工块传输协议）
*/

/* Private define ------------------------------------------------------------*/
/*-------------------------- Answer to reset Commands ------------------------*/
#define SC_GET_A2R         0x00
/* SC STATUS: Status Code ----------------------------------------------------*/
#define SC_EF_SELECTED     0x9F
#define SC_DF_SELECTED     0x9F
#define SC_OP_TERMINATED   0x9000
/* Private macro -------------------------------------------------------------*/
/**
  * @brief  Select CARD:  Select pin low
  */
//#define SC_SEL_LOW()       GPIO_ResetBits(SC_PIN_SEL_GPIO, SC_PIN_SEL)
/**
  * @brief  Deselect CADR:  Select pin high
  */
//#define SC_SEL_HIGH()      GPIO_SetBits(SC_PIN_SEL_GPIO, SC_PIN_SEL)

/**
  * @brief  SC_Reset: Chip Select pin low
  */
#define SC_Reset_LOW()       GPIO_ResetBits(SC_PIN_RESET_GPIO, SC_PIN_RESET)
/**
  * @brief  SC_Reset: Chip Select pin high
  */
#define SC_Reset_HIGH()      GPIO_SetBits(SC_PIN_RESET_GPIO, SC_PIN_RESET)
/**
  * @}
  */
/* Private variables ---------------------------------------------------------*/
SC_State SCState[2]= {SC_POWER_OFF,SC_POWER_OFF} ;
__IO uint8_t CardInserted = 0;//=0:没有卡插入；=1：有卡插入
/* Global variables definition and initialization ----------------------------*/
//uint32_t __IO CardInserted = 0;//=0:没有卡插入；=1：有卡插入
//extern SC_ADPU_Commands SC_ADPU;

static uint8_t SC_ATR_Table[40];
SC_ATR SC_A2R;
static __IO uint8_t SCData = 0;

//波特率因子
//static uint32_t F_Table[16] = {0, 372, 558, 744, 1116, 1488, 1860, 0,
//                               0, 512, 768, 1024, 1536, 2048, 0, 0};
//时钟因子
//static uint32_t D_Table[9] = {0, 1, 2, 4, 8, 16, 0, 0, 12};

/* Private function prototypes -----------------------------------------------*/
/* Transport Layer -----------------------------------------------------------*/
/*--------------APDU-----------*/
static void SC_SendData(SC_ADPU_Commands *SC_ADPU, SC_ADPU_Responce *SC_ResponceStatus);

/*------------ ATR ------------*/
static void SC_AnswerReq(SC_State *SCState, uint8_t *card, uint8_t length);  /* Ask ATR */
static uint8_t SC_decode_Answer2reset(uint8_t *card);  /* Decode ATR */

/* Physical Port Layer -------------------------------------------------------*/
static void SC_Init(void);
static void SC_DeInit(void);
//static void SC_VoltageConfig(uint32_t SC_Voltage);
//static uint8_t SC_Detect(void);
static ErrorStatus USART_ByteReceive(uint8_t *Data, uint32_t TimeOut);

/* Private functions ---------------------------------------------------------*/
#define HC40521S0H()	GPIO_SetBits(GPIOA,GPIO_Pin_12)	//pb14
#define HC40521S0L()	GPIO_ResetBits(GPIOA,GPIO_Pin_12)

#define HC40521S1H()	GPIO_SetBits(GPIOA,GPIO_Pin_11)
#define HC40521S1L()	GPIO_ResetBits(GPIOA,GPIO_Pin_11)

#define HC40522S0H()	GPIO_SetBits(GPIOB,GPIO_Pin_14)	//pb14
#define HC40522S0L()	GPIO_ResetBits(GPIOB,GPIO_Pin_14)

#define HC40522S1H()	GPIO_SetBits(GPIOB,GPIO_Pin_13)
#define HC40522S1L()	GPIO_ResetBits(GPIOB,GPIO_Pin_13)
/***********************************************************************************
**切换PSAM
************************************************************************************/
#ifdef KHP_T4
unsigned char SC_SwitchPSAM(PsamSlot slot)
{
	if(slot > PSAM6)return 1;
	
	if(slot == PSAM1)
	{
		HC40522S1H();		
		HC40522S0H();
		HC40521S1L();		
		HC40521S0L();
	}
	else if(slot == PSAM2)
	{
		HC40522S1H();		
		HC40522S0H();
		HC40521S1H();		
		HC40521S0L();	
	}
	else if(slot == PSAM3)
	{
		HC40522S1H();		
		HC40522S0H();
		HC40521S1L();		
		HC40521S0H();		
	}
	else if(slot == PSAM4)
	{
		HC40521S1H();		
		HC40521S0H();
		HC40522S1L();		
		HC40522S0L();
	}
	else if(slot == PSAM5)
	{
		HC40521S1H();		
		HC40521S0H();
		HC40522S1H();		
		HC40522S0L();		
	}
	else if(slot == PSAM6)
	{
		HC40521S1H();		
		HC40521S0H();
		HC40522S1L();		
		HC40522S0H();		
	}
	
	return 0;
}
#else
unsigned char SC_SwitchPSAM(PsamSlot slot)
{
	if(slot > PSAM6)return 0;
	
	if(slot == PSAM1)
	{
		HC40521S1L();		
		HC40521S0L();
	}
	else if(slot == PSAM3)
	{
		HC40521S1H();		
		HC40521S0L();	
	}
	else if(slot == PSAM2)
	{
		HC40521S1L();		
		HC40521S0H();		
	}
	else if(slot == PSAM4)
	{
		HC40521S1H();		
		HC40521S0H();
	}	
	
	return 1;
}
#endif

/**
  * @brief  Handles all Smartcard states and serves to send and receive all
  *   communication data between Smartcard and reader.
  * @param  SCState: pointer to an SC_State enumeration that will contain the Smartcard state.
  * @param  SC_ADPU: pointer to an SC_ADPU_Commands structure that will be initialized.
  * @param  SC_Response: pointer to a SC_ADPU_Responce structure which will be initialized.
  * @retval =1： 接受数据成功；=0:接受数据失败
  */
void SC_Handler(uint8_t CardType, SC_State *SCState, SC_ADPU_Commands *SC_ADPU, SC_ADPU_Responce *SC_Response)
{
    uint32_t i = 0;

//    if (CardType == SC_PSAM_CARD) SC_SEL_LOW();
//    else SC_SEL_HIGH();

	for(i = 0;i<68000;i++)__NOP();  //70000


    switch(*SCState)
    {
    case SC_POWER_ON:
		
        if (SC_ADPU->Header.INS == SC_GET_A2R)
        {
            /* Reset Data from SC buffer -----------------------------------------*/
            for (i = 0; i < 40; i++)
            {
                SC_ATR_Table[i] = 0;
            }

            /* Reset SC_A2R Structure --------------------------------------------*/
            SC_A2R.TS = 0;
            SC_A2R.T0 = 0;
            for (i = 0; i < SETUP_LENGTH; i++)
            {
                SC_A2R.T[i] = 0;
            }
            for (i = 0; i < HIST_LENGTH; i++)
            {
                SC_A2R.H[i] = 0;
            }
            SC_A2R.Tlength = 0;
            SC_A2R.Hlength = 0;

            /* Next State --------------------------------------------------------*/
            *SCState = SC_RESET_LOW;
			
            /* Smartcard intialization ------------------------------------------*/
            SC_Init();			
        }
        break;

    case SC_RESET_LOW:
        if(SC_ADPU->Header.INS == SC_GET_A2R)
        {
            /* If card is detected then Power ON, Card Reset and wait for an answer) */
//        if ((SC_Detect()== 0x0)||(CardType == SC_PSAM_CARD))
            {
                while(((*SCState) != SC_POWER_OFF) && ((*SCState) != SC_ACTIVE))
                {
                    SC_AnswerReq(SCState, &SC_ATR_Table[0], 40); /* Check for answer to reset */
                }
            }
//        else
//        {
//          (*SCState) = SC_POWER_OFF;
//        }
        }
        break;

    case SC_ACTIVE:
        if (SC_ADPU->Header.INS == SC_GET_A2R)
        {
            if(SC_decode_Answer2reset(&SC_ATR_Table[0]) == T0_PROTOCOL)
            {
                (*SCState) = SC_ACTIVE_ON_T0;
            }
            else
            {
                (*SCState) = SC_POWER_OFF;
            }
        }
        break;

    case SC_ACTIVE_ON_T0:
        SC_SendData(SC_ADPU, SC_Response);
        break;

    case SC_POWER_OFF:
        SC_DeInit(); /* Disable Smartcard interface */
        break;

    default:
        (*SCState) = SC_POWER_OFF;
    }

}

/**
  * @brief  Enables or disables the power to the Smartcard.
  * @param  NewState: new state of the Smartcard power supply.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SC_PowerCmd(FunctionalState NewState)
{
//    if(NewState != DISABLE)
//    {
//        GPIO_SetBits(SC_PIN_CMDVCC_GPIO, SC_PIN_CMDVCC);
//    }
//    else
//    {
//        GPIO_ResetBits(SC_PIN_CMDVCC_GPIO, SC_PIN_CMDVCC);
//    }
}

/**
  * @brief  Sets or clears the Smartcard reset pin.
  * @param  ResetState: this parameter specifies the state of the Smartcard
  *   reset pin. BitVal must be one of the BitAction enum values:
  *     @arg Bit_RESET: to clear the port pin.
  *     @arg Bit_SET: to set the port pin.
  * @retval None
  */
//void SC_Reset(BitAction ResetState)
//{
//  GPIO_WriteBit(SC_PIN_RESET_GPIO, SC_PIN_RESET, ResetState);
//}

/**
  * @brief  Resends the byte that failed to be received (by the Smartcard) correctly.
  * @param  None
  * @retval None
  */
void SC_ParityErrorHandler(void)
{
    USART_SendData(SC_USART, SCData);
    while(USART_GetFlagStatus(SC_USART, USART_FLAG_TC) == RESET)
    {
    }
}

/**
  * @brief  Configures the IO speed (BaudRate) communication.
  * @param  None
  * @retval None
  */
void SC_PTSConfig(void)
{
    RCC_ClocksTypeDef RCC_ClocksStatus;
    uint32_t workingbaudrate = 0, apbclock = 0;
    uint8_t locData = 0, PTSConfirmStatus = 1;
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStructure;

    /* Reconfigure the USART Baud Rate -------------------------------------------*/
    RCC_GetClocksFreq(&RCC_ClocksStatus);
    apbclock = RCC_ClocksStatus.PCLK1_Frequency;			//总线时钟
    apbclock /= ((SC_USART->GTPR & (uint16_t)0x00FF) * 2);	//PSAM时钟

    /* Enable the DMA Receive (Set DMAR bit only) to enable interrupt generation
       in case of a framing error FE */
    USART_DMACmd(SC_USART, USART_DMAReq_Rx, ENABLE);

    if((SC_A2R.T0 & (uint8_t)0x10) == 0x10) //TD1存在
    {
        if(SC_A2R.T[0] != 0x11)
        {
            /* Send PTSS */
            SCData = 0xFF;
            USART_SendData(SC_USART, SCData);
            while(USART_GetFlagStatus(SC_USART, USART_FLAG_TC) == RESET)
            {
            }

            /* Send PTS0 */
            SCData = 0x10;
            USART_SendData(SC_USART, SCData);
            while(USART_GetFlagStatus(SC_USART, USART_FLAG_TC) == RESET)
            {
            }

            /* Send PTS1 */
            SCData = SC_A2R.T[0];
            USART_SendData(SC_USART, SCData);
            while(USART_GetFlagStatus(SC_USART, USART_FLAG_TC) == RESET)
            {
            }

            /* Send PCK */
            SCData = (uint8_t)0xFF^(uint8_t)0x10^(uint8_t)SC_A2R.T[0];
            USART_SendData(SC_USART, SCData);
            while(USART_GetFlagStatus(SC_USART, USART_FLAG_TC) == RESET)
            {
            }

            /* Disable the DMA Receive (Reset DMAR bit only) */
            USART_DMACmd(SC_USART, USART_DMAReq_Rx, DISABLE);
			(void)USART_ReceiveData(SC_USART);
			
            if((USART_ByteReceive(&locData, SC_RECEIVE_TIMEOUT)) == SUCCESS)
            {
                if(locData != 0xFF)
                {
                    PTSConfirmStatus = 0x00;
                }
            }
            if((USART_ByteReceive(&locData, SC_RECEIVE_TIMEOUT)) == SUCCESS)
            {
                if(locData != 0x10)
                {
                    PTSConfirmStatus = 0x00;
                }
            }
            if((USART_ByteReceive(&locData, SC_RECEIVE_TIMEOUT)) == SUCCESS)
            {
                if(locData != SC_A2R.T[0])
                {
                    PTSConfirmStatus = 0x00;
                }
            }
            if((USART_ByteReceive(&locData, SC_RECEIVE_TIMEOUT)) == SUCCESS)
            {
                if(locData != ((uint8_t)0xFF^(uint8_t)0x10^(uint8_t)SC_A2R.T[0]))
                {
                    PTSConfirmStatus = 0x00;
                }
            }
            else
            {
                PTSConfirmStatus = 0x00;
            }
            /* PTS Confirm */
            if(PTSConfirmStatus == 0x01)
            {
//				workingbaudrate = apbclock * D_Table[(SC_A2R.T[0] & (uint8_t)0x0F)];
//				workingbaudrate /= F_Table[((SC_A2R.T[0] >> 4) & (uint8_t)0x0F)];
				
				//捷德,银通卡用
				if(SC_A2R.T[0] == 0x13)	//38400
				{
					workingbaudrate = 38400;
				}
				else if(SC_A2R.T[0] == 0x18)//115200
				{
					workingbaudrate = 115200;
				}

                USART_ClockInitStructure.USART_Clock = USART_Clock_Enable;
                USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
                USART_ClockInitStructure.USART_CPHA = USART_CPHA_1Edge;
                USART_ClockInitStructure.USART_LastBit = USART_LastBit_Enable;
                USART_ClockInit(SC_USART, &USART_ClockInitStructure);

                USART_InitStructure.USART_BaudRate = workingbaudrate;
                USART_InitStructure.USART_WordLength = USART_WordLength_9b;
                USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
                USART_InitStructure.USART_Parity = USART_Parity_Even;
                USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
                USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
                USART_Init(SC_USART, &USART_InitStructure);
            }
        }
    }
}

/**
  * @brief  Manages the Smartcard transport layer: send APDU commands and receives
  *   the APDU responce.
  * @param  SC_ADPU: pointer to a SC_ADPU_Commands structure which will be initialized.
  * @param  SC_Response: pointer to a SC_ADPU_Responce structure which will be initialized.
  * @retval = 1:success; =0:false
  */
static void SC_SendData(SC_ADPU_Commands *SC_ADPU, SC_ADPU_Responce *SC_ResponceStatus)
{
    uint32_t i = 0;
    uint8_t locData = 0;

    /* Reset responce buffer ---------------------------------------------------*/
    for(i = 0; i < LC_MAX; i++)
    {
        SC_ResponceStatus->Data[i] = 0;
    }

    SC_ResponceStatus->SW1 = 0;
    SC_ResponceStatus->SW2 = 0;

    /* Enable the DMA Receive (Set DMAR bit only) to enable interrupt generation
       in case of a framing error FE */
//    USART_DMACmd(SC_USART, USART_DMAReq_Rx, ENABLE);

    /* Send header -------------------------------------------------------------*/
    SCData = SC_ADPU->Header.CLA;
    USART_SendData(SC_USART, SCData);
    while(USART_GetFlagStatus(SC_USART, USART_FLAG_TC) == RESET)
    {
    }

    SCData = SC_ADPU->Header.INS;
    USART_SendData(SC_USART, SCData);
    while(USART_GetFlagStatus(SC_USART, USART_FLAG_TC) == RESET)
    {
    }

    SCData = SC_ADPU->Header.P1;
    USART_SendData(SC_USART, SCData);
    while(USART_GetFlagStatus(SC_USART, USART_FLAG_TC) == RESET)
    {
    }

    SCData = SC_ADPU->Header.P2;
    USART_SendData(SC_USART, SCData);
    while(USART_GetFlagStatus(SC_USART, USART_FLAG_TC) == RESET)
    {
    }

    /* Send body length to/from SC ---------------------------------------------*/
    if(SC_ADPU->Body.LC)
    {
        SCData = SC_ADPU->Body.LC;
        USART_SendData(SC_USART, SCData);
        while(USART_GetFlagStatus(SC_USART, USART_FLAG_TC) == RESET)
        {
        }

    }
    else
    {
        SCData = SC_ADPU->Body.LE;
        USART_SendData(SC_USART, SCData);
        while(USART_GetFlagStatus(SC_USART, USART_FLAG_TC) == RESET)
        {
        }
    }

    /* Flush the SC_USART DR */
    (void)USART_ReceiveData(SC_USART);
	
//	USART_ClearFlag(SC_USART,USART_FLAG_RXNE);
//	USART_ClearFlag(SC_USART,USART_FLAG_RXNE);	
    /* --------------------------------------------------------
      Wait Procedure byte from card:
      1 - ACK
      2 - NULL
      3 - SW1; SW2
     -------------------------------------------------------- */

    if((USART_ByteReceive(&locData, SC_RECEIVE_TIMEOUT)) == SUCCESS)
    {
        if(((locData & (uint8_t)0xF0) == 0x60) || ((locData & (uint8_t)0xF0) == 0x90))
        {
            /* SW1 received */
            SC_ResponceStatus->SW1 = locData;

            if((USART_ByteReceive(&locData, SC_RECEIVE_TIMEOUT)) == SUCCESS)
            {
                /* SW2 received */
                SC_ResponceStatus->SW2 = locData;
            }
        }
        else if (((locData & (uint8_t)0xFE) == (((uint8_t)~(SC_ADPU->Header.INS)) & \
                                                (uint8_t)0xFE))||((locData & (uint8_t)0xFE) == (SC_ADPU->Header.INS & (uint8_t)0xFE)))
        {
            SC_ResponceStatus->Data[0] = locData;/* ACK received */
        }
    }

    /* If no status bytes received ---------------------------------------------*/
    if(SC_ResponceStatus->SW1 == 0x00)
    {
        /* Send body data to SC--------------------------------------------------*/
        if (SC_ADPU->Body.LC)
        {
            for(i = 0; i < SC_ADPU->Body.LC; i++)
            {
                SCData = SC_ADPU->Body.Data[i];

                USART_SendData(SC_USART, SCData);
                while(USART_GetFlagStatus(SC_USART, USART_FLAG_TC) == RESET)
                {
                }
            }
            /* Flush the SC_USART DR */
            (void)USART_ReceiveData(SC_USART);
            /* Disable the DMA Receive (Reset DMAR bit only) */
            USART_DMACmd(SC_USART, USART_DMAReq_Rx, DISABLE);
        }

        /* Or receive body data from SC ------------------------------------------*/
        else if (SC_ADPU->Body.LE)
        {
            for(i = 0; i < SC_ADPU->Body.LE; i++)
            {
                if(USART_ByteReceive(&locData, SC_RECEIVE_TIMEOUT) == SUCCESS)
                {
                    SC_ResponceStatus->Data[i] = locData;
                }
            }
        }
        /* Wait SW1 --------------------------------------------------------------*/

        i = 0;
        while(i < 10)
        {
            if(USART_ByteReceive(&locData, SC_RECEIVE_TIMEOUT) == SUCCESS)
            {
                SC_ResponceStatus->SW1 = locData;
                i = 11;
            }
            else
            {
                i++;
            }
        }

        /* Wait SW2 ------------------------------------------------------------*/

        i = 0;
        while(i < 10)
        {
            if(USART_ByteReceive(&locData, SC_RECEIVE_TIMEOUT) == SUCCESS)
            {
                SC_ResponceStatus->SW2 = locData;
                i = 11;
            }
            else
            {
                i++;
            }
        }
    }
}

/**
  * @brief  Requests the reset answer from card.
  * @param  SCState: pointer to an SC_State enumeration that will contain the Smartcard state.
  * @param  card: pointer to a buffer which will contain the card ATR.
  * @param  length: maximum ATR length
  * @retval None
  */
static void SC_AnswerReq(SC_State *SCState, uint8_t *card, uint8_t length)
{
    uint8_t Data = 0;
    uint32_t i = 0;

    switch(*SCState)
    {
    case SC_RESET_LOW:
        /* Check responce with reset low ---------------------------------------*/
        for (i = 0; i < length; i++)
        {
            if((USART_ByteReceive(&Data, SC_RECEIVE_TIMEOUT)) == SUCCESS)
            {
                card[i] = Data;
            }
//			else if(card[0])
//			{
//				break;
//			}
        }
        if(card[0])
        {
            (*SCState) = SC_ACTIVE;
//        SC_Reset(Bit_SET);
            SC_Reset_HIGH();
        }
        else
        {
            (*SCState) = SC_RESET_HIGH;
        }
        break;

    case SC_RESET_HIGH:
        /* Check responce with reset high --------------------------------------*/
//      SC_Reset(Bit_SET); /* Reset High */
        SC_Reset_HIGH();
        while(length--)
        {
            if((USART_ByteReceive(&Data, SC_RECEIVE_TIMEOUT)) == SUCCESS)
            {
                *card++ = Data; /* Receive data for timeout = SC_RECEIVE_TIMEOUT */
            }
        }
        if(card[0])
        {
            (*SCState) = SC_ACTIVE;
        }
        else
        {
            (*SCState) = SC_POWER_OFF;
        }
        break;

    case SC_ACTIVE:
        break;

    case SC_POWER_OFF:
        /* Close Connection if no answer received ------------------------------*/
//      SC_Reset(Bit_SET); /* Reset high - a bit is used as level shifter from 3.3 to 5 V */
        SC_Reset_HIGH();
//        SC_PowerCmd(DISABLE);
        break;

    default:
        (*SCState) = SC_RESET_LOW;
    }
}

/**
  * @brief  Decodes the Answer to reset received from card.
  * @param  card: pointer to the buffer containing the card ATR.
  * @retval None
  */
static uint8_t SC_decode_Answer2reset(uint8_t *card)
{
    uint32_t i = 0, flagA = 0, flagB = 0 ,flagC = 0 ,flagD = 0 ,buf = 0, protocol = 0;
    uint8_t num = 0;
    uint8_t  guardtime = 0;
    uint8_t  T[4] = {0x0};

    SC_A2R.TS = card[0];  /* Initial character */
    SC_A2R.T0 = card[1];  /* Format character */

    SC_A2R.Hlength = SC_A2R.T0 & (uint8_t)0x0F;  //历史字节个数
//取SC_A2R.T数据
    for (i = 0; i < 4; i++)
    {
        SC_A2R.Tlength = SC_A2R.Tlength + (((SC_A2R.T0 & (uint8_t)0xF0) >> (4 + i)) & (uint8_t)0x1);
    }

    for (i = 0; i < SC_A2R.Tlength; i++)
    {
        SC_A2R.T[i] = card[i + 2];
    }

    if ((SC_A2R.T0 & (uint8_t)0x10) == 0x10)  //TA(1)存在
    {
        flagA = 1;
        T[0] =  SC_A2R.T[num++];
    }
    else   if ((SC_A2R.T0 & (uint8_t)0x20) == 0x20)  //TB(1)存在
    {
        flagB = 1;
        T[1] =  SC_A2R.T[num++];
    }
    if ((SC_A2R.T0 & (uint8_t)0x40) == 0x40)  //TC(1)存在
    {
        flagC = 1;
        T[2] =  SC_A2R.T[num++];
    }
    if ((SC_A2R.T0 & (uint8_t)0x80) == 0x80)  //TD(1)存在
    {
        flagD = 1;
        T[3] =  SC_A2R.T[num++];
    }

    if (flagA == 0x1) {
        ;    //TA(1)存在
    }
    else  {
        ;
    }
    if (flagB == 0x1) {
        ;    //TB(1)存在
    }
    else  {
        ;
    }
    if (flagC == 0x1)//TC(1)存在
    {
        guardtime = T[2] ; //取保护时间值
        USART_SetGuardTime(SC_USART, guardtime);   //保护时间
    }
    else
    {
        USART_SetGuardTime(SC_USART, 0x2);   //保护时间=2
    }

    if (flagD == 0x1)  //如果TD(1)存在，则才检测参数T的定义 T=0、T=1
    {
        protocol = T[3] & (uint8_t)0x0F;
    }
    else
    {
        protocol = 0x0;    //默认T=0
    }


    while (flagD)
    {
        if ((SC_A2R.T[SC_A2R.Tlength - 1] & (uint8_t)0x80) == 0x80)
        {
            flagD = 1;
        }
        else
        {
            flagD = 0;
        }

        buf = SC_A2R.Tlength;
        SC_A2R.Tlength = 0;

        for (i = 0; i < 4; i++)
        {
            SC_A2R.Tlength = SC_A2R.Tlength + (((SC_A2R.T[buf - 1] & (uint8_t)0xF0) >> (4 + i)) & (uint8_t)0x1);
        }

        for (i = 0; i < SC_A2R.Tlength; i++)
        {
            SC_A2R.T[buf + i] = card[i + 2 + buf];
        }
        SC_A2R.Tlength += (uint8_t)buf;
    }

    for (i = 0; i < SC_A2R.Hlength; i++)
    {
        SC_A2R.H[i] = card[i + 2 + SC_A2R.Tlength];
    }

    return (uint8_t)protocol;
}

/**
  * @brief  Initializes all peripheral used for Smartcard interface.
  * @param  None
  * @retval None
  */
static void SC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable GPIO clocks */
//  RCC_APB2PeriphClockCmd(SC_PIN_3_5V_GPIO_CLK | SC_PIN_RESET_GPIO_CLK |
//                         SC_PIN_CMDVCC_GPIO_CLK | SC_USART_GPIO_CLK |
//                         RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(SC_PIN_RESET_GPIO_CLK |SC_USART_GPIO_CLK |
                           RCC_APB2Periph_AFIO, ENABLE);
    /* Enable USART clock */
    RCC_APB2PeriphClockCmd(SC_USART_CLK, ENABLE);

    /* Enable the USART3 Pins Software Full Remapping */
//    GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);

    /* Configure USART CK pin as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = SC_USART_PIN_CK;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(SC_USART_GPIO, &GPIO_InitStructure);

    /* Configure USART Tx pin as alternate function open-drain */
    GPIO_InitStructure.GPIO_Pin = SC_USART_PIN_TX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(SC_USART_GPIO, &GPIO_InitStructure);

    /* Configure Smartcard Reset pin */
    GPIO_InitStructure.GPIO_Pin = SC_PIN_RESET;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(SC_PIN_RESET_GPIO, &GPIO_InitStructure);

    /* Configure Smartcard 3/5V pin */
//  GPIO_InitStructure.GPIO_Pin = SC_PIN_3_5V;
//  GPIO_Init(SC_PIN_3_5V_GPIO, &GPIO_InitStructure);

    /* Configure Smartcard ISO7816_SEL  pin */
    GPIO_InitStructure.GPIO_Pin = SC_PIN_7816;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(SC_PIN_7816_GPIO, &GPIO_InitStructure);

    /* Configure Smartcard CMDVCC pin */
//    GPIO_InitStructure.GPIO_Pin = SC_PIN_CMDVCC;
//    GPIO_Init(SC_PIN_CMDVCC_GPIO, &GPIO_InitStructure);

    /* Enable USART IRQ */
    NVIC_InitStructure.NVIC_IRQChannel = SC_USART_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 13;
    NVIC_Init(&NVIC_InitStructure);

    /* SC_USART configuration ----------------------------------------------------*/
    /* SC_USART configured as follow:
          - Word Length = 9 Bits
          - 0.5 Stop Bit
          - Even parity
          - BaudRate = 9600 baud
          - Hardware flow control disabled (RTS and CTS signals)
          - Tx and Rx enabled
          - USART Clock enabled
    */

    /* USART Clock set to 3.6 MHz (PCLK1 (36 MHZ) / 10) */
    USART_SetPrescaler(SC_USART, 0x05);

    /* USART Guard Time set to 16 Bit */
    USART_SetGuardTime(SC_USART, 16);

    USART_ClockInitStructure.USART_Clock = USART_Clock_Enable;
    USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
    USART_ClockInitStructure.USART_CPHA = USART_CPHA_1Edge;
    USART_ClockInitStructure.USART_LastBit = USART_LastBit_Enable;
    USART_ClockInit(SC_USART, &USART_ClockInitStructure);

//  USART_InitStructure.USART_BaudRate = 9677;
    USART_InitStructure.USART_BaudRate = 19200;//9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(SC_USART, &USART_InitStructure);

    /* Enable the SC_USART Parity Error Interrupt */
    USART_ITConfig(SC_USART, USART_IT_PE, ENABLE);

    /* Enable the SC_USART Framing Error Interrupt */
    USART_ITConfig(SC_USART, USART_IT_ERR, ENABLE);

    /* Enable SC_USART */
    USART_Cmd(SC_USART, ENABLE);

    /* Enable the NACK Transmission */
    USART_SmartCardNACKCmd(SC_USART, ENABLE);

    /* Enable the Smartcard Interface */
    USART_SmartCardCmd(SC_USART, ENABLE);

    /* Select Inside Card*/
//    SC_SEL_LOW();

    /* Set RSTIN HIGH */
//  SC_Reset(Bit_SET);
    SC_Reset_HIGH();

//  SC_VoltageConfig(SC_VOLTAGE_5V);

    /* Disable CMDVCC */
//    SC_PowerCmd(DISABLE);
}

/**
  * @brief  Deinitializes all ressources used by the Smartcard interface.
  * @param  None
  * @retval None
  */
static void SC_DeInit(void)
{
    /* Disable CMDVCC */
//    SC_PowerCmd(ENABLE);

    /* Deinitializes the SC_USART */
    USART_DeInit(SC_USART);

    /* Deinitializes the SC_PIN_SEL_GPIO */
//    GPIO_DeInit(SC_PIN_SEL_GPIO);

    /* Deinitializes the SC_PIN_RESET_GPIO */
    GPIO_DeInit(SC_PIN_RESET_GPIO);

    /* Deinitializes the SC_PIN_CMDVCC_GPIO */
//    GPIO_DeInit(SC_PIN_CMDVCC_GPIO);

    /* Disable GPIO clocks */
    RCC_APB2PeriphClockCmd(SC_PIN_RESET_GPIO_CLK |SC_USART_GPIO_CLK |
                           RCC_APB2Periph_AFIO, DISABLE);

    /* Disable SC_USART clock */
    RCC_APB2PeriphClockCmd(SC_USART_CLK, DISABLE);
}

/**
  * @brief  Configures the card power voltage.
  * @param  SC_Voltage: specifies the card power voltage.
  *   This parameter can be one of the following values:
  *     @arg SC_VOLTAGE_5V: 5V cards.
  *     @arg SC_VOLTAGE_3V: 3V cards.
  * @retval None
  */
//static void SC_VoltageConfig(uint32_t SC_Voltage)
//{
//  if(SC_Voltage == SC_VOLTAGE_5V)
//  {
//    /* Select Smartcard 5V */
//    GPIO_SetBits(SC_PIN_3_5V_GPIO, SC_PIN_3_5V);
//  }
//  else
//  {
//    /* Select Smartcard 3V */
//    GPIO_ResetBits(SC_PIN_3_5V_GPIO, SC_PIN_3_5V);
//  }
//}

/**
  * @brief  Configures Hardware resources used for Samrtcard detection
  * @param  None
  * @retval None
  */
//void SC_DetectPinConfig(void)
//{
//    EXTI_InitTypeDef EXTI_InitStructure;
//    GPIO_InitTypeDef GPIO_InitStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;

//    /* Enable SC_PIN_OFF_GPIO clock and SC_PIN_SEL_GPIO clock */
//    RCC_APB2PeriphClockCmd(SC_PIN_SEL_GPIO_CLK | SC_PIN_OFF_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

//    /* Configure Smartcard OFF pin */
//    GPIO_InitStructure.GPIO_Pin = SC_PIN_OFF;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//    GPIO_Init(SC_PIN_OFF_GPIO, &GPIO_InitStructure);

//    /* Configure ISO7816 SEL pin */
//    GPIO_InitStructure.GPIO_Pin = SC_PIN_SEL;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//    GPIO_Init(SC_PIN_SEL_GPIO, &GPIO_InitStructure);

//    /* Configure EXTI line connected to Smartcard OFF Pin */
//    GPIO_EXTILineConfig(SC_DETECT_PIN, SC_DETECT_GPIO);

//    EXTI_ClearITPendingBit(SC_DETECT_EXTI);

//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
////    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	 //csz
//    EXTI_InitStructure.EXTI_Line = SC_DETECT_EXTI;
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&EXTI_InitStructure);

//    /* Clear the SC_DETECT_EXTI_IRQ Pending Bit */
//    NVIC_ClearPendingIRQ(SC_DETECT_IRQ);

//    NVIC_InitStructure.NVIC_IRQChannel = SC_DETECT_IRQ;
////    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 12;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);

//    CardInserted = 0;
//}

/**
  * @brief  Detects whether the Smartcard is present or not.
  * @param  None.
  * @retval 0 - Smartcard inserted
  *         1 - Smartcard not inserted
  */
//static uint8_t SC_Detect(void)
//{
//  return GPIO_ReadInputDataBit(SC_PIN_OFF_GPIO, SC_PIN_OFF);
//}

/**
  * @brief  Receives a new data while the time out not elapsed.
  * @param  None
  * @retval An ErrorStatus enumuration value:
  *          - SUCCESS: New data has been received
  *          - ERROR: time out was elapsed and no further data is received
  */
static ErrorStatus USART_ByteReceive(uint8_t *Data, uint32_t TimeOut)
{
    uint32_t Counter = 0;

    while((USART_GetFlagStatus(SC_USART, USART_FLAG_RXNE) == RESET) && (Counter != TimeOut))
    {
        Counter++;
    }

	USART_ClearFlag(SC_USART,USART_FLAG_RXNE);
	
    if(Counter != TimeOut)
    {
        *Data = (uint8_t)USART_ReceiveData(SC_USART);
        return SUCCESS;
    }
    else
    {
        return ERROR;
    }
}
/**
  * @brief  reset cpu card.
  * @param  CardType:=0 select psam card ;=1:select user card
            data_buffer:复位响应返回的字符串
  * @retval An ErrorStatus enumuration value:
  *          - SUCCESS: New data has been received
  *          - ERROR: time out was elapsed and no further data is received
  */
uint8_t cpu_reset(uint8_t CardType,uint8_t *rData)
{
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;
    uint8_t i ;

    SCState[CardType]  = SC_POWER_ON ;

//    if (CardType == SC_PSAM_CARD)
//    {
//        SC_SEL_LOW();
//    }
//    else
//    {
//        /* Power ON the card */
//        SC_PowerCmd(ENABLE);
//        __NOP();
//        __NOP();
//        SC_SEL_HIGH();
//    }
//    SC_Reset(Bit_RESET);
    SC_Reset_LOW();
    /* Wait A2R --------------------------------------------------------------*/
    SC_ADPU.Header.CLA = 0x00;
    SC_ADPU.Header.INS = SC_GET_A2R;
    SC_ADPU.Header.P1 = 0x00;
    SC_ADPU.Header.P2 = 0x00;
    SC_ADPU.Body.LC = 0x00;

    while((SCState[CardType] != SC_ACTIVE_ON_T0)&&(SCState[CardType] != SC_POWER_OFF))
    {
        SC_Handler(CardType,&SCState[CardType],&SC_ADPU,&SC_Responce);
    }
    if (SCState[CardType]== SC_POWER_OFF) return 0x0;
    else if (SCState[CardType]== SC_ACTIVE_ON_T0)
    {
        for (i = 0; i < ( SC_A2R.Tlength + SC_A2R.Hlength + 2 ); i++)
        {
            rData[i] = SC_ATR_Table[i] ;
        }

        /* Apply the Procedure Type Selection (PTS) */

        SC_PTSConfig();

        return 0x1;
    }
    else return 0x0;
}
/**
  * @brief  get ramdom  //产生随机数
  * @param  CardType:=0 select psam card ;=1:select user card
            rData:ramdom data
  * @retval An ErrorStatus enumuration value:
  *          - SUCCESS: New data has been received
             = 0x0;读卡失败，卡处于SC_POWER_OFF状态，需要重新复位卡操作
             = 0x9000:读卡成功
  *          - ERROR: time out was elapsed and no further data is received
  */
uint16_t cpu_GetRamdom(uint8_t CardType ,uint8_t RndCount,uint8_t *rData)
{
    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x00;
    SC_ADPU.Header.INS = 0x84;
    SC_ADPU.Header.P1  = 0x00;
    SC_ADPU.Header.P2  = 0x00;
    SC_ADPU.Body.LC    = 0x00;
    SC_ADPU.Body.LE    = RndCount ;

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;

        if( tmp == SC_OP_TERMINATED)
        {
            for(i = 0; i < SC_ADPU.Body.LE; i++)
            {
                rData[i] =  SC_Responce.Data[i] ;
            }
        }

        return tmp ;
    }
}
//按文件标识符选择
uint16_t cpu_SelectFile(uint8_t CardType ,uint8_t Type,uint16_t FID,uint8_t *RevData)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x00;
    SC_ADPU.Header.INS = 0xA4;
    SC_ADPU.Header.P1  = Type;        //按文件标识符选择DF
    SC_ADPU.Header.P2  = 0x00;
    SC_ADPU.Body.LC    = 0x02;

    SC_ADPU.Body.Data[0]    = FID/256;      //文件标识符
    SC_ADPU.Body.Data[1]    = FID%256;

    SC_ADPU.Body.LE    = 0x0;

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        if((tmp/256) == 0x61)
        {
            tmp = cpu_GetResponse(CardType,tmp%256,RevData) ;
        }
        return tmp ;
    }
}



/**************************************************************************************************
*       name:cpu_SelectFileByName
*description:按文件名称选择目录文件
*       InPut:CardType卡类型
*            :FID文件标识
*			 :rDataLen接收数据长度
*			 :接收数据指针
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_SelectFileByName(uint8_t CardType,uint8_t paraP2,uint8_t NameLen,uint8_t * FileName)
{
//    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x00;
    SC_ADPU.Header.INS = 0xA4;
    SC_ADPU.Header.P1  = 0x04;        //按文件标识符选择DF
    SC_ADPU.Header.P2  = paraP2;			//P2=00,选择第一个符合条件的目录文件
    //P2=02,选择下一个符合条件的目录文件
    SC_ADPU.Body.LC    = NameLen;
//  SC_ADPU.Body.LE    = ;						//期望返回的数据长度

    memcpy(SC_ADPU.Body.Data,FileName,NameLen);//文件名称

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;

        return tmp ;
    }
}
/**************************************************************************************************
*       name:cpu_AppendRecord
*description:追加记录
*			 InPut:CardType卡类型
*						:FID文件标识
*						:DataLenDataLen发送数据长度
*						:发送数据指针
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_AppendRecord(uint8_t CardType,uint8_t Fid,uint8_t DataLen,uint8_t *tData)
{
    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x00;
    SC_ADPU.Header.INS = 0xE2;
    SC_ADPU.Header.P1  = 0x00;        //按文件标识符选择DF

    i = (Fid&0xF8)>>3 ;
    if((i!= 0x00)&&(i!=0x1F))					//高5位不为‘00000’和‘11111’时，高5位表示要操作文件的短文
    {   //件标识符，如果短文件标识符的最低位为‘1’，则P2为X8h，否则为X0h
        if(i&0x01)SC_ADPU.Header.P2 = (Fid&0xF0)|0x08;

        else
        {
            SC_ADPU.Header.P2 = (Fid&0xF0);
        }
    }
    else
    {
        SC_ADPU.Header.P2  = 0x00;
    }
    SC_ADPU.Body.LC    = DataLen;			//要发送的数据长度
//  SC_ADPU.Body.LE    = ;						//不存在

    memcpy(SC_ADPU.Body.Data,tData,DataLen);//要发送的数据

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        return tmp ;
    }
}
/**************************************************************************************************
*       name:cpu_ApplicationBlock
*description:应用锁定
*			 InPut:CardType卡类型
*						:BlockType：0：临时锁定，1：永久锁定
*						:Mac:4字节的报文鉴别代码(MAC)数据元，由应用维护密钥生成
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_ApplicationBlock(uint8_t CardType,uint8_t BlockType,uint8_t *Mac)
{
//    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x84;
    SC_ADPU.Header.INS = 0x1E;
    SC_ADPU.Header.P1  = 0x00;        //按文件标识符选择DF

    if(BlockType>1)return 0x0;				//参数错误
    SC_ADPU.Header.P2  = BlockType;

    SC_ADPU.Body.LC    = 4;			//要发送的数据长度
//  SC_ADPU.Body.LE    = ;			//不存在

    memcpy(SC_ADPU.Body.Data,Mac,4);//要发送的数据

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        return tmp ;
    }
}
/**************************************************************************************************
*       name:cpu_AppUnblock
*description:应用解锁
*			 InPut:CardType卡类型
*						:Mac:4字节的报文鉴别代码(MAC)数据元，由应用维护密钥生成,初始值为4字节的随机数+00 00 00 00
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_AppUnblock(uint8_t CardType,uint8_t * Mac)
{
//    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x84;
    SC_ADPU.Header.INS = 0x18;
    SC_ADPU.Header.P1  = 0x00;        //按文件标识符选择DF
    SC_ADPU.Header.P2  = 0x00;

    SC_ADPU.Body.LC    = 4;			//要发送的数据长度
//  SC_ADPU.Body.LE    = ;			//不存在

    memcpy(SC_ADPU.Body.Data,Mac,4);//要发送的数据

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        return tmp ;
    }
}
/**************************************************************************************************
*       name:cpu_ChangePin
*description:修改PIN
*			 InPut:CardType卡类型
*						:PinId 口令ID
*						：OldPinLen 旧密码长度
*						：OldPin    旧密码
*						：NewPinLen 新密码长度
*						：NewPin    新密码
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_ChangePin(uint8_t CardType,uint8_t PinId,uint8_t OldPinLen,uint8_t *OldPin,uint8_t NewPinLen,uint8_t *NewPin)
{
//    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x80;
    SC_ADPU.Header.INS = 0x5E;
    SC_ADPU.Header.P1  = 0x01;        //按文件标识符选择DF
    SC_ADPU.Header.P2  = PinId;

    if((OldPinLen>8)||(OldPinLen<2))return 0x0;
    if((NewPinLen>8)||(NewPinLen<2))return 0x0;
    SC_ADPU.Body.LC = NewPinLen + OldPinLen + 1;			//要发送的数据长度
//  SC_ADPU.Body.LE    = ;			//不存在

    memcpy(SC_ADPU.Body.Data,OldPin,OldPinLen);//旧密码
    SC_ADPU.Body.Data[OldPinLen]=0xFF;
    memcpy((uint8_t *)&SC_ADPU.Body.Data[OldPinLen+1],NewPin,NewPinLen);//新密码

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_DebitForPurchase
*description：消费
*			 InPut：CardType卡类型
*						：tdata 消费数据结构
*						：rdata
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_DebitForPurchase(uint8_t CardType,TRANDATA tdata,RESPTRANDATA *rdata)
{
//    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x80;
    SC_ADPU.Header.INS = 0x54;
    SC_ADPU.Header.P1  = 0x01;        //按文件标识符选择DF
    SC_ADPU.Header.P2  = 0x00;
    SC_ADPU.Body.LC    = sizeof(TRANDATA);			//要发送的数据长度
    SC_ADPU.Body.LE    = sizeof(RESPTRANDATA);

    memcpy(SC_ADPU.Body.Data,(uint8_t *)&tdata,sizeof(TRANDATA));

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        memcpy((uint8_t *)rdata,SC_Responce.Data,SC_ADPU.Body.LE);

        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_CreditForLoad
*description：圈存
*			 InPut：CardType卡类型
*						：tdata 圈存数据 和 消费共用一个结构，只是不用消费数据在中的 交易序号
*			output：TAC
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_CreditForLoad(uint8_t CardType,TRANDATA tdata,uint8_t *tac)
{
//    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x80;
    SC_ADPU.Header.INS = 0x52;
    SC_ADPU.Header.P1  = 0x00;        //按文件标识符选择DF
    SC_ADPU.Header.P2  = 0x00;
    SC_ADPU.Body.LC    = sizeof(TRANDATA)-4;			//要发送的数据长度
    SC_ADPU.Body.LE    = 4;

    memcpy(SC_ADPU.Body.Data,(uint8_t *)tdata.TranDate,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        memcpy((uint8_t *)tac,SC_Responce.Data,SC_ADPU.Body.LE);

        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_InitForLoad
*description：圈存初始化
*			 InPut：CardType卡类型
*						: Type  01:ED  02:EP
*						：tdata 圈存数据 和 消费共用一个结构，只是不用消费数据在中的 交易序号
*			output：TAC
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_InitForLoad(uint8_t CardType,uint8_t Type,INITDATA InitData,RESPINITDATA *rInitdata)
{
//    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x80;
    SC_ADPU.Header.INS = 0x50;
    SC_ADPU.Header.P1  = 0x00;        //按文件标识符选择DF
    SC_ADPU.Header.P2  = Type;
    SC_ADPU.Body.LC    = 0x0B;			//要发送的数据长度
    SC_ADPU.Body.LE    = 0x10;

    memcpy(SC_ADPU.Body.Data,(uint8_t *)&InitData,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        memcpy((uint8_t *)rInitdata,SC_Responce.Data,SC_ADPU.Body.LE);

        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_DebitForUnload
*description：外部认证
*			 InPut：CardType卡类型
*						: KeyId 密钥标识符
*						：sDatalen 随机数长度
*           ：sRomd 随机数
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_ExteAuth(uint8_t CardType,uint8_t KeyId,uint8_t sDatalen,uint8_t *sRomd)
{
//    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x00;
    SC_ADPU.Header.INS = 0x82;
    SC_ADPU.Header.P1  = 0x00;        //按文件标识符选择DF
    SC_ADPU.Header.P2  = KeyId;
    SC_ADPU.Body.LC    = sDatalen;			//要发送的数据长度
//	  SC_ADPU.Body.LE    = 4;

    memcpy(SC_ADPU.Body.Data,sRomd,sDatalen);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_GetBalance
*description：读余额
*			 InPut：CardType卡类型
*			output: sum 余额
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_GetBalance(uint8_t CardType,uint8_t Type,uint8_t *sum)
{
//    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x80;
    SC_ADPU.Header.INS = 0x5C;
    SC_ADPU.Header.P1  = 0x00;        //按文件标识符选择DF
    SC_ADPU.Header.P2  = Type;
    SC_ADPU.Body.LC    = 0;			//要发送的数据长度
    SC_ADPU.Body.LE    = 4;

//		memcpy(SC_ADPU.Body.Data,sRomd,sDatalen);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        memcpy(sum,SC_Responce.Data,SC_ADPU.Body.LE);
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_GetTranProve
*description：取交易认证
*			 InPut：CardType卡类型
*   				: TranType 交易类型
*           ：TranNo   交易号
*			output: RespTranData TAC + MAC2
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_GetTranProve(uint8_t CardType,uint8_t TranType,uint16_t TranNo,RESPTRANDATA * RespTranData)
{
//    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x80;
    SC_ADPU.Header.INS = 0x5A;
    SC_ADPU.Header.P1  = 0x00;        //按文件标识符选择DF
    SC_ADPU.Header.P2  = TranType;
    SC_ADPU.Body.LC    = 0x02;			//要发送的数据长度
    SC_ADPU.Body.LE    = 8;

    memcpy(SC_ADPU.Body.Data,(uint8_t *)&TranNo,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        memcpy((uint8_t *)RespTranData,SC_Responce.Data,SC_ADPU.Body.LE);
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_InitForPurchase
*description：初始化消费
*			 InPut：CardType卡类型
*   				: Type 01：ED  02：EP
*           ：InitData   初始化数据
*			output: RespInitData TAC + MAC2
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_InitForPurchase(uint8_t CardType,uint8_t Type,INITDATA InitData,RESPINITDATA *RespInitData)
{
//    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x80;
    SC_ADPU.Header.INS = 0x50;
    SC_ADPU.Header.P1  = 0x01;        //按文件标识符选择DF
    SC_ADPU.Header.P2  = Type;
    SC_ADPU.Body.LC    = 0x0B;			//要发送的数据长度
    SC_ADPU.Body.LE    = 0x0F;

    memcpy(SC_ADPU.Body.Data,(uint8_t *)&InitData,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        memcpy((uint8_t *)RespInitData,SC_Responce.Data,SC_ADPU.Body.LE);
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_InitForPurchase
*description：初始化圈提
*			 InPut：CardType卡类型
*   				: Type 01：ED  02：EP
*           ：InitData   初始化数据
*			output: RespInitData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_InitForUnload(uint8_t CardType,INITDATA  InitData,RESPINITDATA * RespInitData)
{
//    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x80;
    SC_ADPU.Header.INS = 0x50;
    SC_ADPU.Header.P1  = 0x05;        //按文件标识符选择DF
    SC_ADPU.Header.P2  = 0x01;
    SC_ADPU.Body.LC    = 0x0B;			//要发送的数据长度
    SC_ADPU.Body.LE    = 0x10;

    memcpy(SC_ADPU.Body.Data,(uint8_t *)&InitData,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        memcpy((uint8_t *)RespInitData,SC_Responce.Data,SC_ADPU.Body.LE);
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_DebitForUnload
*description：圈提
*			 InPut：CardType卡类型
*   				: Type 01：ED  02：EP
*           ：InitData   初始化数据
*			output: RespInitData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_DebitForUnload(uint8_t CardType,TRANDATA TranData,RESPTRANDATA * RespTranData)
{
//    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x80;
    SC_ADPU.Header.INS = 0x54;
    SC_ADPU.Header.P1  = 0x03;        //按文件标识符选择DF
    SC_ADPU.Header.P2  = 0x00;
    SC_ADPU.Body.LC    = 0x0B;			//要发送的数据长度
    SC_ADPU.Body.LE    = 0x04;

    memcpy(SC_ADPU.Body.Data,(uint8_t *)&TranData,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        memcpy((uint8_t *)RespTranData,SC_Responce.Data,SC_ADPU.Body.LE);
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_InitForPurchase
*description：内部认证
*			 InPut：CardType卡类型
*   				: Type 01：ED  02：EP
*           ：InitData   初始化数据
*			output: RespInitData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_InteAuth(uint8_t CardType,uint8_t KeyID,uint8_t sDataLen,uint8_t * sData,uint8_t * rDataLen,uint8_t * rData)
{
//    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x00;
    SC_ADPU.Header.INS = 0x88;
    SC_ADPU.Header.P1  = 0x00;
    SC_ADPU.Header.P2  = KeyID;    //密钥标识符
    SC_ADPU.Body.LC    = sDataLen;			//要发送的数据长度
//	  SC_ADPU.Body.LE    = 4;

    memcpy(SC_ADPU.Body.Data,sData,sDataLen);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        if(tmp == 0x9000)
        {
			//////////返回DES运算结果 和数据长度
        }
		else if((tmp&0xFF00)==0x6100)
		{
			tmp = cpu_GetResponse(CardType,tmp&0x00FF,rData);
		}
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_ReadBin
*description：读二进制文件
*			 InPut：CardType卡类型
*   				: Type 01：ED  02：EP
*           ：InitData   初始化数据
*			output: RespInitData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_ReadBin(uint8_t CardType,uint16_t Offset,uint8_t ReadLen,uint8_t * rData)
{
//    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x00;
    SC_ADPU.Header.INS = 0xB0;
    SC_ADPU.Header.P1  = (Offset/256);
    SC_ADPU.Header.P2  = (Offset%256);    //密钥标识符
    SC_ADPU.Body.LC    = 0;			//要发送的数据长度
    SC_ADPU.Body.LE    = ReadLen;

//		memcpy(SC_ADPU.Body.Data,sData,sDatalen);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        if(tmp == 0x9000)
        {
            memcpy(rData,SC_Responce.Data,ReadLen);
        }
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_ReadBin
*description：写二进制文件
*			 InPut：CardType卡类型
*           ：InitData   初始化数据
*			output: RespInitData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_WriteBin(uint8_t CardType,uint8_t P1,uint8_t P2,uint8_t WriteLen,uint8_t * sData,uint8_t WriteMode)
{
//    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    if((WriteMode==0)||(WriteMode==2))
        SC_ADPU.Header.CLA = 0x00;
    else
        SC_ADPU.Header.CLA = 0x04;

    SC_ADPU.Header.INS = 0xD6;
    SC_ADPU.Header.P1  = P1;
    SC_ADPU.Header.P2  = P2;    //密钥标识符
    SC_ADPU.Body.LC    = WriteLen;			//要发送的数据长度
//  SC_ADPU.Body.LE    = ReadLen;

    memcpy(SC_ADPU.Body.Data,sData,WriteLen);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_GetResponse
*description：读取后续数据
*	   InPut：CardType卡类型
*   	    : RLen 接收数据长度
*     output: rData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_GetResponse(uint8_t CardType,uint8_t RLen,uint8_t *rData)
{
//    uint8_t i = 0 ;
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x00;
    SC_ADPU.Header.INS = 0xC0;
    SC_ADPU.Header.P1  = 0x00;
    SC_ADPU.Header.P2  = 0x00;    //密钥标识符
    SC_ADPU.Body.LC    = 0;			//要发送的数据长度
    SC_ADPU.Body.LE    = RLen;

//		memcpy(SC_ADPU.Body.Data,sData,sDatalen);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        if(tmp == 0x9000)
        {
            memcpy(rData,SC_Responce.Data,RLen);
        }
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_VerifyPin
*description：校验个人PIN
*	   InPut：CardType卡类型
*   	    : RLen 接收数据长度
*     output: rData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_VerifyPin(uint8_t CardType,uint8_t KeyID,uint8_t PinLen,uint8_t * Pin)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x00;
    SC_ADPU.Header.INS = 0x20;
    SC_ADPU.Header.P1  = 0x00;
    SC_ADPU.Header.P2  = KeyID;    //密钥标识符
    SC_ADPU.Body.LC    = PinLen;			//要发送的数据长度
    SC_ADPU.Body.LE    = 0;

    memcpy(SC_ADPU.Body.Data,Pin,PinLen);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_GetResponse
*description：取灰锁状态
*	   InPut：CardType卡类型
*   	    : RLen 接收数据长度
*     output: rData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_LockProof(uint8_t CardType,uint8_t ProofType,LOCKPROOFDATA *LockProofData)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0xE0;
    SC_ADPU.Header.INS = 0xCA;
    SC_ADPU.Header.P1  = ProofType;
    SC_ADPU.Header.P2  = 0x00;
    SC_ADPU.Body.LC    = 0;			//要发送的数据长度

    if(ProofType == 0)
    {
        SC_ADPU.Body.LE    = 0x1E;
    }
    else if(ProofType == 1)
    {
        SC_ADPU.Body.LE    = 0;
    }

//	memcpy(SC_ADPU.Body.Data,Pin,PinLen);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        if((tmp==0x9000)&&(ProofType == 0))
        {
            memcpy((uint8_t *)LockProofData,SC_Responce.Data,SC_ADPU.Body.LE);
        }
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_GetResponse
*description：联机解扣初始化
*	   InPut：CardType卡类型
*   	    : RLen 接收数据长度
*     output: rData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_InitGreyUnLock(uint8_t CardType,uint8_t Type,GREYINITDATA GreyInitData,UNLOCKINITRESPDATA *UnLockInitRespData)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0xE0;
    SC_ADPU.Header.INS = 0x7A;
    SC_ADPU.Header.P1  = 0x09;
    SC_ADPU.Header.P2  = Type;
    SC_ADPU.Body.LC    = 0x07;			//要发送的数据长度
    SC_ADPU.Body.LE    = 0x12;

    memcpy(SC_ADPU.Body.Data,(uint8_t *)&GreyInitData,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        if((tmp/256)==0x61)
        {
            tmp = cpu_GetResponse(CardType,tmp%256,(uint8_t *)UnLockInitRespData) ;   //初始化联机解扣返回数据
        }
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_GetResponse
*description：联机解扣
*	   InPut：CardType卡类型
*   	    : RLen 接收数据长度
*     output: rData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_Grey_UnLock(uint8_t CardType,UNLOCKTRANDATA UnLockTranData,uint8_t *UnLockRespData)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0xE0;
    SC_ADPU.Header.INS = 0x7E;
    SC_ADPU.Header.P1  = 0x09;
    SC_ADPU.Header.P2  = 0x00;
    SC_ADPU.Body.LC    = 0x0F;			//要发送的数据长度
    SC_ADPU.Body.LE    = 0x04;

    memcpy(SC_ADPU.Body.Data,(uint8_t *)&UnLockTranData,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        if(tmp==0x9000)memcpy((uint8_t *)UnLockRespData,SC_Responce.Data,SC_ADPU.Body.LE);
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_GetResponse
*description：脱机解扣
*	   InPut：CardType卡类型
*   	    : RLen 接收数据长度
*     output: rData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_DebitForUnLock(uint8_t CardType,uint8_t Type, OFFUNLOCKTRANDATA OffUnLockTranData,uint8_t *Tac)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0xE0;
    SC_ADPU.Header.INS = 0x7E;
    SC_ADPU.Header.P1  = 0x08;
    SC_ADPU.Header.P2  = Type;
    SC_ADPU.Body.LC    = 0x1B;			//要发送的数据长度
    SC_ADPU.Body.LE    = 0x04;

    memcpy(SC_ADPU.Body.Data,(uint8_t *)&OffUnLockTranData,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        if(0x61 == (tmp/256))
        {
            tmp = cpu_GetResponse(CardType,tmp%256,Tac) ;   //返回TAC
        }
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_InitGreyLock
*description：灰锁初始化
*	   InPut：CardType卡类型
*   	    : Type 1,ED电子存折 02，EP电子钱包
*GREYINITDATA：灰锁初始化结构，
*     output: GREYRESPDATA 灰锁初始化响应报文
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_InitGreyLock(uint8_t CardType, uint8_t Type,GREYINITDATA GreyInitData, GREYRESPDATA *GreyRespData)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0xE0;
    SC_ADPU.Header.INS = 0x7A;
    SC_ADPU.Header.P1  = 0x08;
    SC_ADPU.Header.P2  = Type;
    SC_ADPU.Body.LC    = 0x07;			//要发送的数据长度
//	SC_ADPU.Body.LE    = 0x0F;

    memcpy(SC_ADPU.Body.Data,(uint8_t *)&GreyInitData,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        if((tmp/256)==0x61)
        {
            tmp = cpu_GetResponse(CardType,tmp%256,(uint8_t *)GreyRespData) ;
        }
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_GreyLock
*description：灰锁
*	   InPut：CardType卡类型
*   	    : RLen 接收数据长度
*     output: rData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_GreyLock(uint8_t CardType,GREYTRANDATA GreyTranData,RESPTRANDATA *RespTranData)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0xE0;
    SC_ADPU.Header.INS = 0x7C;
    SC_ADPU.Header.P1  = 0x08;
    SC_ADPU.Header.P2  = 0x00;
    SC_ADPU.Body.LC    = 0x13;			//要发送的数据长度
    SC_ADPU.Body.LE    = 0x08;

    memcpy(SC_ADPU.Body.Data,(uint8_t *)&GreyTranData,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        if((tmp/256)==0x61)
        {
            tmp = cpu_GetResponse(CardType,tmp%256,(uint8_t *)RespTranData);
        }
        return tmp ;
    }
}
/**************************************************************************************************
*       name：Inter_Auth
*description：内部认证
*	   InPut：CardType卡类型
*   	    : RLen 接收数据长度
*     output: rData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t Inter_Auth(uint8_t CardType,uint8_t KeyVer,uint8_t *str,uint8_t *RespData)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x00;
    SC_ADPU.Header.INS = 0x88;
    SC_ADPU.Header.P1  = 0x00;
    SC_ADPU.Header.P2  = KeyVer;
    SC_ADPU.Body.LC    = 0x8;			//要发送的数据长度
    SC_ADPU.Body.LE    = 0x8;

    memcpy(SC_ADPU.Body.Data,str,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
		return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        if((tmp/256)==0x61)
        {
            tmp = cpu_GetResponse(CardType,tmp%256,(uint8_t *)RespData);
        }
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_Crypt
*description：Crypt 加解密 ，计算MAC   只能用于SAM卡
*	   InPut：CardType卡类型
*   	    : RLen 接收数据长度
*     output: rData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_Crypt(uint8_t CardType,uint8_t CryptMode,uint8_t KeyID,uint8_t sDataLen,uint8_t * sData,uint8_t * rDataLen,uint8_t * rData)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x80;
    SC_ADPU.Header.INS = 0xF8;
    SC_ADPU.Header.P1  = CryptMode;
    SC_ADPU.Header.P2  = KeyID;
    SC_ADPU.Body.LC    = sDataLen;			//要发送的数据长度
	SC_ADPU.Body.LE    = 0x08;

    memcpy(SC_ADPU.Body.Data,sData,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;

        if((tmp/256)==0x61)
        {
            tmp = cpu_GetResponse(CardType,tmp%256,(uint8_t *)rData);
        }
		

//        if(tmp==0x9000)
//        {
//            memcpy((uint8_t *)rData,SC_Responce.Data,SC_ADPU.Body.LE);
//            *rDataLen = SC_ADPU.Body.LC;
//        }
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_ReadRecord
*description：读记录
*	   InPut：CardType卡类型
*   	    : RLen 接收数据长度
*     output: rData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_ReadRecord(uint8_t CardType,uint8_t SID,uint8_t RecordNo,uint8_t ReadLen,uint8_t * rData)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x00;
    SC_ADPU.Header.INS = 0xB2;
    SC_ADPU.Header.P1  = RecordNo;
    SC_ADPU.Header.P2  = (SID<<3)|0x04;
    SC_ADPU.Body.LC    = 0;			//要发送的数据长度
    SC_ADPU.Body.LE    = ReadLen;

//	memcpy(SC_ADPU.Body.Data,(uint8_t *)&sData,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        if(tmp==0x9000)
        {
            memcpy((uint8_t *)rData,SC_Responce.Data,SC_ADPU.Body.LE);
        }
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_UpdateRecord
*description：修改记录
*	   InPut：CardType卡类型
*   	    : RLen 接收数据长度
*     output: rData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_UpdateRecord(uint8_t CardType,uint8_t SID,uint8_t RecordNo,uint8_t WriteLen,uint8_t * sData,uint8_t WriteMode)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    if((WriteMode==0)||(WriteMode==2))
        SC_ADPU.Header.CLA = 0x00;
    else
        SC_ADPU.Header.CLA = 0x04;
    SC_ADPU.Header.INS = 0xDC;
    SC_ADPU.Header.P1  = RecordNo;
    SC_ADPU.Header.P2  = (SID<<3)|0x04;
    SC_ADPU.Body.LC    = WriteLen;			//要发送的数据长度
    SC_ADPU.Body.LE    = 0;

    memcpy(SC_ADPU.Body.Data,sData,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
//		if(tmp==0x9000)
//		{
//			memcpy((uint8_t *)rData,SC_Responce.Data,SC_ADPU.Body.LE);
//		}
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_CardBlock
*description：卡片锁定
*	   InPut：CardType卡类型
*   	    : RLen 接收数据长度
*     output: rData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_Cardlock(uint8_t CardType,uint8_t * sData)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x84;
    SC_ADPU.Header.INS = 0x16;
    SC_ADPU.Header.P1  = 0x00;
    SC_ADPU.Header.P2  = 0x00;
    SC_ADPU.Body.LC    = 0x04;			//要发送的数据长度
    SC_ADPU.Body.LE    = 0;

    memcpy(SC_ADPU.Body.Data,sData,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
//		if(tmp==0x9000)
//		{
//			memcpy((uint8_t *)rData,SC_Responce.Data,SC_ADPU.Body.LE);
//		}
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_CardBlock
*description：通用DES计算（DES Crypt）
*	   InPut：CardType卡类型
*   	    : RLen 接收数据长度
*     output: rData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_PSAM_DesCrypt(uint8_t CardType,uint8_t CryptMode,uint8_t sDataLen,uint8_t * sData,uint8_t *rDataLen,uint8_t * rData)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x80;
    SC_ADPU.Header.INS = 0xFA;
    SC_ADPU.Header.P1  = CryptMode;
    SC_ADPU.Header.P2  = 0x00;
    SC_ADPU.Body.LC    = sDataLen;			//要发送的数据长度
    SC_ADPU.Body.LE    = 0;

    memcpy(SC_ADPU.Body.Data,sData,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        if((tmp/256)==0x61)
        {
            tmp = cpu_GetResponse(CardType,tmp%256,(uint8_t *)rData) ;
        }
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_CardBlock
*description：通用DES计算初始化（DES Crypt）
*	   InPut：CardType卡类型
*   	    : RLen 接收数据长度
*     output: rData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_PSAM_InitForDesCrypt(uint8_t CardType,uint8_t KeyType,uint8_t KeyID,uint8_t sDataLen,uint8_t *sData)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x80;
    SC_ADPU.Header.INS = 0x1A;
    SC_ADPU.Header.P1  = KeyType;
    SC_ADPU.Header.P2  = KeyID;
    SC_ADPU.Body.LC    = sDataLen;			//要发送的数据长度
    SC_ADPU.Body.LE    = 0;

    memcpy(SC_ADPU.Body.Data,sData,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
//		if((tmp/256)==0x61)
//		{
//			tmp = cpu_GetResponse(CardType,tmp%256,(uint8_t *)rData) ;
//		}
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_CardBlock
*description：校验MAC2
*	   InPut：CardType卡类型
*   	    : RLen 接收数据长度
*     output: rData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_PSAM_CreditSamForPurchase(uint8_t CardType,uint8_t * Mac2)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x80;
    SC_ADPU.Header.INS = 0x72;
    SC_ADPU.Header.P1  = 0x0;
    SC_ADPU.Header.P2  = 0x0;
    SC_ADPU.Body.LC    = 0x04;			//要发送的数据长度
    SC_ADPU.Body.LE    = 0;

    memcpy(SC_ADPU.Body.Data,Mac2,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
//		if(tmp==0x9000)
//		{
//			memcpy((uint8_t *)rData,SC_Responce.Data,SC_ADPU.Body.LE);
//		}
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_CardBlock
*description：MAC1计算
*	   InPut：CardType卡类型
*   	    : DivGrade 信息个数
*           ：InitSamPur 初始化结构数据
*     output: RespInitSam 初始化返回结构数据
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_PSAM_InitSamForPurchase(uint8_t CardType,uint8_t DivGrade,INITSAMPUR InitSamPur,SAMMAC1RESDATA *RespInitSam)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0xE0;
    SC_ADPU.Header.INS = 0x40;
    SC_ADPU.Header.P1  = 0x0;
    SC_ADPU.Header.P2  = 0x0;
    SC_ADPU.Body.LC    = 0x14+8*DivGrade;			//要发送的数据长度
    SC_ADPU.Body.LE    = 12;

    memcpy(SC_ADPU.Body.Data,(uint8_t *)&InitSamPur,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        if((tmp/256)==0x61)
        {
            tmp = cpu_GetResponse(CardType,tmp%256,(uint8_t *)RespInitSam) ;
        }
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_Init_Sam_Grey_Lock
*description：MAC1计算	- SAM灰锁初始化
*	   InPut：CardType卡类型
*   	    : DivGrade
*			: InitSamPur SAM初始化结构
*     output: RespInitSam  SAM初始化返回结构
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_Init_Sam_Grey_Lock(uint8_t CardType,uint8_t DivGrade,SAMMAC1INITDATA InitSamPur,RESPINITSAM *RespInitSam)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x80;
    SC_ADPU.Header.INS = 0x70;
    SC_ADPU.Header.P1  = 0x0;
    SC_ADPU.Header.P2  = 0x0;
    SC_ADPU.Body.LC    = 0x14+8*DivGrade;			//要发送的数据长度
    SC_ADPU.Body.LE    = 8;

    memcpy(SC_ADPU.Body.Data,(uint8_t *)&InitSamPur,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        if(tmp==0x9000)
        {
            memcpy((uint8_t *)RespInitSam,SC_Responce.Data,SC_ADPU.Body.LE);
        }
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_CardBlock
*description：验证MAC2
*	   InPut：CardType卡类型
*   	    : RLen 接收数据长度
*     output: rData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_Cert_Sam_Grey_Lock(uint8_t CardType,uint8_t *SamMac2)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0xE0;
    SC_ADPU.Header.INS = 0x42;
    SC_ADPU.Header.P1  = 0x0;
    SC_ADPU.Header.P2  = 0x0;
    SC_ADPU.Body.LC    = 0x04;			//要发送的数据长度
    SC_ADPU.Body.LE    = 0;

    memcpy(SC_ADPU.Body.Data,SamMac2,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
//		if(tmp==0x9000)
//		{
//			memcpy((uint8_t *)RespInitSam,SC_Responce.Data,SC_ADPU.Body.LE);
//		}
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_CardBlock
*description：计算GMAC
*	   InPut：CardType卡类型
*   	    : RLen 接收数据长度
*     output: rData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_Credit_Sam_For_Grey_Debit(uint8_t CardType,SAMGMACINITDATA SamGmacInitData,SAMGMACRESDATA *SamGmacResData)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0xE0;
    SC_ADPU.Header.INS = 0x44;
    SC_ADPU.Header.P1  = 0x0;
    SC_ADPU.Header.P2  = 0x0;
    SC_ADPU.Body.LC    = 0x0F;			//要发送的数据长度
    SC_ADPU.Body.LE    = 8; 

    memcpy(SC_ADPU.Body.Data,(uint8_t *)&SamGmacInitData,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        if((tmp/256)==0x61)
        {
            tmp = cpu_GetResponse(CardType,tmp%256,(uint8_t *)SamGmacResData) ;
        }
        return tmp ;
    }
}
/**************************************************************************************************
*       name：cpu_CardBlock
*description：取GMAC，用于PSAM卡掉电后获取上次计算的GMAC
*	   InPut：CardType卡类型
*   	    : RLen 接收数据长度
*     output: rData
*     返回值：成功返回0x9000 失败返回错误代码
**************************************************************************************************/
uint16_t cpu_SamGetGmac(uint8_t CardType,uint8_t * ResTranNo,SAMGMACRESDATA *SamGmac)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0xE0;
    SC_ADPU.Header.INS = 0x46;
    SC_ADPU.Header.P1  = 0x0;
    SC_ADPU.Header.P2  = 0x0;
    SC_ADPU.Body.LC    = 0x04;			//要发送的数据长度
    SC_ADPU.Body.LE    = 8;

    memcpy(SC_ADPU.Body.Data,ResTranNo,SC_ADPU.Body.LC);

    SC_Handler(CardType,&SCState[CardType], &SC_ADPU, &SC_Responce );

    if (SCState[CardType] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        if((tmp/256)==0x61)
        {
            tmp = cpu_GetResponse(CardType,tmp%256,(uint8_t *)SamGmac) ;
        }
        return tmp ;
    }
}

/**************************************************************************************************
**name：cpu_Adpu
**PSAM通道指令
**************************************************************************************************/
uint16_t cpu_Adpu(uint8_t *inData,uint8_t inLen,uint8_t *outData,uint8_t *outLen)
{
    uint16_t tmp ;
    uint32_t i = 0;
	uint8_t sIndex;
    uint8_t locData = 0;
	uint8_t cmdINS;
    SC_ADPU_Responce SC_Responce;

	sIndex = 0;
	cmdINS = inData[1];
    /* Reset responce buffer ---------------------------------------------------*/
    for(i = 0; i < LC_MAX; i++)
    {
        SC_Responce.Data[i] = 0;
    }

    SC_Responce.SW1 = 0;
    SC_Responce.SW2 = 0;

    /* Enable the DMA Receive (Set DMAR bit only) to enable interrupt generation
       in case of a framing error FE */
//    USART_DMACmd(SC_USART, USART_DMAReq_Rx, ENABLE);

    /* Send header -------------------------------------------------------------*/
	for(i=0;i<5;i++)
	{
		USART_SendData(SC_USART, inData[sIndex++]);
		while(USART_GetFlagStatus(SC_USART, USART_FLAG_TC) == RESET)
		{
		}
	}

    /* Flush the SC_USART DR */
    (void)USART_ReceiveData(SC_USART);

//	USART_ClearFlag(SC_USART,USART_FLAG_RXNE);
//	USART_ClearFlag(SC_USART,USART_FLAG_RXNE);
//	USART_ClearFlag(SC_USART,USART_FLAG_RXNE);
    /* --------------------------------------------------------
      Wait Procedure byte from card:
      1 - ACK
      2 - NULL
      3 - SW1; SW2
     -------------------------------------------------------- */

    if((USART_ByteReceive(&locData, SC_RECEIVE_TIMEOUT*2)) == SUCCESS)
    {
        if(((locData & (uint8_t)0xF0) == 0x60) || ((locData & (uint8_t)0xF0) == 0x90))
        {
            /* SW1 received */
            SC_Responce.SW1 = locData;

            if((USART_ByteReceive(&locData, SC_RECEIVE_TIMEOUT)) == SUCCESS)
            {
                /* SW2 received */
                SC_Responce.SW2 = locData;
            }
        }
        else if (((locData & (uint8_t)0xFE) == (((uint8_t)~(cmdINS)) & \
                                                (uint8_t)0xFE))||((locData & (uint8_t)0xFE) == (cmdINS & (uint8_t)0xFE)))
        {
            SC_Responce.Data[0] = locData;/* ACK received */
        }
    }
	else
	{
		return 0x6700;
	}

    /* If no status bytes received ---------------------------------------------*/
    if(SC_Responce.SW1 == 0x00)
    {
        /* Send body data to SC--------------------------------------------------*/
		while(sIndex < inLen)//还有数据没有发送完
		{
			USART_SendData(SC_USART, inData[sIndex++]);
			while(USART_GetFlagStatus(SC_USART, USART_FLAG_TC) == RESET)
			{
			}
			/* Flush the SC_USART DR */
			(void)USART_ReceiveData(SC_USART);
			/* Disable the DMA Receive (Reset DMAR bit only) */
			USART_DMACmd(SC_USART, USART_DMAReq_Rx, DISABLE);
		};

		/* Or receive body data from SC ------------------------------------------*/
		i = 0;
		if(USART_ByteReceive(&locData, SC_RECEIVE_TIMEOUT*2) == SUCCESS)
		{
			SC_Responce.Data[i++] = locData;
			while(USART_ByteReceive(&locData, SC_RECEIVE_TIMEOUT) == SUCCESS)
			{
				SC_Responce.Data[i++] = locData;
			}
		}
		
		if(i>=2)
		{
			tmp = (SC_Responce.Data[i-2]<<8) | SC_Responce.Data[i-1] ;
			if((tmp & 0xFF00) == 0x6100)
			{
				
#if 1			//取响应
				*outLen = (tmp%0x100);
				tmp = cpu_GetResponse(SC_PSAM_CARD,tmp%256,outData);
				outData[*outLen] = tmp/0x100;
				(*outLen)++;
				outData[*outLen] = tmp%0x100;
				(*outLen)++;
#else			//返回由上位机自己去响应	
//				outData[0] = tmp/0x100;
//				outData[1] = tmp%0x100;
//				*outLen = 2;
#endif				
			}
			else
			{
				*outLen = i;
				memcpy(outData,SC_Responce.Data,i);
			}
			return tmp;
		}
	}
	
	return 0x6700;
}

/*******************************************************************************
*国密PSAM命令
*******************************************************************************/
unsigned short GuoMiTripleAuth(unsigned char P1,unsigned char P2,unsigned char *txBuf,
								unsigned char txLen,unsigned char *rxBuf,unsigned char rxLen)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x80;
    SC_ADPU.Header.INS = 0xEC;
    SC_ADPU.Header.P1  = P1;
    SC_ADPU.Header.P2  = P2;
    SC_ADPU.Body.LC    = txLen;			//要发送的数据长度
//	SC_ADPU.Body.LE    = 8;

    memcpy(SC_ADPU.Body.Data,txBuf,SC_ADPU.Body.LC);

    SC_Handler(0,&SCState[0], &SC_ADPU, &SC_Responce );

    if (SCState[0] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        if((tmp/256)==0x61)
        {
//			rxLen = tmp&0xFF;
            tmp = cpu_GetResponse(0,tmp%256,(uint8_t *)rxBuf) ;
        }
        return tmp ;
    }	
}

/*******************************************************************************
*获取密钥流
*******************************************************************************/
unsigned short SAMGetKeyliu(unsigned char P1,unsigned char P2,unsigned char *rlen,unsigned char *rdata)
{
    uint16_t tmp ;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;

    SC_ADPU.Header.CLA = 0x80;
    SC_ADPU.Header.INS = 0xED;
    SC_ADPU.Header.P1  = P1;
    SC_ADPU.Header.P2  = P2;
	SC_ADPU.Body.LC = 0;
	if(P2 > 0)
    {
		SC_ADPU.Body.LE = P1+1;			//要发送的数据长度
	}
	else
	{
		SC_ADPU.Body.LE = P1;
	}
//	SC_ADPU.Body.LE    = 8;

//	memcpy(SC_ADPU.Body.Data,txBuf,SC_ADPU.Body.LC);

    SC_Handler(0,&SCState[0], &SC_ADPU, &SC_Responce );

    if (SCState[0] == SC_POWER_OFF)
    {
        return 0x0;
    }
    else
    {
        tmp = ((SC_Responce.SW1 << 8) | (SC_Responce.SW2)) ;
        
		if(tmp == 0x9000)
		{
			*rlen = SC_ADPU.Body.LE;
			memcpy(rdata,SC_Responce.Data,SC_ADPU.Body.LE);
		}
		else if((tmp/256)==0x61)
        {
			*rlen = (unsigned char)(tmp&0xFF);
            tmp = cpu_GetResponse(0,tmp%256,(uint8_t *)rdata) ;
        }
        return tmp ;
    }		
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
