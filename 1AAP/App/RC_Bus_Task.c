/********************************************************************************
 * @file    RC_Bus_Profile.c
 * @author  Andy.Zhang
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
#include "stm32f10x_crc.h"
#include <stdbool.h>
#include "Stm32Usart.h"
#include "RC_Bus_Task.h"
#include "Keyboard_Task.h"
#include "flash_in_stm32.h"

/*********************************************************************************
 *                                  Declare
 ********************************************************************************/
extern uint16_t RC_Channel_Degree[];

uint8_t check_sum8(uint8_t *chk8,uint8_t num);
void hal_rc_bus_init(void);
void RC_Bus_Period_Timer_CB(OS_TMR *ptmr, void *parg);
bool get_rc_profile_param(struct RC_Profile_Cfg_t * config);


/*********************************************************************************
 *                                  Variable
 ********************************************************************************/
rc_info_pkt_t RCBusTxData = {.head = 0x5AA5,
							 .type = 0x01,
							 .length = 14,
                          };

uint8_t rCBusRxBuffer[32];

struct RC_Profile_Cfg_t RC_Profile_Cfg = {
										  .chann1_value_reverse = 0,
										  .chann2_value_reverse = 1,
										  .chann3_value_reverse = 1,
										  .chann4_value_reverse = 0,
										 };
//OS_TMR * RCBuS_Tx_Evt_Timer;
//INT8U RCBusTxEvtTimerErr;
static uint16_t channel1,channel2;
/*********************************************************************************
 *                                  Function
 ********************************************************************************/

/**
 * A uCOS task handling the events on rc bus, transmit and receive data on the bus.
 * This task may have the higher priority in the system, because this task is the
 * most import thing of the system.
 * @param pdata [ucos input param]
 */
void RC_Bus_Task(void * pdata)
{
	hal_rc_bus_init();

	if (get_rc_profile_param(&RC_Profile_Cfg) == true)
		RC_Profile_Cfg.enable = true;

//	RC_Profile_Cfg.chann2_value_reverse = 1;
//	RC_Profile_Cfg.chann3_value_reverse = 1;

	while(1)
	{
		/* Fill in the ADC Sample results */
		memcpy(RCBusTxData.adc, RC_Channel_Degree, 12);
channel1 = RC_Channel_Degree[0];
channel2 = RC_Channel_Degree[1];

if (channel1 || channel2 )
	RCBusTxData.chk = 0;
		/* Fill the key status into the packet */
		RCBusTxData.mode         = KeyState[KEY_MODE_NO];
		RCBusTxData.gohome       = KeyState[KEY_HOME_NO];
		RCBusTxData.one_key_fly  = KeyState[KEY_OKF_NO];
		RCBusTxData.photo        = KeyState[KEY_PHOTO_NO];
		RCBusTxData.key_a        = KeyState[KEY_A_NO];
		RCBusTxData.key_b        = KeyState[KEY_B_NO];

		/* Caculate the pakcet chk byte */
		RCBusTxData.chk = check_sum8(RCBusTxData.buffer, sizeof(rc_info_pkt_t)-1);
		/* Send the Data on the comport bus */
		RC_BUS_Strobe_Write((uint8_t *)&RCBusTxData,sizeof(rc_info_pkt_t));
		

		OSTimeDly(OS_TICKS_PER_SEC/200);
	}
}


/**
 * Initialize the usart1 and relatived dma channels as the rc bus fomat using
 * for transmitting and receiving data to the RF model.
 */
void hal_rc_bus_init(void)
{
	ComPort1Init(115200,UART_CONFIG_PAR_EVEN);
}



/**
 * Strobe the usart1 dma transmitting the valid data.
 */
void RC_BUS_Strobe_Write(uint8_t *rxbuf, uint8_t len)
{
    //hal_usart1_strobe_write(sizeof(rc_info_pkt_t));
		ComPortWrite(&nCom1,rxbuf,len);
		while(0 == ComPortWaitTxEnd(&nCom1))
		{
			//ÑÓÊ±
		}
}


/**
 * Packet check algorithm of the rc bus application profile.
 * @param  chk8 [point to a packet]
 * @param  num  [the size of the packet]
 * @return      [the check byte result]
 */
uint8_t check_sum8(uint8_t *chk8, uint8_t num)
{
	uint8_t temp=0;
	for( ;num>0;num--)
	{
		temp=temp+*chk8;
		if(temp<*chk8)
		temp++;
		chk8++;
	}
	return temp;
}



/**
 * Callback function of the uCOS timer: RCBuS_Tx_Evt_Timer
 * Call this routine every 5 millisconds to transmit valid data
 * @param ptmr [description]
 * @param parg [description]
 */
void RC_Bus_Period_Timer_CB(OS_TMR *ptmr, void *parg)
{
	GPIO_WriteBit(LED_GPIO, LED_CH_G, (BitAction)((LED_GPIO->ODR & LED_CH_G) == 0));

  	/* Fill in the ADC Sample results */
  	memcpy(RCBusTxData.adc, RC_Channel_Degree, 12);

	/* Fill the key status into the packet */
	RCBusTxData.mode         = KeyState[KEY_MODE_NO];
	RCBusTxData.gohome       = KeyState[KEY_HOME_NO];
	RCBusTxData.one_key_fly  = KeyState[KEY_OKF_NO];
	RCBusTxData.photo        = KeyState[KEY_PHOTO_NO];
	RCBusTxData.key_a        = KeyState[KEY_A_NO];
	RCBusTxData.key_b        = KeyState[KEY_B_NO];

	/* Caculate the pakcet chk byte */
	RCBusTxData.chk = check_sum8(RCBusTxData.buffer, sizeof(rc_info_pkt_t)-1);
	/* Send the Data on the comport bus */
  	RC_BUS_Strobe_Write((uint8_t *)&RCBusTxData,sizeof(rc_info_pkt_t));
}


bool get_rc_profile_param(struct RC_Profile_Cfg_t * config)
{
	struct RC_Profile_Cfg_t temp;


	ReadFlashNBtye(RC_PROFILE_PARAM_ADDR, (uint8_t *)&temp, sizeof(struct RC_Profile_Cfg_t));

	/* Check the configuration */
	if ( temp.valid_chk != check_sum8(((uint8_t *)&temp), (sizeof(struct RC_Profile_Cfg_t)-4)))
	    return false;

	memcpy(config, &temp, sizeof(struct RC_Profile_Cfg_t));

	return true;
}



/*
RCBuS_Tx_Evt_Timer  = OSTmrCreate(	OS_TMR_CFG_TICKS_PER_SEC/200,						//dly
						OS_TMR_CFG_TICKS_PER_SEC/200,					//period
						OS_TMR_OPT_PERIODIC,									//opt
						(OS_TMR_CALLBACK)RC_Bus_Period_Timer_CB,			//callback
						NULL,																	//callback_arg
						"Uart Period Timer",									//pname
						&RCBusTxEvtTimerErr										//perr
						);

OSTmrStart(RCBuS_Tx_Evt_Timer, &RCBusTxEvtTimerErr);
*/
