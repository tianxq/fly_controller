/********************************************************************************
 * @file    ADC_Sample_Task.c
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
#include "stm32f10x_crc.h"
#include "Tasks.h"
#include "Keyboard_Task.h"
#include "RC_Bus_Task.h"
#include "flash_in_stm32.h"
#include "led.h"
/*********************************************************************************
 *                                  Decalre
 ********************************************************************************/
extern uint16_t average_filter(uint16_t * filter);
extern uint8_t check_sum8(uint8_t *chk8, uint8_t num);

void RC_Channel_Calibrate_Process(void *pdata);
void adjust_channels_degree(uint16_t raw[4], uint16_t channels[4]);
/*********************************************************************************
 *                                  Variable
 ********************************************************************************/
uint16_t BatteryVoltage;//The battery voltage
uint16_t RC_Channel_Degree[CHANNEL_NUM];


/*********************************************************************************
 *                                  Function
 ********************************************************************************/



/**
 * [TaskADC description]
 * @param pdata [description]
 */
void TaskADC(void *pdata)
{
  uint16_t ADtoFilter[CHANNEL_NUM][AVERAGE_FILTER_BUFFER_SIZE],ADResults[CHANNEL_NUM];
  int i;


  stm32_adc1_init();

  while(1)
  {
    /* Sample the 6 adc channels, and filter the result values */
    for(i=0;i<CHANNEL_NUM;i++)
    {
      for(int j=0;j<AVERAGE_FILTER_BUFFER_SIZE;j++)
      {
        ADtoFilter[i][j]=AD_Value[j][i];
      }
    }

    for(i=0;i<CHANNEL_NUM;i++)
    {
      ADResults[i] = average_filter(&ADtoFilter[i][0]);
    }

    for(i=0;i<CHANNEL_NUM-1;i++)
    {
      ADResults[i]=ADResults[i]>>1;
    }

    /* adjust the value to the standard rang */
    if (RC_Profile_Cfg.enable == TRUE)
	{
		adjust_channels_degree(ADResults, RC_Channel_Degree);

        RC_Channel_Degree[4] = ADResults[4];

        RC_Channel_Degree[5] = ADResults[5];
	}
	else
	{
		memcpy(RC_Channel_Degree, ADResults, 12);
	}

    /* Check the batter voltage */
    BatteryVoltage = ADResults[6];

    if(BatteryVoltage< 0x8c0)
    {
      ledCHstate = 0;
    }
    else
    {
      ledCHstate = 1;
    }

    OSTimeDlyHMSM(0,0,0,5);

    // add for test
    if( KeyState[KEY_B_NO] == PRESSED_AND_HOLD )
    {
		KeyState[KEY_B_NO] = NORMAL;
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

  }
}



void RC_Channel_Calibrate_Process(void *pdata)
{
	struct RC_Profile_Cfg_t newConfig = {
										 {0,0,0,0},	//max
										 {0xffff,0xffff,0xffff,0xffff},//min
										};
    uint32_t timeout = 6000;//6000*5ms = 30s

    RC_Profile_Cfg.enable = FALSE;//disable the adjust.

    do{
		if ((timeout & 0x003ful) == 0)
			ToggleLed(LED_X_R); // blink a led to indicate the process running.

        for (int i=0;i<4;i++)
        {
            /* find the max value ever apeared */
            if (RC_Channel_Degree[i] > newConfig.ADC_Val_Max[i])
                newConfig.ADC_Val_Max[i] = RC_Channel_Degree[i];
            /* record the min value ever apeared */
            if (RC_Channel_Degree[i] < newConfig.ADC_Val_Min[i])
                newConfig.ADC_Val_Min[i] = RC_Channel_Degree[i];
        }

        OSTimeDlyHMSM(0,0,0,5);

    }while(timeout--);


    /* Compute the CRC of "DataBuffer" */
    newConfig.valid_chk = check_sum8(((uint8_t *)&newConfig) , (sizeof(struct RC_Profile_Cfg_t)-4));

    /* Store the parameters into the flash */
    WriteFlashNBtye(RC_PROFILE_PARAM_ADDR,(uint8_t *)&newConfig, sizeof(struct RC_Profile_Cfg_t));

    LedOff(LED_X_R);

    OSTaskDel(OS_PRIO_SELF);
}



void adjust_channels_degree(uint16_t raw[4], uint16_t channels[4])
{
	uint16_t dtemp;
	float ftemp;

	for (int i=0;i<4;i++)
	{
		dtemp = raw[i];

		/* first make sure the value not beyond the rang */
		if (dtemp > RC_Profile_Cfg.ADC_Val_Max[i])
			dtemp = RC_Profile_Cfg.ADC_Val_Max[i];

		if (dtemp < RC_Profile_Cfg.ADC_Val_Min[i])
			dtemp = RC_Profile_Cfg.ADC_Val_Min[i];

		/* covert to standard (0~2000) */
		ftemp = (float)(dtemp - RC_Profile_Cfg.ADC_Val_Min[i]);

		ftemp = ( ftemp/(RC_Profile_Cfg.ADC_Val_Max[i]-RC_Profile_Cfg.ADC_Val_Min[i]) )*2000;

		dtemp = (uint16_t)ftemp;

        /* deverse */
        if ( ((RC_Profile_Cfg.chann_reverse>>i)&0x01) == 1 )
            channels[i] = 2000 - dtemp;
		else
			channels[i] = dtemp;
	}
}
