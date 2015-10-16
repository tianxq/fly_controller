/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_conf.h
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    10/15/2010
  * @brief   Library configuration file.
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
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_CONF_H
#define __STM32F10x_CONF_H

/* Includes ------------------------------------------------------------------*/
 #include <string.h>
/* Uncomment the line below to enable peripheral header file inclusion */
#include "stm32f10x_adc.h" 
/* #include "stm32f10x_bkp.h" */
/* #include "stm32f10x_can.h" */
/* #include "stm32f10x_cec.h" */
/* #include "stm32f10x_crc.h" */
/* #include "stm32f10x_dac.h" */
#include "stm32f10x_dbgmcu.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_fsmc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_rcc.h"
/* #include "stm32f10x_rtc.h" */
#include "stm32f10x_sdio.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "system_stm32f10x.h"
/* #include "stm32f10x_wwdg.h" */
#include "misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

#include "stm32f10x.h"

#include "..\uCOS-II\Source\ucos_ii.h"

#include "Stm32Usart.h"
#include "rs485.h"
//#include "smartcard.h"
//#include "si4432.h" 
//#include "own_spi.h"
#include "FIFO_App.h"
#include "led.h"
#include "sbus.h"
#include "adc.h"
#include "key.h"
#include "usbHID.h"
#include "usb_lib.h"
//#include "usb_desc.h"
//#include "usb_pwr.h"
//#include "usb_prop.h"
//#include "usb_endp.h"
//#include "usb_istr.h"
//#include "platform_config.h"
#include "hw_config.h"
#include "flash_in_stm32.h"


//使用bootload
//#define JKB_SW_H
//无线数据接收FIFO
extern Queue QueueRFrxFIFO;
extern INT8U err_timer;
extern OS_TMR *timekeyHome,*timekeyOkf,*timekeyPhoto,*timekeyA,*timekeyB;
extern KeyStatus keyst[6];

typedef struct {

		uint16_t  head;
		uint8_t type;
		uint8_t length;
		uint8_t adc[12];
		uint16_t key;
	
//		uint8_t mode:2;
//		uint8_t gohome:2;
//		uint8_t launch:2;
//		uint8_t shoot:2;
//		uint8_t key_a:2;
//		uint8_t key_b:2;
//		uint8_t def:4;

		uint8_t chk;
}rc_pro_com_in_pkt_t;


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below to expanse the "assert_param" macro in the
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */
#pragma  diag_suppress 870	//避免编译器警告
/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *   which reports the name of the source file and the source
  *   line number of the call that failed.
  *   If expr is true, it returns no value.
  * @retval None
  */
#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void assert_failed(uint8_t* file, uint32_t line);
#else
#define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F10x_CONF_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
