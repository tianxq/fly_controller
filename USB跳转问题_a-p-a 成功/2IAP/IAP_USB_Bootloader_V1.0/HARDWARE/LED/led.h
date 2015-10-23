/**********************************************************
* @ File name -> led.h
* @ Version   -> V1.0
* @ Date      -> 10-31-2013
* @ Brief     -> LED控制相关函数头文件

 V1.*
* @ Revise    ->
**********************************************************/

#ifndef _led_h_
#define _led_h_

/**********************************************************
                     外部函数头文件
         应用到不同的外设头文件请在这里修改即可                        
**********************************************************/

#include "sys.h"

/**********************************************************
                         定义接口
**********************************************************/

//#define LED0					PCout(0)
//#define LED1					PCout(1)

#define LED0					PBout(5)
#define LED1					PEout(6)

#define LED_GPIO    GPIOB
//#define BEEP_GPIO   GPIOA
#define MOTO_GPIO   GPIOA

#define LED_RCC     RCC_APB2Periph_GPIOB
//#define BEEP_RCC    RCC_APB2Periph_GPIOA
#define MOTO_RCC    RCC_APB2Periph_GPIOA

#define LED_X_G       GPIO_Pin_6
#define LED_X_R       GPIO_Pin_7
#define LED_GPS_G     GPIO_Pin_4
#define LED_GPS_R     GPIO_Pin_5
#define LED_CH_G      GPIO_Pin_8
#define LED_CH_R      GPIO_Pin_9
//#define BEEP        GPIO_Pin_7
#define MOTO        GPIO_Pin_7

#define CHG_ERR     GPIO_Pin_0
#define CHG_STA     GPIO_Pin_1
#define CHG_GPIO    GPIOB
#define CHG_RCC     RCC_APB2Periph_GPIOB

#define LedOn(X)     GPIO_ResetBits(LED_GPIO, X);
#define LedOff(X)    GPIO_SetBits(LED_GPIO, X);
/**********************************************************
                       外部功能函数                      
**********************************************************/

void LED_Init(void);	//初始化LED接口

#endif

