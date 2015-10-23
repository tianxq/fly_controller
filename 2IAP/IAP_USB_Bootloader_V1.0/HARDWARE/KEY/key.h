/**********************************************************
* @ File name -> key.h
* @ Version   -> V1.0
* @ Date      -> 01-18-2014
* @ Brief     -> 按键驱动函数头文件

 V1.*
* @ Revise    ->
**********************************************************/

#ifndef _key_h_
#define _key_h_

/**********************************************************
                     外部函数头文件
         应用到不同的外设头文件请在这里修改即可                        
**********************************************************/

#include "sys.h"
#include "delay.h"

/**********************************************************
                         定义接口
**********************************************************/

#define KEY0					PCin(2)	//S6 --> left
#define KEY1					PCin(3)	//S5 --> up
#define KEY2					PCin(4)	//S4 --> down
#define KEY3					PCin(5)	//S2 --> right

#define KEY4					PAin(0)	//S7 --> wku

//软件返回值

#define KEY_UP					2	//S5
#define KEY_DOWN				3	//S4
#define KEY_LEFT				1	//S6
#define KEY_RIGHT				4	//S2

#define KEY_WKUP				5	//S7

/**********************************************************
                        按键值读取
**********************************************************/

//           k3  k2  k1  k0  k4
//按键位定义：右、下、上、左、唤醒
//b7  b6  b5 b4  b3  b2 b1   b0

//#define KeyInput				(u8)((((GPIOC->IDR & 0x0000003c)) >> 1) | (~(GPIOA->IDR & 0x00000001)))	//读取按键值

#define KeyInput				(u8)(((GPIOC->IDR & 0x0000003c)) >> 1)	//读取按键值

#define Key_ContPress			0x13	//支持连按的按键请将相应的位置0即可，其他位保持1
										//只有up和down支持连按，值为0x13
										//只有左和右支持连按，  值为0x0d

/**********************************************************
                       外部功能函数                      
**********************************************************/

void KEY_Init(void);	//初始化KEY接口

u8 KEY_Scan(u8 mode);	//按键键值读取

void key_Init(void);
#endif

#define KEY_GPIO    GPIOA|GPIOB|GPIOC
#define KEY_RCC     RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC
#define KEY_A        GPIO_Pin_2
#define KEY_B        GPIO_Pin_3
#define KEY_OKF      GPIO_Pin_15
#define KEY_HOME     GPIO_Pin_13
#define KEY_PHOTO    GPIO_Pin_14
//#define KEY_VIDEO    GPIO_Pin_15
#define KEY_A_GPIO        GPIOB
#define KEY_B_GPIO        GPIOB
#define KEY_OKF_GPIO      GPIOA
#define KEY_HOME_GPIO     GPIOC
#define KEY_PHOTO_GPIO    GPIOC
//#define KEY_VIDEO_GPIO    GPIOC

#define KEY_NEWHAND   GPIO_Pin_13
#define KEY_AE   GPIO_Pin_14
#define KEY_VIO   GPIO_Pin_15
#define KEY_THREE_GPIO   GPIOB

