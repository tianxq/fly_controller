/**********************************************************
* @ File name -> key.h
* @ Version   -> V1.0
* @ Date      -> 01-18-2014
* @ Brief     -> ������������ͷ�ļ�

 V1.*
* @ Revise    ->
**********************************************************/

#ifndef _key_h_
#define _key_h_

/**********************************************************
                     �ⲿ����ͷ�ļ�
         Ӧ�õ���ͬ������ͷ�ļ����������޸ļ���                        
**********************************************************/

#include "sys.h"
#include "delay.h"

/**********************************************************
                         ����ӿ�
**********************************************************/

#define KEY0					PCin(2)	//S6 --> left
#define KEY1					PCin(3)	//S5 --> up
#define KEY2					PCin(4)	//S4 --> down
#define KEY3					PCin(5)	//S2 --> right

#define KEY4					PAin(0)	//S7 --> wku

//�������ֵ

#define KEY_UP					2	//S5
#define KEY_DOWN				3	//S4
#define KEY_LEFT				1	//S6
#define KEY_RIGHT				4	//S2

#define KEY_WKUP				5	//S7

/**********************************************************
                        ����ֵ��ȡ
**********************************************************/

//           k3  k2  k1  k0  k4
//����λ���壺�ҡ��¡��ϡ��󡢻���
//b7  b6  b5 b4  b3  b2 b1   b0

//#define KeyInput				(u8)((((GPIOC->IDR & 0x0000003c)) >> 1) | (~(GPIOA->IDR & 0x00000001)))	//��ȡ����ֵ

#define KeyInput				(u8)(((GPIOC->IDR & 0x0000003c)) >> 1)	//��ȡ����ֵ

#define Key_ContPress			0x13	//֧�������İ����뽫��Ӧ��λ��0���ɣ�����λ����1
										//ֻ��up��down֧��������ֵΪ0x13
										//ֻ�������֧��������  ֵΪ0x0d

/**********************************************************
                       �ⲿ���ܺ���                      
**********************************************************/

void KEY_Init(void);	//��ʼ��KEY�ӿ�

u8 KEY_Scan(u8 mode);	//������ֵ��ȡ

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

