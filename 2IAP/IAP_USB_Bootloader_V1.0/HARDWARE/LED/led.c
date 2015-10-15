/**********************************************************
* @ File name -> led.c
* @ Version   -> V1.0
* @ Date      -> 10-31-2013
* @ Brief     -> LED������غ���

 V1.*
* @ Revise    ->
**********************************************************/

#include "led.h"

/**********************************************************
* �������� ---> LED�ӿڳ�ʼ��
* ��ڲ��� ---> none
* ���ز��� ---> none 
* ����˵�� ---> none
**********************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
    /* Enable the GPIO_LED Clock */
    RCC_APB2PeriphClockCmd(LED_RCC, ENABLE);
    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = LED_X_G|LED_X_R|LED_GPS_G|LED_GPS_R
																	|LED_CH_G|LED_CH_R;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(LED_GPIO, &GPIO_InitStructure);
		
	  RCC_APB2PeriphClockCmd(CHG_RCC, ENABLE);
    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = CHG_ERR|CHG_STA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(CHG_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	//���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	GPIO_ResetBits(GPIOA, GPIO_Pin_7);	//�ص����

}

