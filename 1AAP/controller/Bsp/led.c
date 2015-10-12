#include "includes.h"

uint8_t ledXstate=0,ledGPSstate=0,ledCHstate=0;//灯状态
uint8_t motostate=0;
uint8_t keythreeSt1=0,keythreeSt2=0;
/********************************* LED初始化 *************************************/
void ledInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

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

}

/*
void beepInit(uint32_t arr,uint16_t psc)  
{ 
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能time3
	/* Enable the GPIO_BEEP Clock 
	RCC_APB2PeriphClockCmd(BEEP_RCC | RCC_APB2Periph_AFIO, ENABLE);
	/* Configure the GPIO_BEEP pin 
	GPIO_InitStructure.GPIO_Pin = BEEP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BEEP_GPIO, &GPIO_InitStructure);
	
	TIM_DeInit(TIM3);
	 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
	
	//TIM3 Channe2 PWM to PA7
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 
	TIM_OCInitStructure.TIM_Pulse = 128;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  
	
	TIM_ARRPreloadConfig(TIM3,ENABLE);

	TIM_Cmd(TIM3, ENABLE);  

	BeepOff();
}
*/

void motoInit(uint16_t arr,uint16_t psc)  
{   
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能time3
			/* Enable the GPIO_BEEP Clock */
	RCC_APB2PeriphClockCmd(MOTO_RCC | RCC_APB2Periph_AFIO, ENABLE);
	/* Configure the GPIO_BEEP pin */
	GPIO_InitStructure.GPIO_Pin = MOTO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MOTO_GPIO, &GPIO_InitStructure);
	
	TIM_DeInit(TIM3);
	 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
	
	//TIM3 Channe2 PWM to PA7
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 
	TIM_OCInitStructure.TIM_Pulse = 128;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  
	
	TIM_ARRPreloadConfig(TIM3,ENABLE);

	TIM_Cmd(TIM3, ENABLE);  

	TIM_SetCompare2(TIM3, 1);//占空比
	MotoOff();
}
