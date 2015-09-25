#include "stm32f10x.h"

/********************************* LED³õÊ¼»¯ *************************************/
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

//#define KEY_GPIO    GPIOA|GPIOB|GPIOC
//#define KEY_RCC     RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC
//#define KEY_A        GPIO_Pin_2
//#define KEY_B        GPIO_Pin_3
//#define KEY_OKF      GPIO_Pin_15
//#define KEY_HOME     GPIO_Pin_13
//#define KEY_PHOTO    GPIO_Pin_14
//#define KEY_VIDEO    GPIO_Pin_15


#define LedOn(X)     GPIO_ResetBits(LED_GPIO, X);
#define LedOff(X)    GPIO_SetBits(LED_GPIO, X);

//#define BeepOn()     TIM_SetCompare2(TIM3, 128);//TIM3 -> CCER  |= 1<<4; 
//#define BeepOff()    TIM_SetCompare2(TIM3, 0);//TIM3 -> CCER &= 0XFFEF; 

#define MotoOn()     TIM_CCxCmd(TIM3,TIM_Channel_2,ENABLE);//TIM_SetCompare2(TIM3, 50);
#define MotoOff()    TIM_CCxCmd(TIM3,TIM_Channel_2,DISABLE);//TIM_SetCompare2(TIM3, 100);

extern uint8_t ledXstate,ledGPSstate,ledCHstate;//µÆ×´Ì¬
extern uint8_t motostate;
extern uint8_t keythreeSt1,keythreeSt2;


void ledInit(void);
//void beepInit(uint32_t arr,uint16_t psc);
void motoInit(uint16_t arr,uint16_t psc);

