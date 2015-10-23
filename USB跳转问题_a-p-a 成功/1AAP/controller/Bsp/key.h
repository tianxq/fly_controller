#include "stdint.h"
#include "stm32f10x.h"

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

typedef   enum 
{  
	NoKeyDownStatus=0, 
	KeySureDownStatus, 
	OnceKeyDownStatus, 
	ContiousKeyDownStatus 
}KeyStatus; 

typedef struct
{
	KeyStatus state;
	int TimeCount;
}KeyStatusTypeDef;

extern KeyStatusTypeDef keyAst;
extern KeyStatusTypeDef keyBst;
extern KeyStatusTypeDef keyOKFst;
extern KeyStatusTypeDef keyHOMEst;
extern KeyStatusTypeDef keyPHOTOst;
extern KeyStatusTypeDef keyVIDEOst;


//static KeyStatus state = NoKeyDownStatus;      


void key_IO_Init(void);
KeyStatus  ReadKeyStatus(GPIO_TypeDef* keyGPIO,uint16_t keyGPIOPin,
												KeyStatusTypeDef* key);

void key_Init(void);
void timekey_callback(void *ptmr, void *parg);

