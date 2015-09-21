/**********************************************************
                   STM32 DFU移植实验

* @ 硬件平台：战舰STM32开发板

**********************************************************/

#include "stm32_config.h"

#include "led.h"
#include "key.h"
//#include "lcd_tft.h"
#include "stmflash.h"


#include "hw_config.h"	//USB相关头文件
#include "usb_lib.h"
#include "usb_conf.h"
#include "usb_prop.h"
#include "usb_pwr.h"
#include "dfu_mal.h"

u8 Dis_buffer[16];	//显示缓存

typedef  void (*pFunction)(void);

uint8_t DeviceState;
uint8_t DeviceStatus[6];
pFunction Jump_To_Application;
uint32_t JumpAddress;

u8 DFU_Mask_Read(void)
{
	uint8_t mask;
  ReadFlashNBtye(0, &mask, 1);  
	return mask;
}

void DFU_Button_Config1(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

    /* Enable the GPIO_LED Clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void RCC_Configuration(void)
{
  RCC_DeInit();//??? RCC?????????
 
  RCC_HSICmd(ENABLE);//??HSI  
  while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)//??HSI????
  {
  }
 
  if(1)
  {   
    RCC_HCLKConfig(RCC_SYSCLK_Div1);   
    RCC_PCLK1Config(RCC_HCLK_Div2);
    RCC_PCLK2Config(RCC_HCLK_Div1);

    RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12);                
    RCC_PLLCmd(ENABLE);

    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }
 
    //??????(SYSCLK) ??PLL??????
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);   
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
}
/**********************************************************
                           主函数
**********************************************************/
int main(void)
{
	u8 i=0;
  RCC_ClocksTypeDef* RCC_Clocks;
	
	//RCC_Configuration();//用内部晶振，并且屏蔽掉SystemInit
	delay_init(72);	//初始化延时函数
	LED_Init();	//初始化LED接口

	DFU_Button_Config1();	//初始化跳转APP程序按键
	

	i=RCC_GetSYSCLKSource();
	RCC_GetClocksFreq( RCC_Clocks);
	
	//if(DFU_Mask_Read() == 1)//修改判断
	//检测是否进入DFU模式按键，开机没有按下则跳转到APP程序中执行
	if(DFU_Button_Read() == 1)
	{
		if(((*(__IO uint32_t*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000)	//检测APP地址是否合法
		{
			//跳转到APP地址开始执行，地址+4位置是复位中断入口
			JumpAddress = *(__IO uint32_t*) (ApplicationAddress + 4);
			Jump_To_Application = (pFunction) JumpAddress;

			//设置APP程序堆栈指针
			__set_MSP(*(__IO uint32_t*) ApplicationAddress);
			
			//跳转到APP程序中执行
			Jump_To_Application();
		}
	}


	//进入APP升级模式
	//WriteFlashNBtye(0,&i,4); //擦除标志

	DeviceState = STATE_dfuERROR;
	DeviceStatus[0] = STATUS_ERRFIRMWARE;
	DeviceStatus[4] = DeviceState;
	
	Set_System();
	Set_USBClock();
	USB_Init();
	
	while(1)
	{
		i++;
		delay_ms(10);
		if(i == 20)
		{
			LED0 = ~LED0;
			i = 0;
		}
	}
} 
