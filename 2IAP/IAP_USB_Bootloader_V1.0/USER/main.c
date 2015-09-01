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
/**********************************************************
                           主函数
**********************************************************/
int main(void)
{
	u8 i=0;

	delay_init(72);	//初始化延时函数
	LED_Init();	//初始化LED接口

	if(DFU_Mask_Read() == 0)
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
	WriteFlashNBtye(0,&i,4); //擦除标志

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
