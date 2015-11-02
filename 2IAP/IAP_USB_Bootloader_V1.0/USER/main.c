/**********************************************************
                   STM32 DFU移植实验

* @ 硬件平台：战舰STM32开发板

**********************************************************/

#include "stm32_config.h"

#include "led.h"
#include "key.h"
#include "stmflash.h"


#include "hw_config.h"	//USB相关头文件
#include "usb_lib.h"
#include "usb_conf.h"
#include "usb_prop.h"
#include "usb_pwr.h"
#include "dfu_mal.h"

typedef  void (*pFunction)(void);

uint8_t DeviceState;
uint8_t DeviceStatus[6];
pFunction Jump_To_Application;
uint32_t JumpAddress;

u8 DFU_Mask_Read(void)
{
	uint8_t mask[4];

	ReadFlashNBtye(HIDmask_address_start+HIDmask_address_boot, mask, 4); 
	return mask[0];
}
u8 DFU_Mask_Write(u8 *mask)
{
	uint8_t mask2[4];

	WriteFlashNBtye(HIDmask_address_start+HIDmask_address_boot,mask,4);
  ReadFlashNBtye(HIDmask_address_start+HIDmask_address_boot, mask2, 4);

	if(*mask == mask2[0]) return 1;
	else return 0;
}


void pull_down(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	u32 i=0x1ffffff;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
GPIO_ResetBits(GPIOA, GPIO_Pin_11|GPIO_Pin_12);	//USB断开
	
	while(i--);

}
void pull_up(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	u32 i=0x1ffff;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
GPIO_SetBits(GPIOA, GPIO_Pin_11|GPIO_Pin_12);	//USB断开
	
	GPIO_DeInit(GPIOA);
	//while(i--);
	delay_ms(100);

}
void usbDisable(void)
{
	PowerOff();
	pull_down();
}
/**********************************************************
                           主函数
**********************************************************/
int main(void)
{
	u8  mask[4]={1};
  RCC_ClocksTypeDef* RCC_Clocks;
	u32 i=0;

	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOB);
	GPIO_AFIODeInit();

	LED_Init();	//初始化LED接口
  key_Init();


#if 0
	DFU_Mask_Write(mask); //加标志
#endif
	
	if(DFU_Mask_Read() == 0)//修改判断
	//检测是否进入DFU模式按键，开机没有按下则跳转到APP程序中执行
	//if(DFU_Button_Read() == 1)
	{
		__set_PRIMASK(1);//关掉所有中断
		//关闭USB
		usbDisable();

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


	DeviceState = STATE_dfuERROR;
	DeviceStatus[0] = STATUS_ERRFIRMWARE;
	DeviceStatus[4] = DeviceState;
	
	pull_down();
	pull_up();
	Set_System();
	Set_USBClock();
	USB_Init();
		
	//进入APP升级模式
	mask[0]=0;
	DFU_Mask_Write(mask); //写标志
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
