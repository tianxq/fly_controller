#include "includes.h"

/**美国手=0，日本手=1**/
uint8_t handStyle=0;
/**开始摇杆校准=1**/
uint8_t calibration=0;
/**升级**/
uint8_t usbSetup=0;

void usbHIDInit(void)
{
//	usbDisable();
	
	Set_System();

  USB_Interrupts_Config();

  Set_USBClock();

  USB_Init();
}

void usbDisable(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	u32 i=0x1ffffff;
	
	//NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
//	__set_PRIMASK(1);//关掉所有中断
	
		PowerOff();//跳回去就死了
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, DISABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_11|GPIO_Pin_12);
	while(i--);

}


extern void stm32_delay_ms(unsigned int );

void USB_IO_PullDown(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, DISABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		GPIO_ResetBits(GPIOA, GPIO_Pin_12);
	
		stm32_delay_ms(100);
}


void USB_IO_PullUp(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		GPIO_SetBits(GPIOA, GPIO_Pin_12);
	
		stm32_delay_ms(100);
	
		GPIO_DeInit(GPIOA);
}

void readMask(u32 add, u8 *data, u8 num)
{
	add+=HIDmask_address_start;
	ReadFlashNBtye(add, data, num);
}

u8 writeMask(u32 add,u8 *wFlashBuffer, u8 num)
{
	u8 mask2;
	u8 rFlashBuffer[32];
	
	add+=HIDmask_address_start;

	WriteFlashNBtye(add,wFlashBuffer,num);
	ReadFlashNBtye(add, rFlashBuffer, num);
	if(rFlashBuffer[0]==wFlashBuffer[0])
	 return 1;
	else return 0;
}