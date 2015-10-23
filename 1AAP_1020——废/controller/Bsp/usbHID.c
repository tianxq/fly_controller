#include "includes.h"

/**������=0���ձ���=1**/
uint8_t handStyle=0;
/**��ʼҡ��У׼=1**/
uint8_t calibration=0;
/**����**/
uint8_t usbSetup=0;

void usbHIDInit(void)
{
	usbDisable();
	
	Set_System();

  USB_Interrupts_Config();

  Set_USBClock();

  USB_Init();
}

void usbDisable(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	u32 i=0x2ffffff;
	
	//NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
	
		PowerOff();//����ȥ������
	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
GPIO_ResetBits(GPIOA, GPIO_Pin_11|GPIO_Pin_12);	//USB�Ͽ�
	
	while(i--);

}