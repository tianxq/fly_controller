/**********************************************************
                   STM32 DFU��ֲʵ��

* @ Ӳ��ƽ̨��ս��STM32������

**********************************************************/

#include "stm32_config.h"

#include "led.h"
#include "key.h"
#include "stmflash.h"


#include "hw_config.h"	//USB���ͷ�ļ�
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
	
GPIO_ResetBits(GPIOA, GPIO_Pin_11|GPIO_Pin_12);	//USB�Ͽ�
	
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
	
GPIO_SetBits(GPIOA, GPIO_Pin_11|GPIO_Pin_12);	//USB�Ͽ�
	
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
                           ������
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

	LED_Init();	//��ʼ��LED�ӿ�
  key_Init();


#if 0
	DFU_Mask_Write(mask); //�ӱ�־
#endif
	
	if(DFU_Mask_Read() == 0)//�޸��ж�
	//����Ƿ����DFUģʽ����������û�а�������ת��APP������ִ��
	//if(DFU_Button_Read() == 1)
	{
		__set_PRIMASK(1);//�ص������ж�
		//�ر�USB
		usbDisable();

		if(((*(__IO uint32_t*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000)	//���APP��ַ�Ƿ�Ϸ�
		{
			//��ת��APP��ַ��ʼִ�У���ַ+4λ���Ǹ�λ�ж����
			JumpAddress = *(__IO uint32_t*) (ApplicationAddress + 4);
			Jump_To_Application = (pFunction) JumpAddress;

			//����APP�����ջָ��
			__set_MSP(*(__IO uint32_t*) ApplicationAddress);
			
			//��ת��APP������ִ��
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
		
	//����APP����ģʽ
	mask[0]=0;
	DFU_Mask_Write(mask); //д��־
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
