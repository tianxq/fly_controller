/**********************************************************
                   STM32 DFU��ֲʵ��

* @ Ӳ��ƽ̨��ս��STM32������

**********************************************************/

#include "stm32_config.h"

#include "led.h"
#include "key.h"
//#include "lcd_tft.h"
#include "stmflash.h"


#include "hw_config.h"	//USB���ͷ�ļ�
#include "usb_lib.h"
#include "usb_conf.h"
#include "usb_prop.h"
#include "usb_pwr.h"
#include "dfu_mal.h"

u8 Dis_buffer[16];	//��ʾ����

typedef  void (*pFunction)(void);

uint8_t DeviceState;
uint8_t DeviceStatus[6];
pFunction Jump_To_Application;
uint32_t JumpAddress;

u8 DFU_Mask_Read(void)
{
	uint8_t mask[4];
  ReadFlashNBtye(0, mask, 4);  
	return mask[0];
}
u8 DFU_Mask_Write(u8 *mask)
{
	uint8_t mask2[4];
	WriteFlashNBtye(0,mask,4);
  ReadFlashNBtye(0, mask2, 4);
	if(*mask == mask2[0]) return 1;
	else return 0;
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

//�ܽŸ�λ
static void GPIO_DeIntConfiguration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /* Configure all unused GPIO port pins in Analog Input mode (floating input
    trigger OFF), this will reduce the power consumption and increase the device
    immunity against EMI/EMC *************************************************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_GPIOE, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_GPIOE, DISABLE);
}
void usbDisable(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	u32 i=0x2ffffff;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, DISABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	PowerOff();

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_11|GPIO_Pin_12);
	while(i--);
}
/**********************************************************
                           ������
**********************************************************/
int main(void)
{
	u8 i=0, mask[4]={1};
  RCC_ClocksTypeDef* RCC_Clocks;

	//GPIO_DeIntConfiguration();
	
	//RCC_Configuration();//���ڲ����񣬲������ε�SystemInit
	//delay_init(72);	//��ʼ����ʱ����

	//��������ʱ������һ�����������ˣ�������app������֮ǰ�ȹص����������ж�
	LED_Init();	//��ʼ��LED�ӿ�
	LedOn(LED_X_G);
	LedOn(LED_GPS_G);
	LedOn(LED_CH_G);
	LedOff(LED_X_R);
	LedOff(LED_GPS_R);
	LedOff(LED_CH_R);

#if 0
	DFU_Mask_Write(mask); //�ӱ�־
#endif
	
//��ϵͳƵ��
//	i=RCC_GetSYSCLKSource();
//	RCC_GetClocksFreq( RCC_Clocks);
	
	if(DFU_Mask_Read() == 0)//�޸��ж�
	//����Ƿ����DFUģʽ����������û�а�������ת��APP������ִ��
	//if(DFU_Button_Read() == 1)
	{
		//�ر�USB
		usbDisable();
		//__set_PRIMASK(1);//�ص������ж�
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
