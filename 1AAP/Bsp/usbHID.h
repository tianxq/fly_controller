#include "stm32f10x.h"

#define HIDmask_address_start 0x0801F800    //���λ
#define FLASH_PAGE_SIZE		0x400
typedef   enum 
{  
	HIDmask_address_boot=0, 
	HIDmask_address_calibration=0+FLASH_PAGE_SIZE,  //У׼(H/L)
	HIDmask_address_adcinversion=HIDmask_address_calibration+16, //��ת
	HIDmask_address_amjp=30+FLASH_PAGE_SIZE,        //�ձ���������  
	 
}HIDmask_address; 

/**������=0���ձ���=1**/
extern uint8_t handStyle;
/**��ʼҡ��У׼=1**/
extern uint8_t calibration;
/**����**/
extern uint8_t usbSetup;

extern uint8_t HIDReceive_Buffer[64];
extern uint8_t HIDReceive_Buffer_len;

void usbHIDInit(void);
void usbDisable(void);

void USB_IO_PullDown(void);
void USB_IO_PullUp(void);

void readMask(u32 add, u8 *data, u8 num);
u8 writeMask(u32 add,u8 *wFlashBuffer, u8 num);
