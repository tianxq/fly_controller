#include "includes.h"

/**������=0���ձ���=1**/
uint8_t handStyle=0;
/**��ʼҡ��У׼=1**/
uint8_t calibration=0;
/**����**/
uint8_t usbSetup=0;

void usbHIDInit(void)
{
	Set_System();

  USB_Interrupts_Config();

  Set_USBClock();

  USB_Init();
}
