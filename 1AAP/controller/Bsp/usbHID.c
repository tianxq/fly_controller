#include "includes.h"

/**美国手=0，日本手=1**/
uint8_t handStyle=0;
/**开始摇杆校准=1**/
uint8_t calibration=0;
/**升级**/
uint8_t usbSetup=0;

void usbHIDInit(void)
{
	Set_System();

  USB_Interrupts_Config();

  Set_USBClock();

  USB_Init();
}
