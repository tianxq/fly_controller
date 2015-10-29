#include "stm32f10x.h"

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

u8 readMask(u8 add, u8 *data, u8 num);
u8 writeMask(u8 add,u8 *wFlashBuffer, u8 num);