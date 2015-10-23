#include "stm32f10x.h"

/**美国手=0，日本手=1**/
extern uint8_t handStyle;
/**开始摇杆校准=1**/
extern uint8_t calibration;
/**升级**/
extern uint8_t usbSetup;

extern uint8_t HIDReceive_Buffer[64];
extern uint8_t HIDReceive_Buffer_len;

void usbHIDInit(void);
void usbDisable(void);

void USB_IO_PullDown(void);
void USB_IO_PullUp(void);
