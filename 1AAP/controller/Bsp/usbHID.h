#include "stm32f10x.h"

/**美国手=0，日本手=1**/
extern uint8_t handStyle;
/**开始摇杆校准=1**/
extern uint8_t calibration;
/**升级**/
extern uint8_t usbSetup;

void usbHIDInit(void);

