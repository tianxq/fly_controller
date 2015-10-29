/*******************************************************************************
 * Keyboad_Task.h
 */
#ifndef __KEYBOARD_TASK_H
#define __KEYBOARD_TASK_H




/*******************************************************************************
 * Macro
 */
/**/
#define KEY_MODE_NO				 0
#define KEY_HOME_NO        1
#define KEY_OKF_NO         2
#define KEY_PHOTO_NO       3
#define KEY_A_NO           4
#define KEY_B_NO           5



#define KEY_GPIO_PORT    				GPIOA|GPIOB|GPIOC
#define KEY_RCC_CLK     				RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC
#define KEY_A_PIN        				GPIO_Pin_2
#define KEY_B_PIN        				GPIO_Pin_3
#define KEY_OKF_PIN      				GPIO_Pin_15
#define KEY_HOME_PIN     				GPIO_Pin_13
#define KEY_PHOTO_PIN    				GPIO_Pin_14

#define KEY_A_GPIO_PORT        	GPIOB
#define KEY_B_GPIO_PORT        	GPIOB
#define KEY_OKF_GPIO_PORT      	GPIOA
#define KEY_HOME_GPIO_PORT     	GPIOC
#define KEY_PHOTO_GPIO_PORT    	GPIOC


#define KEY_NEWHAND   					GPIO_Pin_13
#define KEY_AE   								GPIO_Pin_14
#define KEY_VIO   							GPIO_Pin_15
#define KEY_THREE_GPIO   				GPIOB
/*******************************************************************************
 * Type
 */

typedef enum
{
	NORMAL=1,
	CLICKED_ONCE=2,
	PRESSED_AND_HOLD=3,
}KeyStatus_t;


/*******************************************************************************
 *  Declare
 */
extern KeyStatus_t KeyState[6];

#endif //__KEYBOARD_TASK_H
