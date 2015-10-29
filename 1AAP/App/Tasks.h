/*******************************************************************************
 * @file:Tasks.h
 */


 /********************** uCOS task defination *******************************/

 #define	  Task_Start_Size			64
 #define   Task_Display_Stk_Size 	64
 #define   Task_ADC_Stk_Size     	128
 #define   Task_Key_Stk_Size    		64
 #define   Task_RCBus_Stk_Size    	256
 #define   Task_USBHID_Stk_Size    	512


 #define	  Task_Start_Prio			(OS_TASK_IDLE_PRIO-1)
 #define   Task_ADC_Prio       		(OS_TASK_IDLE_PRIO-2)
 #define   Task_Key_Prio      		(OS_TASK_IDLE_PRIO-3)
 #define   Task_USBHID_Prio    		(OS_TASK_IDLE_PRIO-4)
 #define   Task_Display_Prio   		(OS_TASK_IDLE_PRIO-5)
 #define   Task_RCBus_Prio      	(OS_TASK_IDLE_PRIO-6)



extern OS_STK		Task_Start_Stk[Task_Start_Size];
extern OS_STK		Task_Display_Stk[Task_Display_Stk_Size];
extern OS_STK		Task_ADC_Stk[Task_ADC_Stk_Size];
extern OS_STK		Task_Key_Stk[Task_Key_Stk_Size];
extern OS_STK		Task_RCBus_Stk[Task_RCBus_Stk_Size];
extern OS_STK		Task_USBHID_Stk[Task_USBHID_Stk_Size];
