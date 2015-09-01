#ifndef __OWN_GPIO_H_
#define __OWN_GPIO_H_

#define GPIO_ENABLE	1

#if GPIO_ENABLE 

#ifdef __cplusplus
 extern "C" {
#endif

#include "own_type.h"
#include "stm32f10x_gpio.h"

typedef uint16_t pinname_type;
typedef GPIOMode_TypeDef  	pinmode_type;
typedef GPIOSpeed_TypeDef  	pinspeed_type;
/****************************************************************
*******    引脚速度配置，可根据具体使用修改
****************************************************************/
#define PIN_SPEED_10MHz 		GPIO_Speed_10MHz		//10MHZ的速度，请不要修改
#define PIN_SPEED_2MHz 			GPIO_Speed_2MHz			//2MHZ的速度，请不要修改
#define PIN_SPEED_50MHz 		GPIO_Speed_50MHz		//50MHZ的速度，请不要修改

/****************************************************************
*******   引脚模式配置，对应管脚的模式可修改    *****************
****************************************************************/
#define PIN_Mode_AIN  			GPIO_Mode_AIN			//模拟输入，请不要修改
#define PIN_Mode_IN_FLOATING  	GPIO_Mode_IN_FLOATING	//浮空输入，请不要修改
#define PIN_Mode_IPD 			GPIO_Mode_IPD			//下拉输入，请不要修改
#define PIN_Mode_IPU 			GPIO_Mode_IPU			//上拉输入，请不要修改
#define PIN_Mode_Out_OD  		GPIO_Mode_Out_OD		//开漏输出，请不要修改
#define PIN_Mode_Out_PP  		GPIO_Mode_Out_PP		//推挽输出，请不要修改
#define PIN_Mode_AF_OD  		GPIO_Mode_AF_OD			//复用开漏(第二功能)，请不要修改
#define PIN_Mode_AF_PP  		GPIO_Mode_AF_PP			//复用推挽(第二功能)，请不要修改

/*********************************************************************************************************
** 函数功能: 引脚全部设置为缺省值
*********************************************************************************************************/
void GPIODeInitAll(void);

/****************************************************************************************
** 函数功能: 将复用功能（重映射事件控制和EXTI设置）重设为缺省值
*****************************************************************************************/
void GPIOAFIODeInit(void);

/***************************************************************************************
** 函数功能: 引脚初始化
** 入口参数: pin_name 管脚名称
			 pin_mode 管脚模式
			 PA_00_SPEED 管脚的速度
*********************************************************************************************************/
void GPIOInit(pinname_type const pin_name,pinmode_type const pin_mode,pinspeed_type const pin_speed);

/*********************************************************************************************************
** 函数功能: 引脚置"1"
** 入口参数: pin_name 管脚名称 eg:PA_05
*********************************************************************************************************/
void GPIOSet(pinname_type const pin_name);

/*********************************************************************************************************
** 函数功能: 引脚置"0"
** 入口参数: pin_name 管脚名称 eg:PA_00
*********************************************************************************************************/
void GPIOClr(pinname_type const pin_name);

/*********************************************************************************************************
** 函数功能: 读取对应管脚的电平 高=1 低=0
** 入口参数: pin_name 管脚名称 eg:PA_00
*********************************************************************************************************/
uint8_t GPIOInputGet(pinname_type const pin_name);

/*********************************************************************************************************
** 函数功能: 读取对应管脚的电平 高=1 低=0
** 入口参数: pin_name 管脚名称 eg:PA_00
*********************************************************************************************************/
uint8_t GPIOOutputGet(pinname_type const pin_name);

/*********************************************************************************************************
** 函数功能: 对应管脚的电平取反
** 入口参数: pin_name 管脚名称 eg:PA_00
*********************************************************************************************************/
void GPIONeg(pinname_type const pin_name);

/****************************************************************
***********    引脚名称，此处请不要改动      *********************
****************************************************************/
//PA管脚名称
#define    PA_00			 101
#define    PA_01			 102
#define    PA_02			 103
#define    PA_03			 104
#define    PA_04			 105
#define    PA_05			 106
#define    PA_06			 107
#define    PA_07			 108
#define    PA_08			 109
#define    PA_09			 110
#define    PA_10			 111
#define    PA_11			 112
#define    PA_12			 113
#define    PA_13			 114
#define    PA_14			 115
#define    PA_15			 116

//PB管脚名称
#define    PB_00			 201
#define    PB_01			 202
#define    PB_02			 203
#define    PB_03			 204
#define    PB_04			 205
#define    PB_05			 206
#define    PB_06			 207
#define    PB_07			 208
#define    PB_08			 209
#define    PB_09			 210
#define    PB_10			 211
#define    PB_11			 212
#define    PB_12			 213
#define    PB_13			 214
#define    PB_14			 215
#define    PB_15			 216

//PC管脚名称
#define    PC_00			 301
#define    PC_01			 302
#define    PC_02			 303
#define    PC_03			 304
#define    PC_04			 305
#define    PC_05			 306
#define    PC_06			 307
#define    PC_07			 308
#define    PC_08			 309
#define    PC_09			 310
#define    PC_10			 311
#define    PC_11			 312
#define    PC_12			 313
#define    PC_13			 314
#define    PC_14			 315
#define    PC_15			 316

//PD管脚名称
#define    PD_00			 401
#define    PD_01			 402
#define    PD_02			 403
#define    PD_03			 404
#define    PD_04			 405
#define    PD_05			 406
#define    PD_06			 407
#define    PD_07			 408
#define    PD_08			 409
#define    PD_09			 410
#define    PD_10			 411
#define    PD_11			 412
#define    PD_12			 413
#define    PD_13			 414
#define    PD_14			 415
#define    PD_15			 416

//PE管脚名称
#define    PE_00			 501
#define    PE_01			 502
#define    PE_02			 503
#define    PE_03			 504
#define    PE_04			 505
#define    PE_05			 506
#define    PE_06			 507
#define    PE_07			 508
#define    PE_08			 509
#define    PE_09			 510
#define    PE_10			 511
#define    PE_11			 512
#define    PE_12			 513
#define    PE_13			 514
#define    PE_14			 515
#define    PE_15			 516

//PF管脚名称
#define    PF_00			 601
#define    PF_01			 602
#define    PF_02			 603
#define    PF_03			 604
#define    PF_04			 605
#define    PF_05			 606
#define    PF_06			 607
#define    PF_07			 608
#define    PF_08			 609
#define    PF_09			 610
#define    PF_10			 611
#define    PF_11			 612
#define    PF_12			 613
#define    PF_13			 614
#define    PF_14			 615
#define    PF_15			 616

//PG管脚名称
#define    PG_00			 701
#define    PG_01			 702
#define    PG_02			 703
#define    PG_03			 704
#define    PG_04			 705
#define    PG_05			 706
#define    PG_06			 707
#define    PG_07			 708
#define    PG_08			 709
#define    PG_09			 710
#define    PG_10			 711
#define    PG_11			 712
#define    PG_12			 713
#define    PG_13			 714
#define    PG_14			 715
#define    PG_15			 716

#ifdef __cplusplus
}
#endif

#endif

#endif

