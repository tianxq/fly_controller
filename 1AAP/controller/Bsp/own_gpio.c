#include "own_gpio.h"
#if GPIO_ENABLE 

//管脚信息结构体
typedef struct 	{
	GPIO_TypeDef *GPIOx;			//GPIO管脚组
	uint16_t GPIO_Pin;				//GPIO的管脚号
	uint32_t RCC_APB2Periph_GPIOx;	//GPIO的时钟
}STRUCT_GPIO_INFO;

/*********************************************************************************************************
** 函数名称: GPIO_Info
** 函数功能: 根据管脚名称(eg:PA_00)，计算管脚组和管脚编号
** 入口参数: pin_name管脚的名称(eg:PA_00) 
** 返 回 值: GPIO_INFO
** 说    明: 根据管脚的名称，返回管脚信息结构体 
** 使用例子：GPIO_INFO gpio_info=GPIO_Info( PA_00);
*********************************************************************************************************/
void Func_GPIO_Info(pinname_type const pin_name,STRUCT_GPIO_INFO *gpio_info)
{
	GPIO_TypeDef *gpiox;							//定义GPIO组
	uint16_t gpio_pin = 0x0;							//定义管脚编号 
	uint32_t rcc_apb2periph_gpiox=0X0;
	uint8_t pin_group = pin_name/100;					//计算管脚组	1-7=A-G
	uint8_t pin_num = (pin_name%100%17)-1;			//计算管脚编号	1-16=00-15

	//计算GPIO组
	if(pin_group==1){
		gpiox = GPIOA;	
		rcc_apb2periph_gpiox = RCC_APB2Periph_GPIOA;
	}					
	else if(pin_group==2){
		gpiox = GPIOB;	
		rcc_apb2periph_gpiox = RCC_APB2Periph_GPIOB;
	} 
	else if(pin_group==3){
		gpiox = GPIOC;	
		rcc_apb2periph_gpiox = RCC_APB2Periph_GPIOC;
	}
	else if(pin_group==4){
		gpiox = GPIOD;	
		rcc_apb2periph_gpiox = RCC_APB2Periph_GPIOD;
	}
	else if(pin_group==5){
		gpiox = GPIOE;	
		rcc_apb2periph_gpiox = RCC_APB2Periph_GPIOE;
	}
	else if(pin_group==6){
		gpiox = GPIOF;	
		rcc_apb2periph_gpiox = RCC_APB2Periph_GPIOF;
	}
	else if(pin_group==7){
		gpiox = GPIOG;	
		rcc_apb2periph_gpiox = RCC_APB2Periph_GPIOG;
	}

	gpio_pin = (((uint16_t)(0X01))<<pin_num);

	gpio_info->GPIOx = gpiox;
	gpio_info->GPIO_Pin = gpio_pin;
	gpio_info->RCC_APB2Periph_GPIOx = rcc_apb2periph_gpiox;
		
}
/*********************************************************************************************************
** 函数名称: GPIODeInit
** 函数功能: 引脚全部设置为缺省值
** 入口参数: 无
** 返 回 值: 无
** 说    明: 函数将初始化所有的引脚配置
** 使用例子：GPIODeInit();
*********************************************************************************************************/
void GPIODeInitAll(void)
{
	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);
	GPIO_DeInit(GPIOD);
	GPIO_DeInit(GPIOE);
	GPIO_DeInit(GPIOF);
	GPIO_DeInit(GPIOG);		
}

/*********************************************************************************************************
** 函数名称: GPIOAFIODeInit
** 函数功能: 将复用功能（重映射事件控制和EXTI设置）重设为缺省值
** 入口参数: 无
** 返 回 值: 无
** 说    明: 将复用功能（重映射事件控制和EXTI设置）重设为缺省值
** 使用例子：GPIODeInit();
*********************************************************************************************************/
void GPIOAFIODeInit(void)
{
	GPIO_AFIODeInit();		
}

/*********************************************************************************************************
** 函数名称: GPIOInit
** 函数功能: 引脚初始化
** 入口参数: pin_name 管脚名称
			 pin_mode 管脚模式
			 PA_00_SPEED 管脚的速度
** 返 回 值: 无
** 说    明: 函数将初始化所有的引脚配置，完成引脚连接、上下拉电阻设置
** 使用例子：GPIOInit(PA_00, PA_00_MODE, PA_00_SPEED);
*********************************************************************************************************/
void GPIOInit(pinname_type const pin_name,pinmode_type const pin_mode,pinspeed_type const pin_speed)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	STRUCT_GPIO_INFO gpio_info;

	Func_GPIO_Info(pin_name,&gpio_info);
	
	RCC_APB2PeriphClockCmd(gpio_info.RCC_APB2Periph_GPIOx, ENABLE); // 使能GPIO时钟
	GPIO_InitStructure.GPIO_Pin = gpio_info.GPIO_Pin;				//管脚编号配置
	GPIO_InitStructure.GPIO_Mode = pin_mode;		//模式配置
	GPIO_InitStructure.GPIO_Speed = pin_speed;	//速度配置	
	
	GPIO_Init(gpio_info.GPIOx, &GPIO_InitStructure);				//GPIO初始化		
}

/*********************************************************************************************************
** 函数名称: GPIOSet
** 函数功能: 引脚置"1"
** 入口参数: pin_name 管脚名称 eg:PA_05
** 返 回 值: 无
** 说    明: 函数将对应的引脚配置为高电平
** 使用例子：GPIOSet(PA_05);
*********************************************************************************************************/
void GPIOSet(pinname_type const pin_name)
{
	STRUCT_GPIO_INFO gpio_info;

	Func_GPIO_Info(pin_name,&gpio_info);		//获取管脚信息
	gpio_info.GPIOx->BSRR = gpio_info.GPIO_Pin;	//
}

/*********************************************************************************************************
** 函数名称: void GPIOClr(uint8_t pin_name)
** 函数功能: 引脚置"0"
** 入口参数: pin_name 管脚名称 eg:PA_00
** 返 回 值: 无
** 说    明: 函数将对应的引脚配置为低电平
** 使用例子：GPIOClr(PA_00);
*********************************************************************************************************/
void GPIOClr(pinname_type const pin_name)
{
	STRUCT_GPIO_INFO gpio_info;

	Func_GPIO_Info(pin_name,&gpio_info);
	gpio_info.GPIOx->BRR = gpio_info.GPIO_Pin;			
}

/*********************************************************************************************************
** 函数名称: void GPIOInputGet(uint8_t pin_name)
** 函数功能: 读取对应管脚的电平 高=1 低=0
** 入口参数: pin_name 管脚名称 eg:PA_00
** 返 回 值: 无
** 说    明: 函数将读取对应的引脚的电平
** 使用例子: GPIOGet(PA_00);
*********************************************************************************************************/
uint8_t GPIOInputGet(pinname_type const pin_name)
{
	uint8_t bitstatus=0x0; 
	STRUCT_GPIO_INFO gpio_info;

	Func_GPIO_Info(pin_name,&gpio_info);
	
	if ((gpio_info.GPIOx->IDR & gpio_info.GPIO_Pin) != (uint8_t)Bit_RESET)
	{
		bitstatus = (uint8_t)Bit_SET;
	}
	else
	{
		bitstatus = (uint8_t)Bit_RESET;
	}
	return bitstatus;		
}

/*********************************************************************************************************
** 函数名称: void GPIOOutputGet(uint8_t pin_name)
** 函数功能: 读取对应管脚的电平 高=1 低=0
** 入口参数: pin_name 管脚名称 eg:PA_00
** 返 回 值: 无
** 说    明: 函数将读取对应的引脚的电平
** 使用例子: GPIOGet(PA_00);
*********************************************************************************************************/
uint8_t GPIOOutputGet(pinname_type const pin_name)
{
	uint8_t bitstatus=0x0; 
	STRUCT_GPIO_INFO gpio_info;

	Func_GPIO_Info(pin_name,&gpio_info);
	
	if ((gpio_info.GPIOx->ODR & gpio_info.GPIO_Pin) != (uint8_t)Bit_RESET)
	{
		bitstatus = (uint8_t)Bit_SET;
	}
	else
	{
		bitstatus = (uint8_t)Bit_RESET;
	}
	return bitstatus;		
}

/*********************************************************************************************************
** 函数名称: void GPIONeg(uint8_t pin_name)
** 函数功能: 对应管脚的电平取反
** 入口参数: pin_name 管脚名称 eg:PA_00
** 返 回 值: 无
** 说    明: 函数将读取对应的引脚的电平
** 使用例子: GPIONeg(PA_00);
*********************************************************************************************************/
void GPIONeg(pinname_type const pin_name)
{
	STRUCT_GPIO_INFO gpio_info;

	Func_GPIO_Info(pin_name,&gpio_info);
	gpio_info.GPIOx->ODR ^= gpio_info.GPIO_Pin;		
}
#endif
