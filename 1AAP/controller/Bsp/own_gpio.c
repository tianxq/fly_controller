#include "own_gpio.h"
#if GPIO_ENABLE 

//�ܽ���Ϣ�ṹ��
typedef struct 	{
	GPIO_TypeDef *GPIOx;			//GPIO�ܽ���
	uint16_t GPIO_Pin;				//GPIO�Ĺܽź�
	uint32_t RCC_APB2Periph_GPIOx;	//GPIO��ʱ��
}STRUCT_GPIO_INFO;

/*********************************************************************************************************
** ��������: GPIO_Info
** ��������: ���ݹܽ�����(eg:PA_00)������ܽ���͹ܽű��
** ��ڲ���: pin_name�ܽŵ�����(eg:PA_00) 
** �� �� ֵ: GPIO_INFO
** ˵    ��: ���ݹܽŵ����ƣ����عܽ���Ϣ�ṹ�� 
** ʹ�����ӣ�GPIO_INFO gpio_info=GPIO_Info( PA_00);
*********************************************************************************************************/
void Func_GPIO_Info(pinname_type const pin_name,STRUCT_GPIO_INFO *gpio_info)
{
	GPIO_TypeDef *gpiox;							//����GPIO��
	uint16_t gpio_pin = 0x0;							//����ܽű�� 
	uint32_t rcc_apb2periph_gpiox=0X0;
	uint8_t pin_group = pin_name/100;					//����ܽ���	1-7=A-G
	uint8_t pin_num = (pin_name%100%17)-1;			//����ܽű��	1-16=00-15

	//����GPIO��
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
** ��������: GPIODeInit
** ��������: ����ȫ������Ϊȱʡֵ
** ��ڲ���: ��
** �� �� ֵ: ��
** ˵    ��: ��������ʼ�����е���������
** ʹ�����ӣ�GPIODeInit();
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
** ��������: GPIOAFIODeInit
** ��������: �����ù��ܣ���ӳ���¼����ƺ�EXTI���ã�����Ϊȱʡֵ
** ��ڲ���: ��
** �� �� ֵ: ��
** ˵    ��: �����ù��ܣ���ӳ���¼����ƺ�EXTI���ã�����Ϊȱʡֵ
** ʹ�����ӣ�GPIODeInit();
*********************************************************************************************************/
void GPIOAFIODeInit(void)
{
	GPIO_AFIODeInit();		
}

/*********************************************************************************************************
** ��������: GPIOInit
** ��������: ���ų�ʼ��
** ��ڲ���: pin_name �ܽ�����
			 pin_mode �ܽ�ģʽ
			 PA_00_SPEED �ܽŵ��ٶ�
** �� �� ֵ: ��
** ˵    ��: ��������ʼ�����е��������ã�����������ӡ���������������
** ʹ�����ӣ�GPIOInit(PA_00, PA_00_MODE, PA_00_SPEED);
*********************************************************************************************************/
void GPIOInit(pinname_type const pin_name,pinmode_type const pin_mode,pinspeed_type const pin_speed)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	STRUCT_GPIO_INFO gpio_info;

	Func_GPIO_Info(pin_name,&gpio_info);
	
	RCC_APB2PeriphClockCmd(gpio_info.RCC_APB2Periph_GPIOx, ENABLE); // ʹ��GPIOʱ��
	GPIO_InitStructure.GPIO_Pin = gpio_info.GPIO_Pin;				//�ܽű������
	GPIO_InitStructure.GPIO_Mode = pin_mode;		//ģʽ����
	GPIO_InitStructure.GPIO_Speed = pin_speed;	//�ٶ�����	
	
	GPIO_Init(gpio_info.GPIOx, &GPIO_InitStructure);				//GPIO��ʼ��		
}

/*********************************************************************************************************
** ��������: GPIOSet
** ��������: ������"1"
** ��ڲ���: pin_name �ܽ����� eg:PA_05
** �� �� ֵ: ��
** ˵    ��: ��������Ӧ����������Ϊ�ߵ�ƽ
** ʹ�����ӣ�GPIOSet(PA_05);
*********************************************************************************************************/
void GPIOSet(pinname_type const pin_name)
{
	STRUCT_GPIO_INFO gpio_info;

	Func_GPIO_Info(pin_name,&gpio_info);		//��ȡ�ܽ���Ϣ
	gpio_info.GPIOx->BSRR = gpio_info.GPIO_Pin;	//
}

/*********************************************************************************************************
** ��������: void GPIOClr(uint8_t pin_name)
** ��������: ������"0"
** ��ڲ���: pin_name �ܽ����� eg:PA_00
** �� �� ֵ: ��
** ˵    ��: ��������Ӧ����������Ϊ�͵�ƽ
** ʹ�����ӣ�GPIOClr(PA_00);
*********************************************************************************************************/
void GPIOClr(pinname_type const pin_name)
{
	STRUCT_GPIO_INFO gpio_info;

	Func_GPIO_Info(pin_name,&gpio_info);
	gpio_info.GPIOx->BRR = gpio_info.GPIO_Pin;			
}

/*********************************************************************************************************
** ��������: void GPIOInputGet(uint8_t pin_name)
** ��������: ��ȡ��Ӧ�ܽŵĵ�ƽ ��=1 ��=0
** ��ڲ���: pin_name �ܽ����� eg:PA_00
** �� �� ֵ: ��
** ˵    ��: ��������ȡ��Ӧ�����ŵĵ�ƽ
** ʹ������: GPIOGet(PA_00);
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
** ��������: void GPIOOutputGet(uint8_t pin_name)
** ��������: ��ȡ��Ӧ�ܽŵĵ�ƽ ��=1 ��=0
** ��ڲ���: pin_name �ܽ����� eg:PA_00
** �� �� ֵ: ��
** ˵    ��: ��������ȡ��Ӧ�����ŵĵ�ƽ
** ʹ������: GPIOGet(PA_00);
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
** ��������: void GPIONeg(uint8_t pin_name)
** ��������: ��Ӧ�ܽŵĵ�ƽȡ��
** ��ڲ���: pin_name �ܽ����� eg:PA_00
** �� �� ֵ: ��
** ˵    ��: ��������ȡ��Ӧ�����ŵĵ�ƽ
** ʹ������: GPIONeg(PA_00);
*********************************************************************************************************/
void GPIONeg(pinname_type const pin_name)
{
	STRUCT_GPIO_INFO gpio_info;

	Func_GPIO_Info(pin_name,&gpio_info);
	gpio_info.GPIOx->ODR ^= gpio_info.GPIO_Pin;		
}
#endif
