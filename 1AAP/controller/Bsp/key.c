#include "includes.h"

KeyStatusTypeDef keyAst;
KeyStatusTypeDef keyBst;
KeyStatusTypeDef keyOKFst;
KeyStatusTypeDef keyHOMEst;
KeyStatusTypeDef keyPHOTOst;
KeyStatusTypeDef keyVIDEOst;

/******************************扫描方式* 按键初始化 *************************/
void key_IO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
//	EXTI_InitTypeDef EXTI_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	
	//RCC_APB2PeriphClockCmd( KEY_RCC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd( KEY_RCC , ENABLE);
 
	GPIO_InitStructure.GPIO_Pin = KEY_OKF; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = KEY_A|KEY_NEWHAND|KEY_AE|KEY_VIO; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //
	GPIO_Init(GPIOB,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = KEY_HOME|KEY_PHOTO; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	GPIO_InitStructure.GPIO_Pin = KEY_B; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //
	GPIO_Init(GPIOB,&GPIO_InitStructure); 
/*
	EXTI_ClearITPendingBit(EXTI_Line2);
	EXTI_ClearITPendingBit(EXTI_Line3);
	EXTI_ClearITPendingBit(EXTI_Line13);
	EXTI_ClearITPendingBit(EXTI_Line14);
	EXTI_ClearITPendingBit(EXTI_Line15);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource2);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource15);

	EXTI_InitStructure.EXTI_Line = EXTI_Line2|EXTI_Line3|EXTI_Line13|EXTI_Line14|EXTI_Line15; 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //中断请求，非事件请求
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //上下沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;        
	EXTI_Init(&EXTI_InitStructure);	
	

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);          
        
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;     //选择中断通道0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占式通道优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //响应式中断优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;     //选择中断通道0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占式通道优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //响应式中断优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;     //选择中断通道0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占式通道优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //响应式中断优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
	NVIC_Init(&NVIC_InitStructure);
*/

	keyAst.state=NoKeyDownStatus;
	keyAst.TimeCount=0;
	keyBst.state=NoKeyDownStatus;
	keyBst.TimeCount=0;
	keyOKFst.state=NoKeyDownStatus;
	keyOKFst.TimeCount=0;
	keyHOMEst.state=NoKeyDownStatus;
	keyHOMEst.TimeCount=0;
	keyPHOTOst.state=NoKeyDownStatus;
	keyPHOTOst.TimeCount=0;
	keyVIDEOst.state=NoKeyDownStatus;
	keyVIDEOst.TimeCount=0;
}

/***************************中断方式* 按键初始化 *************************/

void key_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd( KEY_RCC | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = KEY_OKF; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = KEY_A|KEY_NEWHAND|KEY_AE|KEY_VIO; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //
	GPIO_Init(GPIOB,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = KEY_HOME|KEY_PHOTO; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	
	//JTAG口关闭
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	GPIO_InitStructure.GPIO_Pin = KEY_B; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //
	GPIO_Init(GPIOB,&GPIO_InitStructure); 
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);          
        
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;     //选择中断通道0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占式通道优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //响应式中断优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;     //选择中断通道0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占式通道优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //响应式中断优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;     //选择中断通道0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占式通道优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //响应式中断优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
	NVIC_Init(&NVIC_InitStructure);
	
	EXTI_ClearITPendingBit(EXTI_Line2);
	EXTI_ClearITPendingBit(EXTI_Line3);
	EXTI_ClearITPendingBit(EXTI_Line13);
	EXTI_ClearITPendingBit(EXTI_Line14);
	EXTI_ClearITPendingBit(EXTI_Line15);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource2);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);

	EXTI_InitStructure.EXTI_Line = EXTI_Line2|EXTI_Line3|EXTI_Line13|EXTI_Line14|EXTI_Line15; 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //中断请求，非事件请求
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //上下沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;        
	EXTI_Init(&EXTI_InitStructure);	

}
/************************************************************* 
*Name  :StateStatus  ReadKeyStatus(void) 
*Function :扫描键盘
*Input    :
*************************************************************/ 
KeyStatus  ReadKeyStatus(GPIO_TypeDef* keyGPIO,uint16_t keyGPIOPin,
												KeyStatusTypeDef* key) 
{   
	int  KeyPress = GPIO_ReadInputDataBit(keyGPIO,keyGPIOPin);   
	KeyStatus  KeyReturn = NoKeyDownStatus; 
	
	switch(key->state)  
	{   
		case  NoKeyDownStatus: 
			if(!KeyPress)     
			{       
				key->state = KeySureDownStatus;      
			}      
				break;     
				
		case KeySureDownStatus:    
			if(!KeyPress)    
			{      
				key->state = OnceKeyDownStatus;        
				key->TimeCount = 0;     
			}    
			else       
			{ 
				key->state = NoKeyDownStatus;      
			}       
			break;

		case OnceKeyDownStatus:    
			if(KeyPress) //10ms消除抖动 短按
			{      
				key->state = NoKeyDownStatus;        
				KeyReturn = OnceKeyDownStatus;     
			}     
			/*-按超过0.5s进入长按-*/ 
			else if(++key->TimeCount>=10)     
			{      
				key->state = ContiousKeyDownStatus;       
				key->TimeCount = 0;        
				//KeyReturn = ContiousKeyDownStatus;  
				KeyReturn = NoKeyDownStatus;
				motostate=1;
			}      
			break; 

			case ContiousKeyDownStatus:
				//motostate=1;				
				if(KeyPress)    
				{      
					key->state = NoKeyDownStatus;        
					//KeyReturn = NoKeyDownStatus;
					KeyReturn = ContiousKeyDownStatus;
					//motostate=0;
				}     
				/*-循环持续长按判断，一直按不停-*/   
				else if(++key->TimeCount>10)    
				{              
					//KeyReturn = ContiousKeyDownStatus;
					KeyReturn = NoKeyDownStatus;
					key->TimeCount = 0; 
				}  
				/**/       
				else        
				{ 
					KeyReturn = NoKeyDownStatus;  
				}     
				break;   
		}   
		return KeyReturn;  
}

//timer回调函数
void timekey_callback(void *ptmr, void *parg)
{
	motostate=1;//长按超时响马达一次
}

