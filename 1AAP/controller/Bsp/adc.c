#include "includes.h"

uint16_t AD_Value[AVERAGE_FILTER_BUFFER_SIZE][CHANNEL_NUM];

static averaget_filter_t ADC_Filter[7];

uint16_t CtrlSticksLevel[7];

void stm32_adc1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStruct;
  DMA_InitTypeDef DMA_InitStructure;

	//Init PC4 PC5,used for keyboard
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	//Initialize the PA0 and PA1 using as adc input, one of them is connect to
	//height control stick and the other one is connect to camera pitch control stick
	GPIO_InitStructure.GPIO_Pin = VERT_LEFT | HORZ_LEFT|VERT_RIGHT|HORZ_RIGHT|
																CAMERA_PITH|NC_ADC1|BATTEY_CHECK;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

	RCC_ADCCLKConfig(RCC_PCLK2_Div8); //72M/8=9,ADC²»ÄÜ³¬¹ý14M
	/* Initialize the ADC */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_DeInit(ADC1);
	/* Initialize the ADC_Mode member */
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	/* initialize the ADC_ScanConvMode member */
	ADC_InitStruct.ADC_ScanConvMode = ENABLE;
	/* Initialize the ADC_ContinuousConvMode member */
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
	/* Initialize the ADC_ExternalTrigConv member */
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	/* Initialize the ADC_DataAlign member */
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	/* Initialize the ADC_NbrOfChannel member */
	ADC_InitStruct.ADC_NbrOfChannel = CHANNEL_NUM;

	ADC_Init( ADC1, &ADC_InitStruct);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_239Cycles5 );  //VERT_LEFT
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 2, ADC_SampleTime_239Cycles5 );  //HORZ_LEFT
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 3, ADC_SampleTime_239Cycles5 ); //VERT_RIGHT
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 4, ADC_SampleTime_239Cycles5 ); //HORZ_RIGHT
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_239Cycles5 ); //CAMERA_PITH
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 6, ADC_SampleTime_239Cycles5 ); //NC_ADC1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 7, ADC_SampleTime_239Cycles5 ); //BATTEY_CHECK

	ADC_DMACmd(ADC1, ENABLE);

	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));

	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

    /* Configuration the dma channel using transmittion beteewn adc register and ram space.*/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel1);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR; //
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Value; //
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //
    DMA_InitStructure.DMA_BufferSize = CHANNEL_NUM * AVERAGE_FILTER_BUFFER_SIZE; //
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; //
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //
    DMA_Init(DMA1_Channel1, &DMA_InitStructure); //

    ADC_DMACmd(ADC1, ENABLE);

    DMA_Cmd(DMA1_Channel1, ENABLE);

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}



/**
 * This function is a simple filter arithmetic operation using recursive average algorithm, which can be used for
 * filtering the adc sample values.
 */
 uint16_t average_filter(uint16_t * filter)
 {
	 uint8_t i;
	 uint16_t result;
	 uint16_t min,max;

#if 1   //remove the max and the min number???
    
	 /* find out the max and min number, and remove them */
     for(i=0,min = filter[i],max = filter[i];i<AVERAGE_FILTER_BUFFER_SIZE;i++){
         if(filter[i] < min)
           min = filter[i];

         if(filter[i] > max)
           max = filter[i];
     }
#endif
	 /* Sum the all numbers */
     for(i=0,result=0;i<AVERAGE_FILTER_BUFFER_SIZE;i++){
         result = (uint16_t)(result + filter[i]);
     }

     result = (uint16_t)(result -(min+max));//sub the max one and min one


     result = (uint16_t)(result /(AVERAGE_FILTER_BUFFER_SIZE-2)); //get the average value of the 8 number

     result &= ~0xf000;


     return result;
}


/**
 * Calculate the voltage of the battery
 * @param  adc_value [the result value of the adc convert]
 * @return           [the battery voltage in V unit]
 */
float Calculate_BatteryVoltage(uint16_t adc_value)
{
   return (float)((adc_value/0xfff)*2.1f);
}

