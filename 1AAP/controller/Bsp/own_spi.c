#include "own_spi.h"
#if SPI_ENABLE

void SPIx_SetSpeed(SPI_TypeDef * SPIx,uint8_t SpeedSet)
{	
	SpeedSet&=0X07;			//���Ʒ�Χ
	SPIx->CR1&=0XFFC7; 
	SPIx->CR1|=SpeedSet<<3;	//����SPIx�ٶ�  
	SPIx->CR1|=1<<6; 		//SPIx�豸ʹ��	  
} 
//��������ӿ�SPI�ĳ�ʼ����SPI���ó���ģʽ							  
void SPIx_Init(SPI_TypeDef * SPIx,
			   uint16_t Direction,
			   uint16_t Mode,
			   uint16_t DataSize,
			   uint16_t CPOL,
			   uint16_t CPHA,
			   uint16_t NSS,
			   uint16_t BaudRatePrescaler,
			   uint16_t FirstBit,
			   uint16_t CRCPolynomial)
{
	SPI_InitTypeDef  SPI_InitStructure;
	pinname_type SPIx_CS,SPIx_SCK,SPIx_MISO,SPIx_MOSI; 
	if(SPIx == SPI1)
	{
		RCC_APB2PeriphClockCmd(RCC_APBxPeriph_SPI1, ENABLE);
		SPIx_CS 	= 	SPI1_CS;
		SPIx_SCK	= 	SPI1_SCK;
		SPIx_MISO	=	SPI1_MISO;
		SPIx_MOSI	=	SPI1_MOSI;	
	}
	else if(SPIx == SPI2)
	{
		RCC_APB2PeriphClockCmd(RCC_APBxPeriph_SPI2, ENABLE);
		SPIx_CS 	= 	SPI2_CS;
		SPIx_SCK	= 	SPI2_SCK;
		SPIx_MISO	=	SPI2_MISO;
		SPIx_MOSI	=	SPI2_MOSI;	
	}
	else if(SPIx == SPI3)
	{
		RCC_APB2PeriphClockCmd(RCC_APBxPeriph_SPI3, ENABLE);
		SPIx_CS 	= 	SPI3_CS;
		SPIx_SCK	= 	SPI3_SCK;
		SPIx_MISO	=	SPI3_MISO;
		SPIx_MOSI	=	SPI3_MOSI;	
	}	
	

	GPIOInit(SPIx_CS,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
	GPIOSet(SPIx_CS);
	GPIOInit(SPIx_SCK,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
	GPIOInit(SPIx_MISO,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);
	GPIOInit(SPIx_MOSI,GPIO_Mode_AF_PP,GPIO_Speed_50MHz);

	/* SPI1 configuration */ 
	SPI_InitStructure.SPI_Direction = Direction; 					//SPI����Ϊ����ȫ˫��
	SPI_InitStructure.SPI_Mode = Mode;	                   			//����SPIΪ��ģʽ
	SPI_InitStructure.SPI_DataSize = DataSize;                  	//SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = CPOL;	 		               		//����ʱ���ڲ�����ʱ��ʱ��Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = CPHA;		               			//�ڶ���ʱ���ؿ�ʼ��������
	SPI_InitStructure.SPI_NSS = NSS;			               		//NSS�ź��������ʹ��SSIλ������
	SPI_InitStructure.SPI_BaudRatePrescaler = BaudRatePrescaler;	//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ8
	SPI_InitStructure.SPI_FirstBit = FirstBit;				   		//���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = CRCPolynomial;			//CRCֵ����Ķ���ʽ
	SPI_Init(SPIx, &SPI_InitStructure);
	/* Enable SPI1  */
	SPI_Cmd(SPIx, ENABLE);	

} 

//SPIx ��дһ���ֽ�
//����ֵ:��ȡ�����ֽ�
uint8_t SPIx_ReadWriteByte(SPI_TypeDef * SPIx,uint8_t TxData)
{		
	uint8_t retry=0;				 
	while((SPIx->SR&1<<1)==0)//�ȴ���������	
	{
		retry++;
		if(retry>200)return 0;
	}			  
	SPIx->DR=TxData;	 	  //����һ��byte 
	retry=0;
	while((SPIx->SR&1<<0)==0) //�ȴ�������һ��byte  
	{
		retry++;
		if(retry>200)return 0;
	}	  						    
	return SPIx->DR;          //�����յ�������				    
}

#endif

