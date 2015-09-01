#include "own_spi.h"
#if SPI_ENABLE

void SPIx_SetSpeed(SPI_TypeDef * SPIx,uint8_t SpeedSet)
{	
	SpeedSet&=0X07;			//限制范围
	SPIx->CR1&=0XFFC7; 
	SPIx->CR1|=SpeedSet<<3;	//设置SPIx速度  
	SPIx->CR1|=1<<6; 		//SPIx设备使能	  
} 
//串行外设接口SPI的初始化，SPI配置成主模式							  
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
	SPI_InitStructure.SPI_Direction = Direction; 					//SPI设置为两线全双工
	SPI_InitStructure.SPI_Mode = Mode;	                   			//设置SPI为主模式
	SPI_InitStructure.SPI_DataSize = DataSize;                  	//SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = CPOL;	 		               		//串行时钟在不操作时，时钟为高电平
	SPI_InitStructure.SPI_CPHA = CPHA;		               			//第二个时钟沿开始采样数据
	SPI_InitStructure.SPI_NSS = NSS;			               		//NSS信号由软件（使用SSI位）管理
	SPI_InitStructure.SPI_BaudRatePrescaler = BaudRatePrescaler;	//定义波特率预分频的值:波特率预分频值为8
	SPI_InitStructure.SPI_FirstBit = FirstBit;				   		//数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = CRCPolynomial;			//CRC值计算的多项式
	SPI_Init(SPIx, &SPI_InitStructure);
	/* Enable SPI1  */
	SPI_Cmd(SPIx, ENABLE);	

} 

//SPIx 读写一个字节
//返回值:读取到的字节
uint8_t SPIx_ReadWriteByte(SPI_TypeDef * SPIx,uint8_t TxData)
{		
	uint8_t retry=0;				 
	while((SPIx->SR&1<<1)==0)//等待发送区空	
	{
		retry++;
		if(retry>200)return 0;
	}			  
	SPIx->DR=TxData;	 	  //发送一个byte 
	retry=0;
	while((SPIx->SR&1<<0)==0) //等待接收完一个byte  
	{
		retry++;
		if(retry>200)return 0;
	}	  						    
	return SPIx->DR;          //返回收到的数据				    
}

#endif

