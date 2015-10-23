/**********************************************************
* @ File name -> soft_spi.h
* @ Version   -> V1.0
* @ Date      -> 11-15-2013
* @ Brief     -> GPIO模拟SPI接口函数头文件

 V1.
* @ Revise    -> 
**********************************************************/

#ifndef _soft_spi_h_
#define _soft_spi_h_

/**********************************************************
                     外部函数头文件                        
**********************************************************/

#include "config.h"

/**********************************************************
                      定义模拟接口
                所接的IO不一样请修改这里
    设置IO的方向请修改这里，对于不是具有准双向IO的MCU
**********************************************************/

#define Soft_SPI_CLK                 PAout(1)    //模拟SPI接口片选
#define Soft_SPI_MOSI               PAout(2)    //模拟SPI接口，主 -> 从
#define Soft_SPI_MISO               PAin(3) //模拟SPI接口，主 <- 从

//#define Soft_SPI_MISO_IN()          {GPIOA->CRL &= 0xffff0fff;GPIOA->CRL |= 8<<12;}

/**********************************************************
                        外部功能函数
**********************************************************/

void Soft_SPI_Init(void);   //模拟SPI初始化GPIO

u8 Soft_SPI_ReadWrite_Byte(u8 send_data);   //模拟SPI发送数据函数


#endif

