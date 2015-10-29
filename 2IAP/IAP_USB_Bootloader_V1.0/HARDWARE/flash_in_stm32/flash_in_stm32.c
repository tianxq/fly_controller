
#include "stm32_config.h"

  
volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;   

void ReadFlashNBtye(uint32_t ReadAddress, uint8_t *ReadBuf, uint8_t ReadNum)  
{  
	int DataNum = 0;  
	ReadAddress = (uint32_t)STARTADDR + ReadAddress;   
	while(DataNum < ReadNum)   
	{   
		*(ReadBuf + DataNum) = *(__IO uint8_t*) ReadAddress++;   
		DataNum++;  
	}  
} 
	
void WriteFlashNBtye(uint32_t WriteAddress,uint8_t *WriteBuf,uint8_t WriteNum) 
{
	uint32_t r1;
	WriteNum=WriteNum/4;
	FLASH_Unlock();    
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR |  FLASH_FLAG_WRPRTERR);    
	FLASHStatus = FLASH_ErasePage(STARTADDR);//²Á³ýÒ»Ò³  
	
	while(WriteNum--)
	{
		r1=*(WriteBuf++);
		r1|=*(WriteBuf++)<<8;
		r1|=*(WriteBuf++)<<16;
		r1|=*(WriteBuf++)<<24;
		FLASH_ProgramWord(STARTADDR + WriteAddress, r1);
		WriteAddress+=4;
	}
 
	FLASH_Lock();  
}

