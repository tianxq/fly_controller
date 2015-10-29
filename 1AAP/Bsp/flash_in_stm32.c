
#include "includes.h"

#define  FLASH_STARTADDR  0x0801F800    
volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;   

void ReadFlashNBtye(uint32_t ReadAddress, uint8_t *ReadBuf, uint8_t ReadNum)  
{  
	int DataNum = 0;  
	ReadAddress = (uint32_t)ReadAddress;   
	while(DataNum < ReadNum)   
	{   
		*(ReadBuf + DataNum) = *(__IO uint8_t*) ReadAddress++;   
		DataNum++;  
	}  
} 
	
void WriteFlashNBtye(uint32_t WriteAddress,uint8_t *WriteBuf,uint8_t WriteNum) 
{
	uint32_t r1;
	uint32_t EraseCounter = 0x00, Address = 0x00;//????,????
	uint32_t NbrOfPage = 0x00;//????????
	
	if(WriteNum%4!=0)
	{
		WriteNum=WriteNum/4 +1;
	}
	else
	{
		WriteNum=WriteNum/4;
	}
	NbrOfPage = WriteNum / FLASH_PAGE_SIZE+1;
	FLASH_Unlock();    
	
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR |  FLASH_FLAG_WRPRTERR);


 for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	{
		FLASHStatus = FLASH_ErasePage(WriteAddress + (FLASH_PAGE_SIZE * EraseCounter));
	}
	
	while(WriteNum--)
	{
		r1=*(WriteBuf++);
		r1|=*(WriteBuf++)<<8;
		r1|=*(WriteBuf++)<<16;
		r1|=*(WriteBuf++)<<24;
		FLASH_ProgramWord(WriteAddress, r1);
		WriteAddress+=4;
	}
 
	FLASH_Lock();  
}

