#define  HIDmask_address_start  0x0801F800   //主存的第126页首地址 
#define FLASH_PAGE_SIZE		0x400
typedef   enum 
{  
	HIDmask_address_boot=0, 
	HIDmask_address_adcinversion=24+FLASH_PAGE_SIZE, //反转
	HIDmask_address_calibration=0+FLASH_PAGE_SIZE,  //校准(H/L)
	HIDmask_address_amjp=30+FLASH_PAGE_SIZE,        //日本手美国手  
	 
}HIDmask_address; 

void ReadFlashNBtye(uint32_t ReadAddress, uint8_t *ReadBuf, uint8_t ReadNum);
void WriteFlashNBtye(uint32_t WriteAddress,uint8_t *WriteBuf,uint8_t WriteNum);
