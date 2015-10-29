#define  STARTADDR  0x0801F800   //主存的第126页首地址 

void ReadFlashNBtye(uint32_t ReadAddress, uint8_t *ReadBuf, uint8_t ReadNum);
void WriteFlashNBtye(uint32_t WriteAddress,uint8_t *WriteBuf,uint8_t WriteNum);
