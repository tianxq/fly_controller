/**
  ******************************************************************************
  * @file    Smartcard/inc/smartcard.h
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    07/27/2009
  * @brief   This file contains all the functions prototypes for the Smartcard
  *          firmware library.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 20009 STMicroelectronics</center></h2>
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMARTCARD_H
#define __SMARTCARD_H

#ifdef __cplusplus
extern "C" {
#endif
    /* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
//#include "stm32f10x_gpio.h"
///* Global variables definition and initialization ----------------------------*/
//uint32_t __IO CardInserted = 0;//=0:û�п����룻=1���п�����
    /* Exported constants --------------------------------------------------------*/
#define T0_PROTOCOL        0x00  /* T0 protocol */
#define DIRECT             0x3B  /* Direct bit convention */
#define INDIRECT           0x3F  /* Indirect bit convention */
#define SETUP_LENGTH       20
#define HIST_LENGTH        20
#define LC_MAX             60
#define SC_RECEIVE_TIMEOUT 0x500  /* Direction to reader */
//#define SC_RECEIVE_TIMEOUT 0x80000000  /* Direction to reader */

    /* Define the STM32F10x hardware depending on the used evaluation board */
#define SC_USART                 USART1
#define SC_USART_GPIO            GPIOA
#define SC_USART_CLK             RCC_APB2Periph_USART1
#define SC_USART_GPIO_CLK        RCC_APB2Periph_GPIOA
#define SC_USART_PIN_TX          GPIO_Pin_9
#define SC_USART_PIN_CK          GPIO_Pin_8
#define SC_USART_IRQn            USART1_IRQn
#define SC_USART_IRQHandler      USART1_IRQHandler
    /* Smartcard Inteface GPIO pins */
//  #define SC_PIN_3_5V               GPIO_Pin_11
//  #define SC_PIN_3_5V_GPIO          GPIOD
//  #define SC_PIN_3_5V_GPIO_CLK      RCC_APB2Periph_GPIOD
    /* ISO7816_RST  GPIO pins */
#define SC_PIN_7816              GPIO_Pin_11|GPIO_Pin_12
#define SC_PIN_7816_GPIO         GPIOA
#define SC_PIN_7816_GPIO_CLK     RCC_APB2Periph_GPIOA
	
	
#define SC_PIN_RESET              GPIO_Pin_10
#define SC_PIN_RESET_GPIO         GPIOA
#define SC_PIN_RESET_GPIO_CLK     RCC_APB2Periph_GPIOA


/* SC Tree Structure -----------------------------------------------------------
                                  MasterFile
                               ________|___________
                              |        |           |
                            System   UserData     Note
    ------------------------------------------------------------------------------*/

///* SC ADPU Command: Operation Code -------------------------------------------*/
//#define SC_CLA_GSM11       0xA0
//
///*------------------------ Data Area Management Commands ---------------------*/
//#define SC_SELECT_FILE     0xA4
//#define SC_GET_RESPONCE    0xC0
//#define SC_STATUS          0xF2
//#define SC_UPDATE_BINARY   0xD6
//#define SC_READ_BINARY     0xB0
//#define SC_WRITE_BINARY    0xD0
//#define SC_UPDATE_RECORD   0xDC
//#define SC_READ_RECORD     0xB2
//
///*-------------------------- Administrative Commands -------------------------*/
//#define SC_CREATE_FILE     0xE0
//
///*-------------------------- Safety Management Commands ----------------------*/
//#define SC_VERIFY          0x20
//#define SC_CHANGE          0x24
//#define SC_DISABLE         0x26
//#define SC_ENABLE          0x28
//#define SC_UNBLOCK         0x2C
//#define SC_EXTERNAL_AUTH   0x82
//#define SC_GET_CHALLENGE   0x84

///*-------------------------- Answer to reset Commands ------------------------*/
//#define SC_GET_A2R         0x00
//
///* SC STATUS: Status Code ----------------------------------------------------*/
//#define SC_EF_SELECTED     0x9F
//#define SC_DF_SELECTED     0x9F
//#define SC_OP_TERMINATED   0x9000

    /* Smartcard type */
#define SC_PSAM_CARD      0
#define SC_USER_CARD      1

typedef enum{PSAM1,PSAM2,PSAM3,PSAM4,PSAM5,PSAM6}PsamSlot;			

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    SC_POWER_ON = 0x00,
    SC_RESET_LOW = 0x01,
    SC_RESET_HIGH = 0x02,
    SC_ACTIVE = 0x03,
    SC_ACTIVE_ON_T0 = 0x04,
    SC_POWER_OFF = 0x05
}SC_State;

typedef struct
{
    uint8_t TS;               /* Bit Convention TS���ǳ�ʼ�ַ�����ѡ���������ǵ�TS = 0x3BʱΪ�����䣨b1~b8��λ��ǰ����λ�ں󣩣���TS = 0x3FʱΪ�����䣨b8~b1��λ��ǰ����λ�ں�*/
    uint8_t T0;               /* High nibble = Number of setup byte; low nibble = Number of historical byte */
    uint8_t T[SETUP_LENGTH];  /* Setup array */
    uint8_t H[HIST_LENGTH];   /* Historical array */
    uint8_t Tlength;          /* Setup array dimension */
    uint8_t Hlength;          /* Historical array dimension */
} SC_ATR;


extern SC_State SCState[2] ;
extern SC_ATR SC_A2R;
/********************************************************************************
Header(����ͷ)������������������ Body(������)
CLA INS��P1��P2�������������� Lc Field�� Data Field�� Le Field
CLAΪָ����𡣾�����ο�ISO7816����ֲᡣ
INS��ָ����롣ͨ��ָ�������ʵ�ֿ��Ķ�д���µȲ���
P1��ָ�����1
P2��ָ�����2
�����ĸ��Ǳ�ѡ�ġ�
LC �������ֶεĳ���,ֻ��һ���ֽڱ�ʾ��ȡֵ��Χ1��100�� ��LC=0����ʾû��������
Data ��Lc������ָ������
LE ��������Ӧ���ݵ���󳤶�
7816 �й涨һ����������5���ֽ�
************************************************************************************/

/* ADPU-Header command structure ---------------------------------------------*/
typedef struct
{
    uint8_t CLA;  /* Command class */
    uint8_t INS;  /* Operation code */
    uint8_t P1;   /* Selection Mode */
    uint8_t P2;   /* Selection Option */
}SC_Header;

/* ADPU-Body command structure -----------------------------------------------*/
typedef struct
{
    uint8_t LC;           /* Data field length */
    uint8_t Data[LC_MAX];  /* Command parameters */
    uint8_t LE;           /* Expected length of data to be returned */
}SC_Body;

/* ADPU Command structure ----------------------------------------------------*/
typedef struct
{
    SC_Header Header;
    SC_Body Body;
}SC_ADPU_Commands;

/* SC response structure -----------------------------------------------------*/
typedef struct
{
    uint8_t Data[LC_MAX];  /* Data returned from the card */
    uint8_t SW1;          /* Command Processing status */
    uint8_t SW2;          /* Command Processing qualification */
} SC_ADPU_Responce;

/******************************�������� ********************************/
typedef struct
{
    uint8_t TranNo[4];		//�ն˽�����
    uint8_t TranDate[4];	//�������ڣ��նˣ�
    uint8_t TranTime[3];	//����ʱ�䣨�նˣ�
    uint8_t MAC[4];				//MAC1
}TRANDATA;

typedef struct
{
    uint8_t Tac[4];				//TAC
    uint8_t Mac[4];				//MAC2
}RESPTRANDATA;

typedef struct 			  //Ȧ���ʼ������
{
    uint8_t KeyID;			//��Կ������
    uint8_t Sum[4];			//���׽��
    uint8_t TerminalNO[6];//�ն˱��
}INITDATA;

typedef struct 			//Ȧ���ʼ����Ӧ����
{
    uint8_t OldAccount[4]; //ED/EP���
    uint8_t TranNo[2];		 //ED/EP�ѻ����׺�
    uint8_t WithdrawSum[3];//͸֧�޶�
    uint8_t KeyVer;				 //��Կ�汾��(DPK)
    uint8_t AlgoID;				 //�㷨��ʶ(DPK)
    uint8_t Random[4];		 //α�����
    uint8_t MAC1[4];			 //MAC1
}RESPINITDATA;

typedef struct 				//����״̬����:�淶�ڶ����ֱ�26����28
{
    uint8_t State;			//����״̬
    uint8_t Type;			 //��������
    uint8_t UpLockET;		 //�����������
    uint8_t ETAccount[4];	 //ET���
    uint8_t OfflineNo[2];	 //�ѻ����׺�
    uint8_t TerminalNO[6];	 //�����ն˱��
    uint8_t TranDate[4];	 //��������
    uint8_t TranTime[3];	 //����ʱ��
    uint8_t Sum[4];			 //MAC2
    uint8_t TAC[4];			 //GTAT2
}LOCKPROOFDATA;

typedef struct				//������۳�ʼ������
{
    uint8_t KeyID;			//��Կ�汾��
    uint8_t TerminalNO[6];	//�����ն˺�
}GREYINITDATA;

typedef struct 				//������ʼ����������
{
    uint8_t ETAccount[4];		//ED/EP���
    uint8_t OfflineNo[2];		//�ѻ����׺�
    uint8_t WithdrawSum[3];	//͸֧�޶�
    uint8_t KeyVer;			//��Կ�汾��
    uint8_t AlgoID;			//��Կ�㷨��ʶ
    uint8_t Random[4];		//α�����
}GREYRESPDATA;

typedef struct				//������۷�������
{
    uint8_t ETAccount[4];
    uint8_t OfflineNo[2];
    uint8_t LineNo[2];
    uint8_t KeyVer;
    uint8_t AlgoID;
    uint8_t Random[4];
    uint8_t MAC[4];
}UNLOCKINITRESPDATA;

typedef struct 			   //�����������	��������
{
    uint8_t Sum[4];		   //���׽��
    uint8_t TranDate[4];	   //��������
    uint8_t TranTime[3];	   //����ʱ��
    uint8_t MAC[4];		   //MAC2
}UNLOCKTRANDATA;

typedef struct 			  //�ѻ��������
{
    uint8_t Sum[4];
    uint8_t OfflineNo[2];
    uint8_t TerminalNO[6];
    uint8_t TranNo[4];
    uint8_t TranDate[4];
    uint8_t TranTime[3];
    uint8_t GMAC[4];
}OFFUNLOCKTRANDATA;

typedef struct 				//��������
{
    uint8_t TranNo[4];		//�ն˽��׺�
    uint8_t Random[4]; 		//�ն������
    uint8_t TranDate[4];	//��������
    uint8_t TranTime[3];	//����ʱ��
    uint8_t MAC[4];			//MAC
}GREYTRANDATA;

typedef struct 				//����MAC1���������
{
    uint8_t Random[4];		//�û��������
    uint8_t TranNo[2];		//�û������׺�
    uint8_t TranSum[4];		//���׽��
    uint8_t TranType;			//�������ͱ�ʶ��
    uint8_t TranDate[4];		//��������
    uint8_t TranTime[3];		//����ʱ��
    uint8_t PurKeyVer;		//������Կ�汾
    uint8_t PurAlgoID;		//������Կ�㷨��ʶ
    uint8_t CardSerialNo[8];	//�û����������к�
    uint8_t MemberBankID[8];	//�����б�ʶ
    uint8_t CityID[8];		//�Ե���б�ʶ
}INITSAMPUR,SAMMAC1INITDATA;

typedef struct 				//����MAC1,GMAC��Ӧ��������
{
    union
    {
        uint8_t OffLineNo[4];	    //�ն��ѻ����׺�
        uint8_t GMAC[4];
    }Res1;
    union
    {
        uint8_t Mac[4];			//MAC1
        uint8_t SAMTAC[4];
    } Res2;
}RESPINITSAM,SAMGMACRESDATA;

typedef struct 			   	//SAMGMAC��ʼ������
{
    uint8_t TranType;			//�������ͱ�ʶ
    uint8_t CardSerialNo[8];	//������Ʊ�������кţ���16λ��
    uint8_t OfflineNo[2];		//ET�ѻ��������к�
    uint8_t SUM[4];			//���׽��
} SAMGMACINITDATA;

typedef struct 				//GMAC��������
{
    uint8_t TranNo[4];	 	//���׺�
    uint8_t Random[4];		//�����
    uint8_t MAC[4];			//MAC
}SAMMAC1RESDATA;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
	
/***********************************************************************************
**�л�PSAM
************************************************************************************/
unsigned char SC_SwitchPSAM(PsamSlot slot);	
	
/* APPLICATION LAYER ---------------------------------------------------------*/
void SC_Handler(uint8_t CardType ,SC_State *SCState, SC_ADPU_Commands *SC_ADPU, SC_ADPU_Responce *SC_Response);
void SC_PowerCmd(FunctionalState NewState);
//void SC_Reset(BitAction ResetState);
void SC_DetectPinConfig(void);
void SC_ParityErrorHandler(void);
void SC_PTSConfig(void);
uint8_t cpu_reset(uint8_t CardType,uint8_t *rData);
uint16_t cpu_GetRamdom(uint8_t CardType ,uint8_t RndCount,uint8_t *rData);
uint16_t cpu_SelectFile(uint8_t CardType ,uint8_t Type,uint16_t FID,uint8_t *RevData);
uint16_t cpu_SelectFileByName(uint8_t CardType,uint8_t paraP2,uint8_t NameLen,uint8_t * FileName);
uint16_t cpu_AppendRecord(uint8_t CardType,uint8_t Fid,uint8_t DataLen,uint8_t *tData);
uint16_t cpu_ApplicationBlock(uint8_t CardType,uint8_t BlockType,uint8_t *Mac);
uint16_t cpu_AppUnblock(uint8_t CardType,uint8_t * Mac);
uint16_t cpu_ChangePin(uint8_t CardType,uint8_t PinId,uint8_t OldPinLen,uint8_t *OldPin,uint8_t NewPinLen,uint8_t *NewPin);
uint16_t cpu_DebitForPurchase(uint8_t CardType,TRANDATA tdata,RESPTRANDATA *rdata);
uint16_t cpu_CreditForLoad(uint8_t CardType,TRANDATA tdata,uint8_t *tac);
uint16_t cpu_InitForLoad(uint8_t CardType,uint8_t Type,INITDATA InitData,RESPINITDATA *rInitdata);
uint16_t cpu_ExteAuth(uint8_t CardType,uint8_t KeyId,uint8_t sDatalen,uint8_t *sRomd);
uint16_t cpu_GetBalance(uint8_t CardType,uint8_t Type,uint8_t *sum);
uint16_t cpu_GetTranProve(uint8_t CardType,uint8_t TranType,uint16_t TranNo,RESPTRANDATA * RespTranData);
uint16_t cpu_InitForPurchase(uint8_t CardType,uint8_t Type,INITDATA InitData,RESPINITDATA *RespInitData);
uint16_t cpu_InitForUnload(uint8_t CardType,INITDATA  InitData,RESPINITDATA * RespInitData);
uint16_t cpu_DebitForUnload(uint8_t CardType,TRANDATA TranData,RESPTRANDATA * RespTranData);
uint16_t cpu_InteAuth(uint8_t CardType,uint8_t KeyID,uint8_t sDataLen,uint8_t * sData,uint8_t * rDataLen,uint8_t * rData);
uint16_t cpu_ReadBin(uint8_t CardType,uint16_t Offset,uint8_t ReadLen,uint8_t * rData);
uint16_t cpu_GetResponse(uint8_t CardType,uint8_t RLen,uint8_t *rData);
uint16_t cpu_VerifyPin(uint8_t CardType,uint8_t KeyID,uint8_t PinLen,uint8_t * Pin);
uint16_t cpu_WriteBin(uint8_t CardType,uint8_t P1,uint8_t P2,uint8_t WriteLen,uint8_t * sData,uint8_t WriteMode);
uint16_t cpu_LockProof(uint8_t CardType,uint8_t ProofType,LOCKPROOFDATA *LockProofData);
uint16_t cpu_InitGreyUnLock(uint8_t CardType,uint8_t Type,GREYINITDATA GreyInitData,UNLOCKINITRESPDATA *UnLockInitRespData);
uint16_t cpu_Grey_UnLock(uint8_t CardType,UNLOCKTRANDATA UnLockTranData,uint8_t *UnLockRespData);
uint16_t cpu_DebitForUnLock(uint8_t CardType,uint8_t Type, OFFUNLOCKTRANDATA OffUnLockTranData,uint8_t *Tac);
uint16_t cpu_InitGreyLock(uint8_t CardType, uint8_t Type,GREYINITDATA GreyInitData, GREYRESPDATA *GreyRespData);
uint16_t cpu_GreyLock(uint8_t CardType,GREYTRANDATA GreyTranData,RESPTRANDATA *RespTranData);
uint16_t cpu_Crypt(uint8_t CardType,uint8_t CryptMode,uint8_t KeyID,uint8_t sDataLen,uint8_t * sData,uint8_t * rDataLen,uint8_t * rData);
uint16_t cpu_ReadRecord(uint8_t CardType,uint8_t SID,uint8_t RecordNo,uint8_t ReadLen,uint8_t * rData);
uint16_t cpu_UpdateRecord(uint8_t CardType,uint8_t SID,uint8_t RecordNo,uint8_t WriteLen,uint8_t * sData,uint8_t WriteMode);
uint16_t cpu_Cardlock(uint8_t CardType,uint8_t * sData);
uint16_t cpu_PSAM_DesCrypt(uint8_t CardType,uint8_t CryptMode,uint8_t sDataLen,uint8_t * sData,uint8_t *rDataLen,uint8_t * rData);
uint16_t cpu_PSAM_InitForDesCrypt(uint8_t CardType,uint8_t KeyType,uint8_t KeyID,uint8_t sDataLen,uint8_t *sData);
uint16_t cpu_Init_Sam_Grey_Lock(uint8_t CardType,uint8_t DivGrade,SAMMAC1INITDATA InitSamPur,RESPINITSAM *RespInitSam);
uint16_t cpu_PSAM_InitSamForPurchase(uint8_t CardType,uint8_t DivGrade,INITSAMPUR InitSamPur,SAMMAC1RESDATA *RespInitSam);
uint16_t cpu_Cert_Sam_Grey_Lock(uint8_t CardType,uint8_t *SamMac2);
uint16_t cpu_Credit_Sam_For_Grey_Debit(uint8_t CardType,SAMGMACINITDATA SamGmacInitData,SAMGMACRESDATA *SamGmacResData);
uint16_t cpu_SamGetGmac(uint8_t CardType,uint8_t * ResTranNo,SAMGMACRESDATA *SamGmac);
uint16_t Inter_Auth(uint8_t CardType,uint8_t KeyVer,uint8_t *str,uint8_t *RespData);
uint16_t cpu_Adpu(uint8_t *inData,uint8_t inLen,uint8_t *outData,uint8_t *outLen);

/*******************************************************************************
*����PSAM����
*******************************************************************************/
unsigned short GuoMiTripleAuth(unsigned char P1,unsigned char P2,unsigned char *txBuf,
								unsigned char txLen,unsigned char *rxBuf,unsigned char rxLen);

/*******************************************************************************
*��ȡ��Կ��
*******************************************************************************/
unsigned short SAMGetKeyliu(unsigned char P1,unsigned char P2,unsigned char *rlen,unsigned char *rdata);




#ifdef __cplusplus
}
#endif
#endif /* __SMARTCARD_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
