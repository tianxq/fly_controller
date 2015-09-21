/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* Company            : STMicroelectronics
* Author             : MCD Application Team
* Description        : STMicroelectronics Device Firmware Upgrade STMicroelectronics Extension Demo
* Version            : V3.0
* Date               : 07/02/2009
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#define STDFUFILES_ERROR_OFFSET				(0x12340000+0x6000)

#define STDFUFILES_NOERROR					(0x12340000+0x0000)
#define STDFUFILES_BADSUFFIX				(STDFUFILES_ERROR_OFFSET+0x0002)
#define STDFUFILES_UNABLETOOPENFILE			(STDFUFILES_ERROR_OFFSET+0x0003)
#define STDFUFILES_UNABLETOOPENTEMPFILE		(STDFUFILES_ERROR_OFFSET+0x0004)
#define STDFUFILES_BADFORMAT				(STDFUFILES_ERROR_OFFSET+0x0005)
#define STDFUFILES_BADADDRESSRANGE			(STDFUFILES_ERROR_OFFSET+0x0006)
#define STDFUFILES_BADPARAMETER				(STDFUFILES_ERROR_OFFSET+0x0008)
#define STDFUFILES_UNEXPECTEDERROR			(STDFUFILES_ERROR_OFFSET+0x000A)	
#define STDFUFILES_FILEGENERALERROR			(STDFUFILES_ERROR_OFFSET+0x000D)	

extern "C" DWORD PASCAL EXPORT STDFUFILES_OpenExistingDFUFile(PSTR pPathFile, PHANDLE phFile, PWORD pVid, PWORD pPid, PWORD pBcd,PBYTE pNbImages);
extern "C" DWORD PASCAL EXPORT STDFUFILES_CreateNewDFUFile(PSTR pPathFile, PHANDLE phFile, WORD Vid, WORD Pid, WORD Bcd);
extern "C" DWORD PASCAL EXPORT STDFUFILES_CloseDFUFile(HANDLE hFile);

extern "C" DWORD PASCAL EXPORT STDFUFILES_AppendImageToDFUFile(HANDLE hFile, HANDLE Image);
extern "C" DWORD PASCAL EXPORT STDFUFILES_ReadImageFromDFUFile(HANDLE hFile, int Rank, PHANDLE pImage);

extern "C" DWORD PASCAL EXPORT STDFUFILES_ImageFromFile(PSTR pPathFile, PHANDLE pImage, BYTE nAlternate);
extern "C" DWORD PASCAL EXPORT STDFUFILES_ImageToFile(PSTR pPathFile, HANDLE Image);

extern "C" DWORD PASCAL EXPORT STDFUFILES_CreateImage(PHANDLE pHandle, BYTE nAlternate);
extern "C" DWORD PASCAL EXPORT STDFUFILES_CreateImageFromMapping(PHANDLE pHandle, PMAPPING pMapping);
extern "C" DWORD PASCAL EXPORT STDFUFILES_DuplicateImage(HANDLE hSource, PHANDLE pDest);

extern "C" DWORD PASCAL EXPORT STDFUFILES_FilterImageForOperation(HANDLE Handle, PMAPPING pMapping, DWORD Operation, BOOL bTruncateLeadFFForUpgrade);
extern "C" DWORD PASCAL EXPORT STDFUFILES_DestroyImageElement(HANDLE Handle, DWORD dwRank);
extern "C" DWORD PASCAL EXPORT STDFUFILES_DestroyImage(PHANDLE pHandle);

extern "C" DWORD PASCAL EXPORT STDFUFILES_GetImageAlternate(HANDLE Handle, PBYTE pAlternate);
extern "C" DWORD PASCAL EXPORT STDFUFILES_GetImageNbElement(HANDLE Handle, PDWORD pNbElements);
extern "C" DWORD PASCAL EXPORT STDFUFILES_GetImageName(HANDLE Handle, PSTR Name);
extern "C" DWORD PASCAL EXPORT STDFUFILES_SetImageName(HANDLE Handle, PSTR Name);

extern "C" DWORD PASCAL EXPORT STDFUFILES_SetImageElement(HANDLE Handle, DWORD dwRank, BOOL bInsert, DFUIMAGEELEMENT Element);
extern "C" DWORD PASCAL EXPORT STDFUFILES_GetImageElement(HANDLE Handle, DWORD dwRank, PDFUIMAGEELEMENT pElement);

extern "C" DWORD PASCAL EXPORT STDFUFILES_GetImageSize(HANDLE Image);