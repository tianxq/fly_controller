/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : readme.txt
* Author             : MCD Application Team
* Version            : V3.0.0
* Date               : 07/03/2009
* Description        : read me file for DfuSe Demonstrator
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

Last version 
***************

        - V3.0.0 - 07/03/2009

Package content
***************
       + Binaries :([INSTALLATION PATH]\BIN\)

         - DfuSeDemo.exe              : DfuSe Demo application
         - STDFUTester.exe            : DfuSe Tester application   
         - DfuFileMgr.exe             : DFU File Manager aplication
  	 - STDFU.dll		     : Dll that issues basic DFU requests
	 - STDFUPRT.dll	             : Dll that implements Protocol for upload and download. 
         - STDFUFiles.dll	     : Dll that implements .dfu files.

       + Sources :([INSTALLATION PATH]\Sources\)

	 - STDFUPRT    
         - STDFUFiles
	 - STDFU.dll  (Lib file only)
	 - DfuSeDemo   
         - DfuFileMgr  

       + Driver([INSTALLATION PATH]\Driver)

         - STTubeDevice30.dll	Dll layer for eadier driver access
         - STTub30.sys		Driver to be loaded for demoboard
	 - STDFU.inf		Configuration file for driver (W98SE/2000/XP/Vista/Seven (x86 & x64))

       + Documents:([INSTALLATION PATH]\Sources\Doc\)
          
         - UM0384 : DfuSe Application Programming Interface  
         - UM0391 : DfuSe File Format Specification          
         - UM0392 : DfuSe Application Programming Guide
         - UM0412 : DfuSe getting started

       + Additional tools([INSTALLATION PATH]\Tools)
    
         - GUID Generator application

Supported OS
***************

       + Windows 98SE, 2000, XP, Vista, Seven (x86 & x64 Windows platforms)

How to use 
***************

       1- Uninstall previous versions (Start-> Settings-> Control Panel-> Add or remove programs)

       2- run DfuSe setup.

       3- Install your device with the driver and the inf file (it should be better if you copy
          the STTub30.sys file manually in the windows driver directory C:\Windows\System32\drivers).
          
       4- Use it !


******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE******

