## <b>FirmwareUpdate Application Description</b>

The FirmwareUpdate application saves the new firwmare received via NFC using the ST FTM protocol and saves it on the right flash region and works with the SimpleBootLoader example for enabling the Firmware Update capability.

The package supports battery operated use cases.

This firmware package includes Components Device Drivers, Board Support Package
and example application for the following STMicroelectronics elements:

  - STEVAL-SMARTAG2 (SmarTag2) evaluation board that contains the following components:
     - MEMS sensor devices (LPS22DF, STTS22H, LIS2DUXS12, H3LIS331DL, LSM6DSO32X)
     - ambient light sensors(VD6283TX)
     - dynamic NFC tag (ST25DV64KC)
	
The example application allows the user to control the initialization phase via UART.

Launch a terminal application and set the UART port to 115200 bps, 8 bit, No Parity, 1 stop bit.
For having the same UART functionality on STEVAL-SMARTAG2 board, is necessary to recompile the code uncomment the line

	//#define SMARTAG_ENABLE_PRINTF
	
on file:

	Projects\STM32L4P5CE-SmarTag2\Applications\FirmwareUpdate\Inc\Firmware_conf.h

### <b>Very Important</b>
			  
 1) The firmware updated is done using the ST25 Android/iOS application (Version 3.7.0 and above)
 
 2) This example must run starting at address 0x08002000 in memory and works ONLY if the SimpleBootLoader 
 is saved at the beginning of the FLASH (address 0x08000000)
 
 3) For each IDE (IAR/µVision/STM32CubeIDE) there are some scripts *.bat/*.sh that makes the following operations:
    - Full Flash Erase
    - Load the SimpleBootLoader on the rigth flash region
    - Load the Program (after the compilation) on the rigth flash region
    - Dump back one single binary that contain SimpleBootLoader+Program that could be 
      flashed at the flash beginning (address 0x08000000)
    - Reset the board
	
 4) After code generation from STM32CubeMX software before building:
    - Replaced in the folder Src the file system_stm32f4xx.c
    - for IAR and STM32CubeIDE replace the linker scripts with the scripts in the folder LinkerScript
    - for Keil, open the "Options for Target" and in the tab "Target" set start= 0x8002000 and sixe= 0x7E000. 
	
### <b>Keywords</b>

NFC, SPI, I2C, UART

### <b>Hardware and Software environment</b>

 - This example runs on STEVAL-SMARTAG2 (SmarTag2) evaluation board
   can be easily tailored to any other supported device and development board.
    
 - This example must be used with the ST25 Android (Version 3.7.0 or higher) /iOS (Version 3.7.0 or higher) application available on Play/itune store.

ADDITIONAL_COMP : LPS22DF https://www.st.com/en/mems-and-sensors/lps22df.html
ADDITIONAL_COMP : STTS22H https://www.st.com/en/mems-and-sensors/stts22h.html
ADDITIONAL_COMP : LIS2DUXS12 https://www.st.com/en/mems-and-sensors/lis2duxs12.html
ADDITIONAL_COMP : H3LIS331DL https://www.st.com/en/mems-and-sensors/h3lis331dl.html
ADDITIONAL_COMP : LSM6DSO32X https://www.st.com/en/mems-and-sensors/lsm6dso32x.html
ADDITIONAL_COMP : VD6283TX https://www.st.com/en/imaging-and-photonics-solutions/vd6283tx.html
ADDITIONAL_COMP : ST25DV64KC https://www.st.com/en/nfc/st25dv64kc.html

### <b>Dependencies</b>

STM32Cube packages:

  - STM32L4xx drivers from STM32CubeL4 v1.17.2

STEVAL-SMARTAG1:

  - STEVAL-SMARTAG2 v1.2.0
  
### <b>How to use it?</b>

This package contains projects for 3 IDEs viz. IAR, Keil µVision 5 and Integrated Development Environment for STM32. 
In order to make the  program work, you must do the following:

 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.

For IAR:

 - Open IAR toolchain (this firmware has been successfully tested with Embedded Workbench V9.20.1).
 - Open the IAR project file EWARM\FirmwareUpdate.eww
 - Rebuild all files and load your image into target memory.
 - Run the example.

For Keil µVision 5:

 - Open Keil µVision 5 toolchain (this firmware has been successfully tested with MDK-ARM Professional Version: 5.37.0)
 - Open the Keil µVision 5 project file MDK-ARM\Project.uvprojx 
 - Rebuild all files and load your image into target memory.
 - Run the example.
 
For Integrated Development Environment for STM32:

 - Open STM32CubeIDE (this firmware has been successfully tested with Version 1.12'.0).
 - Set the default workspace proposed by the IDE (please be sure that there are not spaces in the workspace path).
 - Press "File" -> "Import" -> "Existing Projects into Workspace"; press "Browse" in the "Select root directory" and choose the path where the CubeIDE
   project is located (it should be STM32CubeIDE). 
 - Rebuild all files
 
### <b>Author</b>

SRA Application Team

### <b>License</b>

Copyright (c) 2023 STMicroelectronics.
All rights reserved.

This software is licensed under terms that can be found in the LICENSE file
in the root directory of this software component.
If no LICENSE file comes with this software, it is provided AS-IS.