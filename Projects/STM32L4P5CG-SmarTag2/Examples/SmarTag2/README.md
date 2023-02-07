## <b>Smartag2 Application Description</b>

The SmarTag2 Application reads the ambient light, motion and environmental sensor data on your IoT node by the means of an NFC enabled reader, such as a mobile phone or tablet, throught a suitable AndroidST or iOST application such as the ST Asset Tracking.

The package supports battery operated use cases.

This firmware package includes Components Device Drivers, Board Support Package
and example application for the following STMicroelectronics elements:

  - STEVAL-SMARTAG2 (SmarTag2) evaluation board that contains the following components:
     - MEMS sensor devices (LPS22DF, STTS22H, LIS2DUX12, H3LIS331DL, LSM6DSO32X)
     - ambient light sensors(VD6283TX)
     - dynamic NFC tag (ST25DV64KC)
	
The example application allows the user to control the initialization phase via UART.

Launch a terminal application and set the UART port to 9600 bps, 8 bit, No Parity, 1 stop bit.
For having the same UART functionality on STEVAL-SMARTAG1 board, is necessary to recompile the code uncomment the line

	//#define SMARTAG_ENABLE_PRINTF
	
on file:

	Projects\STM32L4P5CE-SmarTag2\Examples\SmarTag2\Inc\SMARTAG2_config.h
	
### <b>Keywords</b>

NFC, SPI, I2C, UART

### <b>Hardware and Software environment</b>

 - This example runs on STEVAL-SMARTAG2 (SmarTag2) evaluation board
   can be easily tailored to any other supported device and development board.
    
 - This example must be used with the related ST Asset Tracking Android (Version 3.1.0 or higher) /iOS (Version 3.1.0 or higher) application available on Play/itune store,
   to read the sent information by NFC/RFID tag IC protocol

### <b>Dependencies</b>

STM32Cube packages:

  - STM32L4xx drivers from STM32CubeL4 v1.17.1

STEVAL-SMARTAG1:

  - STEVAL-SMARTAG2 v1.0.0


   
### <b>How to use it?</b>

This package contains projects for 3 IDEs viz. IAR, Keil µVision 5 and Integrated Development Environment for STM32. 
In order to make the  program work, you must do the following:

 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.

For IAR:

 - Open IAR toolchain (this firmware has been successfully tested with Embedded Workbench V9.20.1).
 - Open the IAR project file EWARM\SmarTag2.eww
 - Rebuild all files and load your image into target memory.
 - Run the example.

For Keil µVision 5:

 - Open Keil µVision 5 toolchain (this firmware has been successfully tested with MDK-ARM Professional Version: 5.37.0)
 - Open the Keil µVision 5 project file MDK-ARM\Project.uvprojx 
 - Rebuild all files and load your image into target memory.
 - Run the example.
 
For Integrated Development Environment for STM32:

 - Open STM32CubeIDE (this firmware has been successfully tested with Version 1.11.0).
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