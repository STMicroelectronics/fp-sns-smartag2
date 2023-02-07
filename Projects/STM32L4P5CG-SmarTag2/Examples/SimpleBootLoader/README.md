## <b>SimpleBootLoader Application Description</b>

The SimpleBootLoader apply the firmware updated replacing the current running firmware with the new version received via NFC or runs the current firmware if the new version is not available.

The package supports battery operated use cases.

This firmware package includes Components Device Drivers, Board Support Package
and example application for the following STMicroelectronics elements:

  - STEVAL-SMARTAG2 (SmarTag2) evaluation board that contains the following components:
     - MEMS sensor devices (LPS22DF, STTS22H, LIS2DUX12, H3LIS331DL, LSM6DSO32X)
     - ambient light sensors(VD6283TX)
     - dynamic NFC tag (ST25DV64KC)

### <b>Keywords</b>

NFC, SPI, I2C

### <b>Hardware and Software environment</b>

 - This example runs on STEVAL-SMARTAG2 (SmarTag2) evaluation board
   can be easily tailored to any other supported device and development board.

ADDITIONAL_COMP : [ST25DV64KC](https://www.st.com/en/nfc/st25dv64kc.html)
	
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
 - Open the Keil µVision 5 project file MDK-ARM\SimpleBootLoader.uvprojx 
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