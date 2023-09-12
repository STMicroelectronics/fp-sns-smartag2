@echo off
set CUBE_PROG="C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
set NAME_FW=FirmwareUpdate\Exe\FirmwareUpdate
set BOOTLOADER="..\..\..\Examples\SimpleBootLoader\Binary\SimpleBootLoader.bin"
color 0F
echo                /********************************************/
echo                            Clean FP-SNS-SMARTAG2
echo                /********************************************/
echo                              Full Chip Erase
echo                /********************************************/
%CUBE_PROG% -c port=swd mode=UR reset=HWrst -e all
echo                /********************************************/
echo                              Install BootLoader
echo                /********************************************/
%CUBE_PROG% -c port=swd mode=UR reset=HWrst -d %BOOTLOADER% 0x08000000 -v "after_programming"
echo                /********************************************/
echo                 Install FP-SNS-SMARTAG2 - (FirmwareUpdate)
echo                /********************************************/
%CUBE_PROG% -c port=swd mode=UR reset=HWrst -d %NAME_FW%.bin 0x08002000 -v "after_programming"
echo                /********************************************/
echo                      Dump FirmwareUpdate + BootLoader
echo                /********************************************/
set offset_size=0x2000
for %%I in (%NAME_FW%.bin) do set application_size=%%~zI
echo %NAME_FW%.bin size is %application_size% bytes
set /a size=%offset_size%+%application_size%
echo Dumping %offset_size% + %application_size% = %size% bytes ...
echo ..........................
%CUBE_PROG% -c port=swd mode=UR reset=HWrst -u 0x08000000 %size% %NAME_FW%_BL.bin
echo                /********************************************/
echo                                 Reset STM32
echo                /********************************************/
%CUBE_PROG% -c port=swd mode=UR reset=HWrst -rst
if NOT "%1" == "SILENT" pause