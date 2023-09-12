#!/bin/bash

######## Modify this Section:
# 1) Set the Installation path for STM32CubeProgrammer
# example:
#CubeProg="/c/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"
CubeProg="STM32_Programmer_CLI"

## Run section

echo "/******************************************/"
echo "           Clean FP-SNS-SMARTAG2"
echo "/******************************************/"
echo "              Full Chip Erase"
echo "/******************************************/"
"${CubeProg}" -c port=swd mode=UR reset=HWrst -e all
echo "/******************************************/"
echo "              Install BootLoader"
echo "/******************************************/"
"${CubeProg}" -c port=swd mode=UR reset=HWrst -d ../../../Examples/SimpleBootLoader/Binary/SimpleBootLoader.bin 0x08000000 -v
echo "/******************************************/"
echo " Install FP-SNS-SMARTAG2 - (FirmwareUpdate) "
echo "/******************************************/"
"${CubeProg}" -c port=swd mode=UR reset=HWrst -d ./Debug/FirmwareUpdate.bin 0x08002000 -v
echo "/******************************************/"
echo "      Dump FirmwareUpdate + BootLoader"
echo "/******************************************/"
SizeBinBL=`ls -l ./Debug/FirmwareUpdate.bin | awk '{print $5+0x4000};'`
"${CubeProg}" -c port=swd mode=UR reset=HWrst -u 0x08000000 ${SizeBinBL} ./Debug/FirmwareUpdate_BL.bin
echo "/******************************************/"
echo "                Reset STM32"
echo "/******************************************/"
"${CubeProg}" -c port=swd mode=UR reset=HWrst -rst