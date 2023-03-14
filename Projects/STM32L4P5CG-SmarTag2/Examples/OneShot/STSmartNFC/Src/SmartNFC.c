/**
  ******************************************************************************
  * @file    SmartNFC.c
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.1.0
  * @date    22-February-2023
  * @brief   Minimal NDEF header APIs implementation
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "SmartNFC.h"
#include "TagType5.h"

/* Define  -------------------------------------------------------------------*/

/* Macros  -------------------------------------------------------------------*/
//update the Max/Min for any valid value
#define MCR_STNFC_ComputeMaxMinCompareTHs(Type)\
void STNFC_ComputeMaxMinCompareTHs##Type##t(SNFC_VirtualSensor_t *VirtualSensor,SNFC_LogDefinition_t *LogDefinition)\
{\
  /* Compare with Ths */\
  switch(VirtualSensor->ThsUsageType) {\
    case TH_EXT:\
      /* External Range: -->TH1 TH2<-- */\
      if((VirtualSensor->Sample.Type##Value<=VirtualSensor->Th1.Type##Value) | \
         (VirtualSensor->Sample.Type##Value>=VirtualSensor->Th2.Type##Value)) {\
        VirtualSensor->SampleDeltaDateTime= 1 /*Fake */;\
      }\
    break;\
    case TH_INT:\
      /* Internal Range: TH1<---->TH2 */\
      if((VirtualSensor->Sample.Type##Value>=VirtualSensor->Th1.Type##Value) & \
          (VirtualSensor->Sample.Type##Value<=VirtualSensor->Th2.Type##Value)) {\
        VirtualSensor->SampleDeltaDateTime= 1 /*Fake */;\
      }\
    break;\
    case TH_LESS:\
      /* Less than     : -->TH1 */\
      if(VirtualSensor->Sample.Type##Value<=VirtualSensor->Th1.Type##Value) {\
        VirtualSensor->SampleDeltaDateTime=1 /*Fake */;\
      }\
    break;\
   case TH_BIGGER:\
      /* Bigger than   : TH1<-- */\
      if(VirtualSensor->Sample.Type##Value>=VirtualSensor->Th1.Type##Value) {\
        VirtualSensor->SampleDeltaDateTime= 1 /*Fake */;\
      }\
  }\
  \
  /* Compare with Max Min */\
    VirtualSensor->MaxValue.Type##Value = VirtualSensor->Sample.Type##Value;\
    VirtualSensor->MaxDeltaDateTime = 1 /*Fake */;\
  \
    VirtualSensor->MinValue.Type##Value = VirtualSensor->Sample.Type##Value;\
    VirtualSensor->MinDeltaDateTime = 1 /*Fake */;\
}
          
/* Exported Variables -------------------------------------------------------- */
char *ThresholdsUsageName[4] = {
  "Ext",
  "Int",
  "Less",
  "Bigger"
};

//Functions For making the Comparison with the Thresholds
MCR_STNFC_ComputeMaxMinCompareTHs(Ui8)
MCR_STNFC_ComputeMaxMinCompareTHs(Ui16)
MCR_STNFC_ComputeMaxMinCompareTHs(Ui32)

/**
  * @brief  Initialize the ST-Smart NFC
  * @param  None
  * @retval None
  */
void InitSTSmartNFC(void)
{
  NfcType5_NDEFInitHeader();
}

/**
  * @brief  Error Handler for Reading/Writing function for NFC.
  *         User may add here some code to deal with this error.
  * @param  SNFC_ErrorCode_t ErroCode Error Code Flag
  * @retval None
  */
__weak void STNFC_Error(SNFC_ErrorCode_t ErroCode, char *file, int32_t line)
{
  SMARTAG2_PRINTF("%s@%d:",file,line);
  switch(ErroCode) {
    case STNFC_CONFIG_ERROR:
      SMARTAG2_PRINTF("STNFC_CONFIG_ERROR\r\n");
      break;
    case STNFC_WRITING_ERROR:
      SMARTAG2_PRINTF("NSTFC_WRITING_ERROR\r\n");
      break;
    case STNFC_READING_ERROR:
      SMARTAG2_PRINTF("NFC_READING_ERROR\r\n");
      break;
     case STNFC_INIT_ERROR:
      SMARTAG2_PRINTF("STNFC_INIT_ERROR\r\n");
      break;
     case STNFC_RUNTIME_ERROR:
      SMARTAG2_PRINTF("STNFC_RUNTIME_ERROR\r\n");
      break;
  }
}

