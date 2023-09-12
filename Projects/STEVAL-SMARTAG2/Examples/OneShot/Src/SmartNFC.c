/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    SmartNFC.c
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.2.0
  * @date    30-May-2023
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

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "SmartNFC.h"
#include "TagType5.h"

/* Exported Variables -------------------------------------------------------- */
char *ThresholdsUsageName[4] = {
  "Ext",
  "Int",
  "Less",
  "Bigger"
};

/**
  * @brief  Function for making the comparison with the thresholds with 8 bit data
  * @param  VirtualSensor       Virtual sensor  used
  * @param  LogDefinition       Pointer to Log definition
  * @retval None
  */
void STNFC_ComputeMaxMinCompareTHsUi8t(SNFC_VirtualSensor_t *VirtualSensor,SNFC_LogDefinition_t *LogDefinition)
{
  /* Compare with Ths */
  switch(VirtualSensor->ThsUsageType) {
    case TH_EXT:
      /* External Range: -->TH1 TH2<-- */
      if((VirtualSensor->Sample.Ui8Value<=VirtualSensor->Th1.Ui8Value) |
         (VirtualSensor->Sample.Ui8Value>=VirtualSensor->Th2.Ui8Value)) {
        VirtualSensor->SampleDeltaDateTime= 1; /* Fake Value */
      }
    break;
    case TH_INT:
      /* Internal Range: TH1<---->TH2 */
      if((VirtualSensor->Sample.Ui8Value>=VirtualSensor->Th1.Ui8Value) &
          (VirtualSensor->Sample.Ui8Value<=VirtualSensor->Th2.Ui8Value)) {
        VirtualSensor->SampleDeltaDateTime= 1; /* Fake Value */
      }
    break;
    case TH_LESS:
      /* Less than     : -->TH1 */
      if(VirtualSensor->Sample.Ui8Value<=VirtualSensor->Th1.Ui8Value) {
        VirtualSensor->SampleDeltaDateTime= 1; /* Fake Value */
      }
    break;
   case TH_BIGGER:
      /* Bigger than   : TH1<-- */
      if(VirtualSensor->Sample.Ui8Value>=VirtualSensor->Th1.Ui8Value) {
        VirtualSensor->SampleDeltaDateTime= 1; /* Fake Value */
      }
  }

  VirtualSensor->MaxValue.Ui8Value = VirtualSensor->Sample.Ui8Value;
  VirtualSensor->MaxDeltaDateTime = 1;

  VirtualSensor->MinValue.Ui8Value = VirtualSensor->Sample.Ui8Value;
  VirtualSensor->MinDeltaDateTime = 1;
}

/**
  * @brief  Function for making the comparison with the thresholds with 16 bit data
  * @param  VirtualSensor       Virtual sensor  used
  * @param  LogDefinition       Pointer to Log definition
  * @retval None
  */
void STNFC_ComputeMaxMinCompareTHsUi16t(SNFC_VirtualSensor_t *VirtualSensor,SNFC_LogDefinition_t *LogDefinition)
{
  /* Compare with Ths */
  switch(VirtualSensor->ThsUsageType) {
    case TH_EXT:
      /* External Range: -->TH1 TH2<-- */
      if((VirtualSensor->Sample.Ui16Value<=VirtualSensor->Th1.Ui16Value) |
         (VirtualSensor->Sample.Ui16Value>=VirtualSensor->Th2.Ui16Value)) {
        VirtualSensor->SampleDeltaDateTime= 1; /* Fake Value */
      }
    break;
    case TH_INT:
      /* Internal Range: TH1<---->TH2 */
      if((VirtualSensor->Sample.Ui16Value>=VirtualSensor->Th1.Ui16Value) &
          (VirtualSensor->Sample.Ui16Value<=VirtualSensor->Th2.Ui16Value)) {
        VirtualSensor->SampleDeltaDateTime= 1; /* Fake Value */
      }
    break;
    case TH_LESS:
      /* Less than     : -->TH1 */
      if(VirtualSensor->Sample.Ui16Value<=VirtualSensor->Th1.Ui16Value) {
        VirtualSensor->SampleDeltaDateTime= 1; /* Fake Value */
      }
    break;
   case TH_BIGGER:
      /* Bigger than   : TH1<-- */
      if(VirtualSensor->Sample.Ui16Value>=VirtualSensor->Th1.Ui16Value) {
        VirtualSensor->SampleDeltaDateTime= 1; /* Fake Value */
      }
  }

  VirtualSensor->MaxValue.Ui16Value = VirtualSensor->Sample.Ui16Value;
  VirtualSensor->MaxDeltaDateTime = 1;

  VirtualSensor->MinValue.Ui16Value = VirtualSensor->Sample.Ui16Value;
  VirtualSensor->MinDeltaDateTime = 1;
}

/**
  * @brief  Function for making the comparison with the thresholds with 32 bit data
  * @param  VirtualSensor       Virtual sensor  used
  * @param  LogDefinition       Pointer to Log definition
  * @retval None
  */
void STNFC_ComputeMaxMinCompareTHsUi32t(SNFC_VirtualSensor_t *VirtualSensor,SNFC_LogDefinition_t *LogDefinition)
{
  /* Compare with Ths */
  switch(VirtualSensor->ThsUsageType) {
    case TH_EXT:
      /* External Range: -->TH1 TH2<-- */
      if((VirtualSensor->Sample.Ui32Value<=VirtualSensor->Th1.Ui32Value) |
         (VirtualSensor->Sample.Ui32Value>=VirtualSensor->Th2.Ui32Value)) {
        VirtualSensor->SampleDeltaDateTime= 1; /* Fake Value */
      }
    break;
    case TH_INT:
      /* Internal Range: TH1<---->TH2 */
      if((VirtualSensor->Sample.Ui32Value>=VirtualSensor->Th1.Ui32Value) &
          (VirtualSensor->Sample.Ui32Value<=VirtualSensor->Th2.Ui32Value)) {
        VirtualSensor->SampleDeltaDateTime= 1; /* Fake Value */
      }
    break;
    case TH_LESS:
      /* Less than     : -->TH1 */
      if(VirtualSensor->Sample.Ui32Value<=VirtualSensor->Th1.Ui32Value) {
        VirtualSensor->SampleDeltaDateTime= 1; /* Fake Value */
      }
    break;
   case TH_BIGGER:
      /* Bigger than   : TH1<-- */
      if(VirtualSensor->Sample.Ui32Value>=VirtualSensor->Th1.Ui32Value) {
        VirtualSensor->SampleDeltaDateTime= 1; /* Fake Value */
      }
  }

  VirtualSensor->MaxValue.Ui32Value = VirtualSensor->Sample.Ui32Value;
  VirtualSensor->MaxDeltaDateTime = 1;

  VirtualSensor->MinValue.Ui32Value = VirtualSensor->Sample.Ui32Value;
  VirtualSensor->MinDeltaDateTime = 1;
}

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
  SMARTAG2_PRINTF("%s@%ld:",file,line);
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
