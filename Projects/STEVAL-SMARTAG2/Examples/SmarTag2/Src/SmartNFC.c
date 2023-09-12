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
  * @param  hrtc                RTC handle
  * @retval None
  */
void STNFC_ComputeMaxMinCompareTHsUi8t(SNFC_VirtualSensor_t *VirtualSensor,SNFC_LogDefinition_t *LogDefinition,RTC_HandleTypeDef *hrtc)
{
  /* Compare with Ths */
  switch(VirtualSensor->ThsUsageType) {
    case TH_EXT:
      /* External Range: -->TH1 TH2<-- */
      if((VirtualSensor->Sample.Ui8Value<=VirtualSensor->Th1.Ui8Value) |
         (VirtualSensor->Sample.Ui8Value>=VirtualSensor->Th2.Ui8Value)) {
        VirtualSensor->SampleDeltaDateTime= STNFC_GetDeltaDateTime(hrtc,LogDefinition);
      }
    break;
    case TH_INT:
      /* Internal Range: TH1<---->TH2 */
      if((VirtualSensor->Sample.Ui8Value>=VirtualSensor->Th1.Ui8Value) &
          (VirtualSensor->Sample.Ui8Value<=VirtualSensor->Th2.Ui8Value)) {
        VirtualSensor->SampleDeltaDateTime= STNFC_GetDeltaDateTime(hrtc,LogDefinition);
      }
    break;
    case TH_LESS:
      /* Less than     : -->TH1 */
      if(VirtualSensor->Sample.Ui8Value<=VirtualSensor->Th1.Ui8Value) {
        VirtualSensor->SampleDeltaDateTime= STNFC_GetDeltaDateTime(hrtc,LogDefinition);
      }
    break;
   case TH_BIGGER:
      /* Bigger than   : TH1<-- */
      if(VirtualSensor->Sample.Ui8Value>=VirtualSensor->Th1.Ui8Value) {
        VirtualSensor->SampleDeltaDateTime= STNFC_GetDeltaDateTime(hrtc,LogDefinition);
      }
  }

  /* Compare with Max Min */
  if(VirtualSensor->Sample.Ui8Value>VirtualSensor->MaxValue.Ui8Value) {
    VirtualSensor->MaxValue.Ui8Value = VirtualSensor->Sample.Ui8Value;
    VirtualSensor->MaxDeltaDateTime = STNFC_GetDeltaDateTime(hrtc,LogDefinition);
  }

  if(VirtualSensor->Sample.Ui8Value<VirtualSensor->MinValue.Ui8Value) {
    VirtualSensor->MinValue.Ui8Value = VirtualSensor->Sample.Ui8Value;
    VirtualSensor->MinDeltaDateTime = STNFC_GetDeltaDateTime(hrtc,LogDefinition);
  }
}

/**
  * @brief  Function for making the comparison with the thresholds with 16 bit data
  * @param  VirtualSensor       Virtual sensor  used
  * @param  LogDefinition       Pointer to Log definition
  * @param  hrtc                RTC handle
  * @retval None
  */
void STNFC_ComputeMaxMinCompareTHsUi16t(SNFC_VirtualSensor_t *VirtualSensor,SNFC_LogDefinition_t *LogDefinition,RTC_HandleTypeDef *hrtc)
{
  /* Compare with Ths */
  switch(VirtualSensor->ThsUsageType) {
    case TH_EXT:
      /* External Range: -->TH1 TH2<-- */
      if((VirtualSensor->Sample.Ui16Value<=VirtualSensor->Th1.Ui16Value) |
         (VirtualSensor->Sample.Ui16Value>=VirtualSensor->Th2.Ui16Value)) {
        VirtualSensor->SampleDeltaDateTime= STNFC_GetDeltaDateTime(hrtc,LogDefinition);
      }
    break;
    case TH_INT:
      /* Internal Range: TH1<---->TH2 */
      if((VirtualSensor->Sample.Ui16Value>=VirtualSensor->Th1.Ui16Value) &
          (VirtualSensor->Sample.Ui16Value<=VirtualSensor->Th2.Ui16Value)) {
        VirtualSensor->SampleDeltaDateTime= STNFC_GetDeltaDateTime(hrtc,LogDefinition);
      }
    break;
    case TH_LESS:
      /* Less than     : -->TH1 */
      if(VirtualSensor->Sample.Ui16Value<=VirtualSensor->Th1.Ui16Value) {
        VirtualSensor->SampleDeltaDateTime= STNFC_GetDeltaDateTime(hrtc,LogDefinition);
      }
    break;
   case TH_BIGGER:
      /* Bigger than   : TH1<-- */
      if(VirtualSensor->Sample.Ui16Value>=VirtualSensor->Th1.Ui16Value) {
        VirtualSensor->SampleDeltaDateTime= STNFC_GetDeltaDateTime(hrtc,LogDefinition);
      }
  }

  /* Compare with Max Min */
  if(VirtualSensor->Sample.Ui16Value>VirtualSensor->MaxValue.Ui16Value) {
    VirtualSensor->MaxValue.Ui16Value = VirtualSensor->Sample.Ui16Value;
    VirtualSensor->MaxDeltaDateTime = STNFC_GetDeltaDateTime(hrtc,LogDefinition);
  }

  if(VirtualSensor->Sample.Ui16Value<VirtualSensor->MinValue.Ui16Value) {
    VirtualSensor->MinValue.Ui16Value = VirtualSensor->Sample.Ui16Value;
    VirtualSensor->MinDeltaDateTime = STNFC_GetDeltaDateTime(hrtc,LogDefinition);
  }
}

/**
  * @brief  Function for making the comparison with the thresholds with 32 bit data
  * @param  VirtualSensor       Virtual sensor  used
  * @param  LogDefinition       Pointer to Log definition
  * @param  hrtc                RTC handle
  * @retval None
  */
void STNFC_ComputeMaxMinCompareTHsUi32t(SNFC_VirtualSensor_t *VirtualSensor,SNFC_LogDefinition_t *LogDefinition,RTC_HandleTypeDef *hrtc)
{
  /* Compare with Ths */
  switch(VirtualSensor->ThsUsageType) {
    case TH_EXT:
      /* External Range: -->TH1 TH2<-- */
      if((VirtualSensor->Sample.Ui32Value<=VirtualSensor->Th1.Ui32Value) |
         (VirtualSensor->Sample.Ui32Value>=VirtualSensor->Th2.Ui32Value)) {
        VirtualSensor->SampleDeltaDateTime= STNFC_GetDeltaDateTime(hrtc,LogDefinition);
      }
    break;
    case TH_INT:
      /* Internal Range: TH1<---->TH2 */
      if((VirtualSensor->Sample.Ui32Value>=VirtualSensor->Th1.Ui32Value) &
          (VirtualSensor->Sample.Ui32Value<=VirtualSensor->Th2.Ui32Value)) {
        VirtualSensor->SampleDeltaDateTime= STNFC_GetDeltaDateTime(hrtc,LogDefinition);
      }
    break;
    case TH_LESS:
      /* Less than     : -->TH1 */
      if(VirtualSensor->Sample.Ui32Value<=VirtualSensor->Th1.Ui32Value) {
        VirtualSensor->SampleDeltaDateTime= STNFC_GetDeltaDateTime(hrtc,LogDefinition);
      }
    break;
   case TH_BIGGER:
      /* Bigger than   : TH1<-- */
      if(VirtualSensor->Sample.Ui32Value>=VirtualSensor->Th1.Ui32Value) {
        VirtualSensor->SampleDeltaDateTime= STNFC_GetDeltaDateTime(hrtc,LogDefinition);
      }
  }

  /* Compare with Max Min */
  if(VirtualSensor->Sample.Ui32Value>VirtualSensor->MaxValue.Ui32Value) {
    VirtualSensor->MaxValue.Ui32Value = VirtualSensor->Sample.Ui32Value;
    VirtualSensor->MaxDeltaDateTime = STNFC_GetDeltaDateTime(hrtc,LogDefinition);
  }

  if(VirtualSensor->Sample.Ui32Value<VirtualSensor->MinValue.Ui32Value) {
    VirtualSensor->MinValue.Ui32Value = VirtualSensor->Sample.Ui32Value;
    VirtualSensor->MinDeltaDateTime = STNFC_GetDeltaDateTime(hrtc,LogDefinition);
  }
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
  * @brief  Set the data and time value.
  * @param  uint32_t DateTime Compressed Date&Time
  * @param  RTC_HandleTypeDef *hrtc RTC handler pointer
  * @param SNFC_LogDefinition_t *LogDef Pointer to Log definition
  * @retval Returns 1 in case of success, otherwise 0 on error..
  */
int32_t STNFC_SetDateTime(uint32_t DateTime,RTC_HandleTypeDef *hrtc,SNFC_LogDefinition_t *LogDef)
{
  int32_t RetValue =0;
  RTC_TimeTypeDef StartTime;  /* Starting Time */
  RTC_DateTypeDef StartDate;  /* Starting Date */

  struct tm *currTime;

#ifdef  SMARTAG2_VERBOSE_PRINTF
  SMARTAG2_PRINTF("Read TimeStamp %u\r\n",DateTime);
#endif /* SMARTAG2_VERBOSE_PRINTF */
  LogDef->StartTimeStamp = DateTime;

  currTime = localtime(&LogDef->StartTimeStamp);

  StartDate.Year    = currTime->tm_year - 100;
  StartDate.Date    = currTime->tm_mday;
  StartDate.Month   = currTime->tm_mon + 1;
  StartDate.WeekDay = currTime->tm_wday;
  StartTime.Hours   = currTime->tm_hour;
  StartTime.Minutes = currTime->tm_min;
  StartTime.Seconds = currTime->tm_sec;

#ifdef  SMARTAG2_VERBOSE_PRINTF
  SMARTAG2_PRINTF("\tYear    =%d\r\n",StartDate.Year);
  SMARTAG2_PRINTF("\tDate    =%d\r\n",StartDate.Date);
  SMARTAG2_PRINTF("\tMonth   =%d\r\n",StartDate.Month);
  SMARTAG2_PRINTF("\tWDay    =%d\r\n",StartDate.WeekDay);
  SMARTAG2_PRINTF("\tHours   =%d\r\n",StartTime.Hours);
  SMARTAG2_PRINTF("\tMinutes =%d\r\n",StartTime.Minutes);
  SMARTAG2_PRINTF("\tSeconds =%d\r\n",StartTime.Seconds);
#endif /* SMARTAG2_VERBOSE_PRINTF */

  StartTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  StartTime.StoreOperation = RTC_STOREOPERATION_RESET;

  if(HAL_RTC_SetTime(hrtc, &(StartTime), RTC_FORMAT_BIN) == HAL_OK) {
    if(HAL_RTC_SetDate(hrtc, &(StartDate), RTC_FORMAT_BIN) == HAL_OK) {
      RetValue = 1;
    }
  }
  return RetValue;
}

/**
  * @brief  Get the Delta epoch time.
  * @param  RTC_HandleTypeDef *hrtc RTC handler pointer
  * @param SNFC_LogDefinition_t *LogDef Pointer to Log definition
  * @retval Returns Delta epoch time in case of success, otherwise 0 on error.
  */
time_t STNFC_GetDeltaDateTime(RTC_HandleTypeDef *hrtc,SNFC_LogDefinition_t *LogDef)
{
  RTC_TimeTypeDef CurTime;
  RTC_DateTypeDef CurDate;
  time_t DeltaDateTimeStamp;
  struct tm currTime;

  if(HAL_RTC_GetTime(hrtc, &CurTime, RTC_FORMAT_BIN) != HAL_OK) {
    return 0;
  }

  if (HAL_RTC_GetDate(hrtc, &CurDate, RTC_FORMAT_BIN) != HAL_OK) {
    return 0;
  }

  currTime.tm_year  = CurDate.Year + 100;
  currTime.tm_mday  = CurDate.Date;
  currTime.tm_mon   = CurDate.Month - 1;
  /* mktime doesn't look tm_wday */
  //currTime.tm_wday = CurDate.WeekDay;
  currTime.tm_hour  = CurTime.Hours;
  currTime.tm_min   = CurTime.Minutes;
  currTime.tm_sec   = CurTime.Seconds;
  currTime.tm_isdst = 0;

  /* Find the Epoch Time */
  DeltaDateTimeStamp = mktime(&currTime);

  /* Compute the Delta Epoch Time */
  DeltaDateTimeStamp -= LogDef->StartTimeStamp;
#ifdef SMARTAG2_VERBOSE_PRINTF
  SMARTAG2_PRINTF("Delta TS=%u\r\n",(uint32_t) DeltaDateTimeStamp);
#endif /* SMARTAG2_VERBOSE_PRINTF */
  return DeltaDateTimeStamp;
}

/**
  * @brief  From Delta epoch time to Short Delta epoch time (without Seconds)
  * @param  time_t DeltaDateTimeStamp Delta epoch time stamp
  * @retval Returns Short Delta epoch time stamp
  */
uint32_t STNFC_ToShortDeltaDateTime(time_t DeltaDateTimeStamp)
{
  uint32_t ShortDeltaTimeStamp;

  DeltaDateTimeStamp/=60;
  //For using like Maximum 20bits
  ShortDeltaTimeStamp = ((uint32_t) DeltaDateTimeStamp)&0xFFFFF;

#ifdef SMARTAG2_VERBOSE_PRINTF
  SMARTAG2_PRINTF("Short Delta TS=%u\r\n",ShortDeltaTimeStamp);
#endif /* SMARTAG2_VERBOSE_PRINTF */
  return ShortDeltaTimeStamp;
}

/**
  * @brief  Get the Epoch time.
  * @param  RTC_HandleTypeDef *hrtc RTC handler pointer
  * @retval Returns Epoch time in case of success, otherwise 0 on error.
  */
time_t STNFC_GetDateTime(RTC_HandleTypeDef *hrtc)
{
  RTC_TimeTypeDef CurTime;
  RTC_DateTypeDef CurDate;
  time_t DateTimeStamp;
  struct tm currTime;

  if(HAL_RTC_GetTime(hrtc, &CurTime, RTC_FORMAT_BIN) != HAL_OK) {
    return 0;
  }

  if (HAL_RTC_GetDate(hrtc, &CurDate, RTC_FORMAT_BIN) != HAL_OK) {
    return 0;
  }

  currTime.tm_year  = CurDate.Year + 100;
  currTime.tm_mday  = CurDate.Date;
  currTime.tm_mon   = CurDate.Month - 1;
  /* mktime doesn't look tm_wday */
  //currTime.tm_wday = CurDate.WeekDay;
  currTime.tm_hour  = CurTime.Hours;
  currTime.tm_min   = CurTime.Minutes;
  currTime.tm_sec   = CurTime.Seconds;
  currTime.tm_isdst = 0;

  /* Find the Epoch Time */
  DateTimeStamp = mktime(&currTime);

  return DateTimeStamp;
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
