/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    AppSmarTag.c
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.2.0
  * @date    30-May-2023
  * @brief   Application Process
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

#include "AppSmarTag.h"
#include "SMARTAG2_config.h"
#include "app_smartag2.h"
#include "main.h"

#include <string.h>
#include <math.h>

#include "SmarTag2_env_sensors.h"
#include "SmarTag2_env_sensors_ex.h"
#include "SmarTag2_motion_sensors_ex.h"
#include "SmarTag2_nfctag.h"
#include "SmarTag2_nfctag_ex.h"
#include "Smartag2_light_sensor.h"

#include "st25dvxxkc.h"
#include "SmartNFC.h"
#include "TagType5.h"
#include "lsm6dso32x_tilt_angle_mode0.h"
#include "lis2duxs12_asset_tracking_lp.h"

/* Private Defines -----------------------------------------------------------*/

#define STTS22H_SAMPLE_TO_CODED(Value)        ((uint16_t)(((Value)*5) - ((-10)*5)))
#define LPS22DF_SAMPLE_TO_CODED(Value)       ((uint16_t)(((Value)*2) - ((260)*2)))
#define VD6283_LUX_SAMPLE_TO_CODED(Value)    ((uint32_t)((Value)*1000))

#define STTS22H_CODED_TO_SAMPLE(Value)       ((((float)(Value))/5)-10)
#define LPS22DF_CODED_TO_SAMPLE(Value)       ((((float)(Value))/2)+260)
#define VD6283_LUX_CODED_TO_SAMPLE(Value)     (((float)(Value))/1000)

/* Private typedef -----------------------------------------------------------*/
/**
* @brief CCT/Lux Resulst Structure
*/
typedef struct
{
  double_t cct; /* cct (expressed in °K) */
  double_t X;
  double_t Y; /* illuminance (expressed in lux) */
  double_t Z;
} ResultCCT_t;

/**
  * @brief  Valid Configuration Type
  */
typedef enum
{
  STNFC_VALID_CONFIG = 0,
  STNFC_NOT_VALID_CONFIG,
  STNFC_NOT_CHANGED_CONFIG,
  STNFC_ERROR_READING_CONFIG
} SNFC_ValidConfiguration_t;

/**
* @brief NFC Status Enumerative Type
*/
typedef enum
{
  NFC_STATUS_OFF = 0,
  NFC_STATUS_ON
} FNCStatusEnum_t;

/* Exported Variables --------------------------------------------------------*/
RTC_HandleTypeDef hrtc;
EXTI_HandleTypeDef hexti0 = {.Line = EXTI_LINE_0};
EXTI_HandleTypeDef hexti4 = {.Line = EXTI_LINE_4};
EXTI_HandleTypeDef hexti13 = {.Line = EXTI_LINE_13};

/* Imported variables --------------------------------------------------------*/
/* Context of LSM6DSOX32 */
extern void *MotionCompObj[];
#define LSM6DSOX32_Contex (&(((LSM6DSO32X_Object_t *)MotionCompObj[LSM6DSO32X_0])->Ctx))
/* Context of LIS2DUXS12 */
#define LIS2DUXS12_Contex (&(((LIS2DUXS12_Object_t *)MotionCompObj[LIS2DUXS12_0])->Ctx))

/* Private variables ---------------------------------------------------------*/
static volatile FNCStatusEnum_t NFCStatus = NFC_STATUS_OFF;

static volatile uint8_t FirstEventNotApplicable = 0;

/* We need to Read the Sensors and Save Log */
static int32_t ReadSensorAndLog = NO_EVENT;

static int32_t ForceStart=0;

static uint8_t LogMode = SMARTAG2_LOGMODE_INACTIVE;

/* R/F Activity from ST25DV */
static uint32_t RFActivityStatus = FIELD_UNDEF;

//ST Smart NFC Protocol
SNFC_CodeDefinition_t SmarTag2CodeHeader;
SNFC_LogDefinition_t LogDefinition;
SNFC_VirtualSensor_t AllVirtualSensorsArray[SMARTAG2_VIRTUAL_SENSORS_NUM];
SNFC_VirtualSensor_t *ConfiguratedVirtualSensorsArray[SMARTAG2_VIRTUAL_SENSORS_NUM];

/* Private function prototypes -----------------------------------------------*/
static void BSP_ACC_INT_Callback(void);

static void InitSmarTagEnvSensors(void);
static void SetNFCBehavior(void);
static int32_t InitDeInitAccEventThreshold(void);
static uint8_t Understand6DOrientation(void);
static int32_t DetectorValueForAccEvent(void);
static int32_t AccNormVectorApproxEvaluator(BSP_MOTION_SENSOR_Axes_t Value_XYZ_mg);
static void MEMS_Sensors_ReadData(void);
static void ComputeCCT(uint32_t TimeExposure, uint32_t *AlsResults,ResultCCT_t *Result);

/* Function prototypes for managing the Log configuration */
static void CheckIfNewConfiguration(void);
static void ReadConfiguration(SNFC_LogDefinition_t *LogDefinition,uint32_t CurrentStartDateTime,int32_t OnlyChecks);
static void SetAllAvailableVirtualSensors(void);
static void SaveDefaultConfiguration(void);
static void ResetMaxMinValuesAllVirtualSensors(void);
static void SaveMaxMinValuesForVirtualSensors(void);
static void SaveVirtualSensorsConfiguration(void);
static void UpdateLastSamplePointerAndSampleCounter(SNFC_LogDefinition_t *LogDefinition);
static void MX_RTC_Init(void);

/**
  * @brief RTC initialization
  * @param None
  * @retval None
  */
void Init_RTC(void)
{
  /* Initialize all configured peripherals */
  MX_RTC_Init();
}

/**
  * @brief Register event irq handler for accelerometer interrupt pin
  * @param None
  * @retval None
  */
void SetAccIntPin_exti(void)
{
  /* register event irq handler */
  HAL_EXTI_GetHandle(&hexti0, EXTI_LINE_0);
  HAL_EXTI_RegisterCallback(&hexti0, HAL_EXTI_COMMON_CB_ID, BSP_ACC_INT_Callback);
  HAL_NVIC_SetPriority(EXTI0_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /** Initialize RTC Only */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    STNFC_Error_Handler(STNFC_INIT_ERROR);
  }

  /** Initialize RTC and set the Time and Date */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    STNFC_Error_Handler(STNFC_INIT_ERROR);
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    STNFC_Error_Handler(STNFC_INIT_ERROR);
  }

  /** Enable the WakeUp */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 1, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0) != HAL_OK)
  {
    STNFC_Error_Handler(STNFC_INIT_ERROR);
  }

  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);

}

/**
* @brief  SmarTag Application Start
* @param  None
* @retval None
*/
void SmarTagAppStart(void)
{
  BSP_NFCTAG_EEP_PowerOn();
  SMARTAG2_PRINTF("Power on NFC (VDD EEP On)\r\n\r\n");
  /* Rise time required by VDD_EEPROM for NFC */
  HAL_Delay(200);

  /* Init I2C interface */
  if(BSP_NFCTAG_Init(BSP_NFCTAG_INSTANCE)!=NFCTAG_OK ) {
    SMARTAG2_PRINTF("Error NFCTAG Initialized\r\n");
    STNFC_Error_Handler(STNFC_CONFIG_ERROR);
  } else {
    SMARTAG2_PRINTF("NFCTAG Initialized\r\n");
  }

  BSP_NFCTAG_LPD_Off();
  NFCStatus = NFC_STATUS_ON;

  InitSTSmartNFC();

  /* Reset the Structures used for Controlling the Log */
  memset(&SmarTag2CodeHeader,0,sizeof(SNFC_CodeDefinition_t));
  memset(&LogDefinition,0,sizeof(SNFC_LogDefinition_t));
  memset(AllVirtualSensorsArray,0,SMARTAG2_VIRTUAL_SENSORS_NUM*sizeof(SNFC_VirtualSensor_t));
  memset(ConfiguratedVirtualSensorsArray,0,SMARTAG2_VIRTUAL_SENSORS_NUM*sizeof(SNFC_VirtualSensor_t *));

  SetAllAvailableVirtualSensors();

  /* Control if there is a valid Configuration saved on Tag */
  SMARTAG2_PRINTF("\r\nControl if there is a Valid Configuration\r\n\r\n");
  ReadConfiguration(&LogDefinition,0,0);

  /* Check if there a valid configuration saved on the TAG */
  if(LogDefinition.StartDateTime==0) {
    SMARTAG2_PRINTF("\tConfiguration NOT Present use default One\r\n\r\n");
    /* Set the Default Configuration */
    SaveDefaultConfiguration();

    /* Set the Date&Time */
    if(STNFC_SetDateTime(LogDefinition.StartDateTime,&hrtc,&LogDefinition)==0) {
      SMARTAG2_PRINTF("\r\nError: Setting RTC\r\n");
    } else {
       SMARTAG2_PRINTF("\r\nSet RTC Date&Time\r\n");
    }

  } else {

    /* There is a valid Configuration
    * Initializes the log section */
    SMARTAG2_PRINTF("\tConfiguration Present on NFC\r\n\r\n");

    /* Set the Date&Time */
    if(STNFC_SetDateTime(LogDefinition.StartDateTime,&hrtc,&LogDefinition)==0) {
      SMARTAG2_PRINTF("Error: Setting RTC\r\n");
    } else {
       SMARTAG2_PRINTF("Set RTC Date&Time\r\n");
    }

    /* Reset the Max/Min For each Sensor */
    ResetMaxMinValuesAllVirtualSensors();

    /* Save the Max/Min for each Sensor */
    SaveMaxMinValuesForVirtualSensors();

    /* Write Sample Counter and Last Sample Pointer*/
    LogDefinition.SampleCounterAddress = LogDefinition.LastSamplePointer;
    LogDefinition.SampleCounter=0;
    UpdateLastSamplePointerAndSampleCounter(&LogDefinition);
    LogDefinition.LastSamplePointer+=8;

  }

  NfcType5_SetInitialNDEFPayLoadLengthValue(LogDefinition.LastSamplePointer);

  NfcType5_ComputeNDEFPayLoadSize(LogDefinition.LastSamplePointer,LogDefinition.SampleCounter);

  /* we go in INACTIVE Mode */
  LogMode = SMARTAG2_LOGMODE_INACTIVE;

  /* Set the NFC behavior */
  SMARTAG2_PRINTF("\r\nSet NFC Behavior\r\n");
  SetNFCBehavior();
  BSP_NFCTAG_LPD_On();
  NFCStatus = NFC_STATUS_OFF;
}

/**
* @brief  Callback Function for Wake Up Timer
* @param  None
* @retval None
*/
void SmarTagAppWakeUpTimerCallBack(void)
{
  static int32_t NumWakeUps= 0;
  static int32_t FirstTime=1;

  if(RFActivityStatus != FIELD_RISING ) {
    /* if the phone is under the NFC */
    NumWakeUps++;
  }

  if((FirstTime==1) & (LogMode == SMARTAG2_LOGMODE_INACTIVE)) {

    if(NumWakeUps==SMARTAG2_AUTOSTART_SECONDS) {

      HWInitializationStep2();

      FirstTime =0;
      NumWakeUps = 0;
      SMARTAG2_PRINTF("AutoStart\r\n\r\n");

      /* we go in ACTIVE Mode */
      LogMode = SMARTAG2_LOGMODE_ACTIVE;

      /* we Force the Start even if there is not a change of the field */
      ForceStart = 1;

      /* Power On/Off the Intertial sensors */
      if((AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Enable) |
         (AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].Enable) |
         (AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].Enable) |
         (AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].Enable)) {
         BSP_MOTION_SENSOR_PowerOn();
         SMARTAG2_PRINTF("VDD ACC On\r\n");
       } else {
          BSP_MOTION_SENSOR_PowerOff();
         SMARTAG2_PRINTF("VDD ACC Off\r\n");
       }

      /* Rise time required by VDD_SENS and MEMS sensor startup time */
      HAL_Delay(400);

      /* lsm6dso32x WakeUp/6D Rec or MLC: Init/DeInit */
      InitDeInitAccEventThreshold();

      /* We said to Read and Save next sample */
      ReadSensorAndLog |= SYNC_EVENT;
    }
  } else {
    if(NumWakeUps>=LogDefinition.SampleTime) {
      NumWakeUps = 0;
      /* We said to Read and Save next sample */
      ReadSensorAndLog |=SYNC_EVENT;
      FirstTime=0;
    }
  }
}

void SmarTagAppProcess(void)
{
  /* Receive one interrupt from Timer */
  if((RFActivityStatus==FIELD_FALLING) | (ForceStart==1)) {
    if(ForceStart) {
      RFActivityStatus=FIELD_FALLING;
      ForceStart=0;
    }

    if( (ReadSensorAndLog & SYNC_EVENT ) ||
       (ReadSensorAndLog & ASYNC_EVENT) ) {
         if(NFCStatus == NFC_STATUS_OFF) {
           BSP_NFCTAG_LPD_Off();
           /* rise time required by NFC */
           HAL_Delay(10);
           NFCStatus = NFC_STATUS_ON;
           ST25_RETRY(BSP_NFCTAG_SetRFSleep_Dyn(BSP_NFCTAG_INSTANCE));
#ifdef SMARTAG2_VERBOSE_PRINTF
           SMARTAG2_PRINTF("\t-->Sleep RF\r\n");
#endif /* SMARTAG2_VERBOSE_PRINTF */
         }

         BSP_LED_On(LED2);

         if (LogMode == SMARTAG2_LOGMODE_INACTIVE) {
           /* Do Nothing */
           goto SMARTAG2_SLEEP;
         }

         if(ReadSensorAndLog & ASYNC_EVENT) {
           uint32_t DataBuf32;
           ReadSensorAndLog &= ~ASYNC_EVENT;
           SMARTAG2_PRINTF("Async Event:\r\n");

           if(AllVirtualSensorsArray[LSM6DSOX32_VS_ID].SampleDeltaDateTime!=0) {
#ifdef SMARTAG2_VERBOSE_PRINTF
             SMARTAG2_PRINTF("Save LSM6DSOX32 WakeUp=%d\r\n", AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Sample.Ui16Value);
#else /* SMARTAG2_VERBOSE_PRINTF */
             SMARTAG2_PRINTF("Save LSM6DSOX32 WakeUp\r\n");
#endif /* SMARTAG2_VERBOSE_PRINTF */

             DataBuf32 = LSM6DSOX32_VS_ID |
               (((uint32_t)AllVirtualSensorsArray[LSM6DSOX32_VS_ID].SampleDeltaDateTime)<<4);
             if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
               STNFC_Error_Handler(STNFC_WRITING_ERROR);
             }
             LogDefinition.LastSamplePointer+=4;

             DataBuf32 = AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Sample.Ui16Value;
             if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
               STNFC_Error_Handler(STNFC_WRITING_ERROR);
             }
             LogDefinition.LastSamplePointer+=4;

             /* Increment the new Sample Counter until the end of the Tag */
             NfcType5_UpdateSampleCounter(&LogDefinition,8);

             /* Update Sample Counter and Last Sample Pointer */
             UpdateLastSamplePointerAndSampleCounter(&LogDefinition);

             AllVirtualSensorsArray[LSM6DSOX32_VS_ID].SampleDeltaDateTime=0;
           }

           if(AllVirtualSensorsArray[LSM6DSOX32_VS_ID].MaxDeltaDateTime!=0)  {
#ifdef SMARTAG2_VERBOSE_PRINTF
             SMARTAG2_PRINTF("Save Max Value for LSM6DSOX32 =%d\r\n", AllVirtualSensorsArray[LSM6DSOX32_VS_ID].MaxValue.Ui16Value);
#else /* SMARTAG2_VERBOSE_PRINTF */
             SMARTAG2_PRINTF("Save Max Value for LSM6DSOX32\r\n");
#endif /* SMARTAG2_VERBOSE_PRINTF */

             DataBuf32 = STNFC_ToShortDeltaDateTime(AllVirtualSensorsArray[LSM6DSOX32_VS_ID].MaxDeltaDateTime);
             if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32,
                                     AllVirtualSensorsArray[LSM6DSOX32_VS_ID].MaxPositionPointer, 4)!=NFCTAG_OK){
                                       STNFC_Error_Handler(STNFC_WRITING_ERROR);
                                     }
             DataBuf32 = (uint32_t ) AllVirtualSensorsArray[LSM6DSOX32_VS_ID].MaxValue.Ui16Value;
             if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32,
                                     AllVirtualSensorsArray[LSM6DSOX32_VS_ID].MaxPositionPointer+4, 4)!=NFCTAG_OK){
                                       STNFC_Error_Handler(STNFC_WRITING_ERROR);
                                     }
             AllVirtualSensorsArray[LSM6DSOX32_VS_ID].MaxDeltaDateTime=0;
           }

           if(AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].SampleDeltaDateTime!=0) {
#ifdef SMARTAG2_VERBOSE_PRINTF
             SMARTAG2_PRINTF("Save LSM6DSOX32 6D=%d\r\n", AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].Sample.Ui16Value);
#else /* SMARTAG2_VERBOSE_PRINTF */
             SMARTAG2_PRINTF("Save LSM6DSOX32 6D\r\n");
#endif /* SMARTAG2_VERBOSE_PRINTF */

             DataBuf32 = LSM6DSOX32_6D_VS_ID |
               (((uint32_t)AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].SampleDeltaDateTime)<<4);
             if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
               STNFC_Error_Handler(STNFC_WRITING_ERROR);
             }
             LogDefinition.LastSamplePointer+=4;
             DataBuf32 = AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].Sample.Ui8Value;
             if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
               STNFC_Error_Handler(STNFC_WRITING_ERROR);
             }
             LogDefinition.LastSamplePointer+=4;

             /* Increment the new Sample Counter until the end of the Tag */
             NfcType5_UpdateSampleCounter(&LogDefinition,8);

             /* Update Sample Counter and Last Sample Pointer */
             UpdateLastSamplePointerAndSampleCounter(&LogDefinition);

             AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].SampleDeltaDateTime=0;
           }

           if(AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].SampleDeltaDateTime!=0) {
#ifdef SMARTAG2_VERBOSE_PRINTF
             SMARTAG2_PRINTF("Save LSM6DSOX32 MLC=%d\r\n", AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].Sample.Ui8Value);
#else /* SMARTAG2_VERBOSE_PRINTF */
             SMARTAG2_PRINTF("Save LSM6DSOX32 MLC\r\n");
#endif /* SMARTAG2_VERBOSE_PRINTF */

             DataBuf32 = LSM6DSOX32_MLC_VS_ID |
               (((uint32_t)AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].SampleDeltaDateTime)<<4);
             if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
               STNFC_Error_Handler(STNFC_WRITING_ERROR);
             }
             LogDefinition.LastSamplePointer+=4;
             DataBuf32 = AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].Sample.Ui8Value;
             if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
               STNFC_Error_Handler(STNFC_WRITING_ERROR);
             }
             LogDefinition.LastSamplePointer+=4;

             /* Increment the new Sample Counter until the end of the Tag */
             NfcType5_UpdateSampleCounter(&LogDefinition,8);

             /* Update Sample Counter and Last Sample Pointer */
             UpdateLastSamplePointerAndSampleCounter(&LogDefinition);

             AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].SampleDeltaDateTime=0;
           }

           if(AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].SampleDeltaDateTime!=0) {
#ifdef SMARTAG2_VERBOSE_PRINTF
             SMARTAG2_PRINTF("Save LIS2DUXS12 MLC=%d\r\n", AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].Sample.Ui8Value);
#else /* SMARTAG2_VERBOSE_PRINTF */
             SMARTAG2_PRINTF("Save LIS2DUXS12 MLC\r\n");
#endif /* SMARTAG2_VERBOSE_PRINTF */

             DataBuf32 = LIS2DUXS12_MLC_VS_ID |
               (((uint32_t)AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].SampleDeltaDateTime)<<4);
             if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
               STNFC_Error_Handler(STNFC_WRITING_ERROR);
             }
             LogDefinition.LastSamplePointer+=4;
             DataBuf32 = AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].Sample.Ui8Value;
             if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
               STNFC_Error_Handler(STNFC_WRITING_ERROR);
             }
             LogDefinition.LastSamplePointer+=4;

             /* Increment the new Sample Counter until the end of the Tag */
             NfcType5_UpdateSampleCounter(&LogDefinition,8);

             /* Update Sample Counter and Last Sample Pointer */
             UpdateLastSamplePointerAndSampleCounter(&LogDefinition);

             AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].SampleDeltaDateTime=0;
           }
         }

         if(ReadSensorAndLog & SYNC_EVENT) {
           ReadSensorAndLog &= ~SYNC_EVENT;
           SMARTAG2_PRINTF("\r\nSync Event:\r\n");

           if((AllVirtualSensorsArray[VD6283_LUX_VS_ID].Enable) |
             (AllVirtualSensorsArray[VD6283_CCT_VS_ID].Enable) |
             (AllVirtualSensorsArray[STTS22H_VS_ID].Enable) |
             (AllVirtualSensorsArray[LPS22DF_VS_ID].Enable)) {
            MEMS_Sensors_ReadData();
             }
         }

         /* end of activelog */
SMARTAG2_SLEEP:

         BSP_LED_Off(LED2);
         if(NFCStatus == NFC_STATUS_ON) {
           ST25_RETRY(BSP_NFCTAG_ResetRFSleep_Dyn(BSP_NFCTAG_INSTANCE));
#ifdef SMARTAG2_VERBOSE_PRINTF
           SMARTAG2_PRINTF("\t-->WakeUp RF\r\n");
#endif /* SMARTAG2_VERBOSE_PRINTF */
           BSP_NFCTAG_LPD_On();
           NFCStatus = NFC_STATUS_OFF;
         }
       }
  }
}

/**
* @brief  Understanding the MEMS interrupt Event
* @param  None
* @retval None
*/
void SmarTagAppDetectMemsEvent(void)
{
  if(AccInit_LSM6DSO32X_Done != 0)
  {
    if(AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Enable | AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].Enable) {
      BSP_MOTION_SENSOR_Event_Status_t EventStatus;
      BSP_MOTION_SENSOR_Get_Event_Status( LSM6DSO32X_0, &EventStatus);

      if(!FirstEventNotApplicable) {
        int32_t AccEventVmax = DetectorValueForAccEvent();

        if((AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Enable!=0) &
           (EventStatus.WakeUpStatus != 0) &
             (AccEventVmax>AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Th1.Ui16Value)) {
               uint16_t ValueToCheck;
               ReadSensorAndLog |= ASYNC_EVENT;
               ValueToCheck = AccEventVmax;
               SMARTAG2_PRINTF("WakeUp=%d\r\n",ValueToCheck);
               /* Check the Value respect Min and Max Limit Values*/
               MCR_STNFC_CompareWithLimits(Ui16,AllVirtualSensorsArray[LSM6DSOX32_VS_ID],ValueToCheck);
               /* Compare with Ths and Update the Max/Min Sample Value */
               STNFC_ComputeMaxMinCompareTHsUi16t(&AllVirtualSensorsArray[LSM6DSOX32_VS_ID],&LogDefinition,&hrtc);

             } else {
               EventStatus.WakeUpStatus= 0;
             }

        if((AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].Enable!=0) &
           (EventStatus.D6DOrientationStatus != 0)) {
             static uint8_t LastSendOrientation = ORIENTATION_UNDEF;
             uint8_t SmarTagPosition = Understand6DOrientation();
             uint8_t ValueToCheck = SmarTagPosition;

             SMARTAG2_PRINTF("6D Orientation=%d\r\n",SmarTagPosition);
             if (SmarTagPosition != LastSendOrientation) {
               ReadSensorAndLog |= ASYNC_EVENT;
               LastSendOrientation = SmarTagPosition;
               /* Check the Value respect Min and Max Limit Values*/
               MCR_STNFC_CompareWithLimits(Ui8,AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID],ValueToCheck);
               /* Compare with Ths and Update the Max/Min Sample Value */
               STNFC_ComputeMaxMinCompareTHsUi8t(&AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID],&LogDefinition,&hrtc);
             } else {
               EventStatus.D6DOrientationStatus= 0;
             }
           }
      }
      /* Reset FIFO by setting FIFO mode to Bypass */
      BSP_MOTION_SENSOR_FIFO_Set_Mode(LSM6DSO32X_0, LSM6DSO32X_BYPASS_MODE);

      /* Set again the FIFO in Continuous to FIFO mode */
      BSP_MOTION_SENSOR_FIFO_Set_Mode(LSM6DSO32X_0, LSM6DSO32X_STREAM_TO_FIFO_MODE);

      FirstEventNotApplicable= 0;
    }

    if(AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].Enable) {
      lsm6dso32x_all_sources_t status;
      uint8_t MLCStatus;
      lsm6dso32x_all_sources_get(LSM6DSOX32_Contex, &status);
      MLCStatus = ((status.mlc1)    | (status.mlc2<<1) | (status.mlc3<<2) | (status.mlc4<<3) |
                   (status.mlc5<<4) | (status.mlc6<<5) | (status.mlc7<<6) | (status.mlc8<<7));

      if(MLCStatus!=0) {
        uint8_t mlc_out[8];
        uint16_t Angle;

        lsm6dso32x_mlc_out_get(LSM6DSOX32_Contex, mlc_out);
        Angle = ((uint16_t)mlc_out[0])*6;

#ifdef SMARTAG2_VERBOSE_PRINTF
        SMARTAG2_PRINTF("MLC Tilt =%d'\r\n",Angle);
#endif /* SMARTAG2_VERBOSE_PRINTF */

         /* Check the Value respect Min and Max Limit Values*/
         MCR_STNFC_CompareWithLimits(Ui8,AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID],Angle);

         /* Compare with Ths and Update the Max/Min Sample Value */
         STNFC_ComputeMaxMinCompareTHsUi8t(&AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID],&LogDefinition,&hrtc);
         if(AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].SampleDeltaDateTime!=0) {
           ReadSensorAndLog |= ASYNC_EVENT;
         }
      }
    }
  }

  if(AccInit_LIS2DUXS12_Done != 0)
  {
    if(AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].Enable) {
      lis2duxs12_mlc_status_mainpage_t status;
      uint8_t MLCStatus;
      lis2duxs12_mlc_status_get(LIS2DUXS12_Contex, &status);
      MLCStatus = ((status.is_mlc1) | (status.is_mlc2) | (status.is_mlc3) | (status.is_mlc4));

      if(MLCStatus!=0) {
        uint8_t mlc_out[8];
        uint8_t AT_Ouput;

        lis2duxs12_mlc_out_get(LIS2DUXS12_Contex, mlc_out);
        AT_Ouput = mlc_out[0];

#ifdef SMARTAG2_VERBOSE_PRINTF
        SMARTAG2_PRINTF("MLC AT =%d\r\n",AT_Ouput);
#endif /* SMARTAG2_VERBOSE_PRINTF */

        /* Check the Value respect Min and Max Limit Values*/
        MCR_STNFC_CompareWithLimits(Ui8,AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID],AT_Ouput);

        /* Compare with Ths and Update the Max/Min Sample Value */
        STNFC_ComputeMaxMinCompareTHsUi8t(&AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID],&LogDefinition,&hrtc);
        if(AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].SampleDeltaDateTime!=0) {
          ReadSensorAndLog |= ASYNC_EVENT;
        }
      }
    }
  }
}

/**
* @brief  Understanding the Interrupt from ST25DV
* @param  None
* @retval None
*/
void SmarTagAppDetectRFActivity(void)
{
  uint8_t ITStatus;
  static ST25DVxxKC_FIELD_STATUS_E RFfield = ST25DVXXKC_FIELD_OFF;
  uint8_t FieldOn=0;
  uint8_t FieldOff=0;

  if(NFCStatus == NFC_STATUS_OFF ) {
    BSP_NFCTAG_LPD_Off();
    /* rise time required by NFC */
    HAL_Delay(10);
    NFCStatus = NFC_STATUS_ON;
  }

  /* Read the IT status register */
  ST25_RETRY(BSP_NFCTAG_ReadITSTStatus_Dyn(BSP_NFCTAG_INSTANCE, &ITStatus));

  //Check the Falling Bit
  if((ITStatus & ST25DVXXKC_ITSTS_DYN_FIELDFALLING_MASK) == ST25DVXXKC_ITSTS_DYN_FIELDFALLING_MASK){
    FieldOff = 1;
  }

  //Check the Rising Bit
  if((ITStatus & ST25DVXXKC_ITSTS_DYN_FIELDRISING_MASK) == ST25DVXXKC_ITSTS_DYN_FIELDRISING_MASK) {
    FieldOn = 1;
  }

  //Make the decision
  if(((FieldOff == 1) & (FieldOn == 1)) |
     ((FieldOff == 0) & (FieldOn == 0) & (RFfield==ST25DVXXKC_FIELD_OFF)))
    {
     // can't decide, need to read the register to get actual state
     static ST25DVxxKC_FIELD_STATUS_E field = ST25DVXXKC_FIELD_OFF;
     int32_t status = BSP_NFCTAG_GetRFField_Dyn(BSP_NFCTAG_INSTANCE,&field);
     if((field == ST25DVXXKC_FIELD_ON) || (status == NFCTAG_NACK)) {
       if(FieldOn || FieldOff) {
         // Off->On
         SMARTAG2_PRINTF("\r\nDetected NFC FIELD Off->On\r\n");
         RFActivityStatus = FIELD_RISING;
         RFfield = ST25DVXXKC_FIELD_ON;
       }
     } else {
        if(FieldOn || FieldOff) {
          //On->Off
          SMARTAG2_PRINTF("\r\nDetected NFC FIELD On->Off\r\n");
          RFfield = ST25DVXXKC_FIELD_OFF;
          RFActivityStatus = FIELD_FALLING;
       }
     }
  } else {
    if((FieldOff == 0) & (FieldOn == 1)) {
      // On
      SMARTAG2_PRINTF("\r\nDetected NFC FIELD On\r\n");
      RFActivityStatus = FIELD_RISING;
    } else if((FieldOff == 1) & (FieldOn == 0)) {
      //Off
      SMARTAG2_PRINTF("\r\nDetected NFC FIELD Off\r\n");
      RFActivityStatus = FIELD_FALLING;
    }
  }

 //When the Phone is no more under the Tag
 if(RFActivityStatus == FIELD_FALLING) {
    /* Control if there is a new configuration */
    ST25_RETRY(BSP_NFCTAG_SetRFSleep_Dyn(BSP_NFCTAG_INSTANCE));
#ifdef SMARTAG2_VERBOSE_PRINTF
    SMARTAG2_PRINTF("\t-->Sleep RF\r\n");
#endif /* SMARTAG2_VERBOSE_PRINTF */
    CheckIfNewConfiguration();
    ST25_RETRY(BSP_NFCTAG_ResetRFSleep_Dyn(BSP_NFCTAG_INSTANCE));
#ifdef SMARTAG2_VERBOSE_PRINTF
    SMARTAG2_PRINTF("\t-->WakeUp RF\r\n");
#endif /* SMARTAG2_VERBOSE_PRINTF */
    BSP_NFCTAG_LPD_On();
    NFCStatus = NFC_STATUS_OFF;
  }
}

/**
* @brief  Set the NFC Security Level, GPO behavior and Energy harvesting mode
* @param  None
* @retval None
*/
static void SetNFCBehavior(void)
{
  /* Setting the New Password for I2C protection */
  if(BSP_NFCTAG_ChangeI2CPassword(0x90ABCDEF,0x12345678)!=NFCTAG_OK ) {
    STNFC_Error_Handler(STNFC_CONFIG_ERROR);
  }

  /* GPO sensible to RF Field change */
  if(BSP_NFCTAG_WriteConfigIT(0x90ABCDEF,0x12345678,ST25DVXXKC_GPO1_ENABLE_MASK | ST25DVXXKC_GPO1_FIELDCHANGE_MASK)!=NFCTAG_OK ) {
    STNFC_Error_Handler(STNFC_CONFIG_ERROR);
  }

  /* Setting the Energy harvesting mode */
  if(BSP_NFCTAG_CheckChangeEHMODE(0x90ABCDEF,0x12345678,ST25DVXXKC_EH_ON_DEMAND)!=NFCTAG_OK ) {
    STNFC_Error_Handler(STNFC_CONFIG_ERROR);
  }
}

/**
* @brief  Accelerometer events initialization
* @retval None
*/
static int32_t InitDeInitAccEventThreshold(void)
{
  int32_t Success= BSP_ERROR_NONE;

  static uint8_t FirstTime_LSM6DSOX32=0;
  static uint8_t FirstTime_LIS2DUXS12=0;

  FirstEventNotApplicable= 1;

  if((AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Enable) |
     (AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].Enable) |
     (AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].Enable) |
     (AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].Enable) ) {

       SMARTAG2_PRINTF("Init Accelerometer Events:\r\n");

       if((AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Enable) |
          (AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].Enable) |
          (AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].Enable)) {

            if(FirstTime_LSM6DSOX32)
            {
              BSP_MOTION_SENSOR_DeInit(LSM6DSO32X_0);
            }
            FirstTime_LSM6DSOX32= 1;

            if(BSP_MOTION_SENSOR_Init(LSM6DSO32X_0, MOTION_ACCELERO)!=BSP_ERROR_NONE) {
              SMARTAG2_PRINTF("Error Init LSM6DSOX32\r\n");
            } else {
              SMARTAG2_PRINTF("Init LSM6DSOX32\r\n");
            }

           /* Enable wake up events */
           if(AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Enable){
             /* Enable Wake Up */
             uint8_t WakeUpTHS= 1;
             uint32_t Th_Max;
             SMARTAG2_PRINTF("WakeUp On\r\n");
             Success = BSP_MOTION_SENSOR_Enable_Wake_Up_Detection(LSM6DSO32X_0, MOTION_SENSOR_INT1_PIN);

             if(Success!=BSP_ERROR_NONE) {
               SMARTAG2_PRINTF("\r\nError enabling WakeUp\r\n");
               return Success;
             }

             Success = BSP_MOTION_SENSOR_SetFullScale(LSM6DSO32X_0, MOTION_ACCELERO, 16); /* FullScale 16G */

             if(Success!=BSP_ERROR_NONE) {
               SMARTAG2_PRINTF("\r\nError Setting Full Scale\r\n");
               return Success;
             }

             Th_Max= AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Th1.Ui16Value>>8;
             if(Th_Max < 1) {
               WakeUpTHS= 1;
             } else if(Th_Max > 63) {
               WakeUpTHS= 63;
             } else {
               WakeUpTHS = Th_Max;
             }

      #ifdef SMARTAG2_VERBOSE_PRINTF
             SMARTAG2_PRINTF("Acc_Th_Max= %ld\tWakeUpTHS= %d\r\n", AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Th1.Ui16Value, WakeUpTHS);
      #endif /* SMARTAG2_VERBOSE_PRINTF */

             Success = BSP_MOTION_SENSOR_Set_Wake_Up_Threshold(LSM6DSO32X_0, WakeUpTHS);
             if(Success!=BSP_ERROR_NONE) {
               SMARTAG2_PRINTF("\r\nError Setting WakeUp Threshold\r\n");
               return Success;
             }
           } else {
             /* Disable Wake Up */
             SMARTAG2_PRINTF("WakeUp Off\r\n");
             Success = BSP_MOTION_SENSOR_Disable_Wake_Up_Detection(LSM6DSO32X_0);
             if(Success!=BSP_ERROR_NONE) {
               SMARTAG2_PRINTF("\r\nError disabling WakeUp\r\n");
               return Success;
             }
           }

           /* Enable 6D orientation events */
           if(AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].Enable) {
             /* Enable 6D Orientation */
             SMARTAG2_PRINTF("6D On\r\n");
             Success = BSP_MOTION_SENSOR_Enable_6D_Orientation(LSM6DSO32X_0, MOTION_SENSOR_INT1_PIN);
             if(Success!=BSP_ERROR_NONE) {
               SMARTAG2_PRINTF("\r\nError enabling 6D\r\n");
               return Success;
             }
             Success = BSP_MOTION_SENSOR_SetFullScale(LSM6DSO32X_0, MOTION_ACCELERO, 16); /* FullScale 16G */
             if(Success!=BSP_ERROR_NONE) {
               SMARTAG2_PRINTF("\r\nError Setting 6D Threshold\r\n");
               return Success;
             }
           } else {
             /* Disable 6D Orientation */
             SMARTAG2_PRINTF("6D Off\r\n");
             Success = BSP_MOTION_SENSOR_Disable_6D_Orientation(LSM6DSO32X_0);
             if(Success!=BSP_ERROR_NONE) {
               SMARTAG2_PRINTF("\r\nError Disabling 6D\r\n");
               return Success;
             }
           }

           SMARTAG2_PRINTF("\r\n");

           if((AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Enable) |
              (AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].Enable) ) {
                /* Set the FIFO in Continuous to FIFO mode */
                {
                  float Bdr;
                  Success = BSP_MOTION_SENSOR_GetOutputDataRate(LSM6DSO32X_0,MOTION_ACCELERO,&Bdr);
                  if(Success!=BSP_ERROR_NONE) {
                    SMARTAG2_PRINTF("\r\nError Reading ODR\r\n");
                    return Success;
                  }
                  Success = BSP_MOTION_SENSOR_FIFO_Set_BDR(LSM6DSO32X_0,MOTION_ACCELERO,Bdr);
                  if(Success!=BSP_ERROR_NONE) {
                    SMARTAG2_PRINTF("\r\nError Setting FIFO BDR\r\n");
                    return Success;
                  }
                }
                Success = BSP_MOTION_SENSOR_FIFO_Set_Mode(LSM6DSO32X_0, LSM6DSO32X_STREAM_TO_FIFO_MODE);
                if(Success!=BSP_ERROR_NONE) {
                  SMARTAG2_PRINTF("\r\nError Setting FIFO Mode\r\n");
                }
              } else {
                /* Set the FIFO Bypass mode */
                Success = BSP_MOTION_SENSOR_FIFO_Set_Mode(LSM6DSO32X_0, LSM6DSO32X_BYPASS_MODE);
                if(Success!=BSP_ERROR_NONE) {
                  SMARTAG2_PRINTF("\r\nError Setting FIFO Mode\r\n");
                }
              }

             /* lsm6dso32x MLC sensor: Init */

           if(AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].Enable) {
             ucf_line_t *ProgramPointer;
             int32_t LineCounter;
             int32_t TotalNumberOfLine;
             int32_t RetValue;

             ProgramPointer = (ucf_line_t *)lsm6dso32x_tilt_angle_mode0;
             TotalNumberOfLine = sizeof(lsm6dso32x_tilt_angle_mode0) / sizeof(ucf_line_t);
             for (LineCounter=0; LineCounter<TotalNumberOfLine; LineCounter++) {
               RetValue = BSP_MOTION_SENSOR_Write_Register(LSM6DSO32X_0,
                                                           ProgramPointer[LineCounter].address,
                                                           ProgramPointer[LineCounter].data);
               if(RetValue!=BSP_ERROR_NONE) {
                 SMARTAG2_PRINTF("Error loading the Program to LSM6DSO32X [%ld]->%lx\n\r",LineCounter,RetValue);
                 STNFC_Error_Handler(STNFC_INIT_ERROR);
               }
             }
               SMARTAG2_PRINTF("MLC program Loaded on LSM6DSO32X [%ld]\r\n",TotalNumberOfLine);
               SMARTAG2_PRINTF("    Detect angles from 0 to 90 degrees\r\n");
           }

           AccInit_LSM6DSO32X_Done= 1;
          } else {
            if(FirstTime_LSM6DSOX32)
            {
              BSP_MOTION_SENSOR_DeInit(LSM6DSO32X_0);
            }

            AccInit_LSM6DSO32X_Done= 0;
          }

       if((AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].Enable) |
          (AllVirtualSensorsArray[LIS2DUXS12_VS_ID].Enable) |
          (AllVirtualSensorsArray[LIS2DUXS12_6D_VS_ID].Enable)) {

            if(FirstTime_LIS2DUXS12)
            {
              BSP_MOTION_SENSOR_DeInit(LIS2DUXS12_0);
            }
            FirstTime_LIS2DUXS12= 1;

            if(BSP_MOTION_SENSOR_Init(LIS2DUXS12_0, MOTION_ACCELERO)!=BSP_ERROR_NONE) {
               SMARTAG2_PRINTF("Error Init LIS2DUXS12\r\n");
             } else {
               SMARTAG2_PRINTF("Init LIS2DUXS12\r\n");
             }

             /* lis2duxs12 MLC sensor: Init */
             if(AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].Enable) {
               ucf_line_ext_t *ProgramPointer;
               int32_t LineCounter;
               int32_t TotalNumberOfLine;
               int32_t RetValue;

               ProgramPointer = (ucf_line_ext_t *)at_lp;
               TotalNumberOfLine = sizeof(at_lp) / sizeof(ucf_line_ext_t);
               for (LineCounter=0; LineCounter<TotalNumberOfLine; LineCounter++) {
                 switch(ProgramPointer[LineCounter].op) {
                 case MEMS_UCF_OP_WRITE:
                   /* MEMS_UCF_OP_WRITE: write the value specified by the "data" field at the
                    *  location specified by the "address" field */
                   RetValue = BSP_MOTION_SENSOR_Write_Register(LIS2DUXS12_0,
                                                             ProgramPointer[LineCounter].address,
                                                             ProgramPointer[LineCounter].data);
                 break;
                 case MEMS_UCF_OP_DELAY:
                   /* MEMS_UCF_OP_DELAY: wait the number of milliseconds specified by the "data"
                    * field ("address" field is ignored) */
                   HAL_Delay(ProgramPointer[LineCounter].data);
                 break;
                 case MEMS_UCF_OP_POLL_SET:
                   /* MEMS_UCF_OP_POLL_SET: poll the register at the location specified by the
                    * "address" field until all the bits identified by the mask specified by the
                    * "data" field are set to 1 */
                   {
                     uint8_t Data;
                     do {
                      RetValue = BSP_MOTION_SENSOR_Read_Register(LIS2DUXS12_0, ProgramPointer[LineCounter].address,&Data);
                      HAL_Delay(1);
                     } while((Data & ProgramPointer[LineCounter].data)!=ProgramPointer[LineCounter].data);
                   }
                 break;
                 case MEMS_UCF_OP_POLL_RESET:
                   /* MEMS_UCF_OP_POLL_RESET: poll the register at the location specified by the
                    * "address" field until all the bits identified by the mask specified by the
                    * "data" field are reset to 0 */
                   {
                     uint8_t Data;
                     do {
                      RetValue = BSP_MOTION_SENSOR_Read_Register(LIS2DUXS12_0, ProgramPointer[LineCounter].address,&Data);
                      HAL_Delay(1);
                     } while((Data & ProgramPointer[LineCounter].data)!=0);
                   }
                 break;
                 case MEMS_UCF_OP_READ:
                   /*  MEMS_UCF_OP_READ: read the register at the location specified by the
                    *  "address" field ("data" field is ignored */
                   {
                     uint8_t Data;
                     RetValue = BSP_MOTION_SENSOR_Read_Register(LIS2DUXS12_0, ProgramPointer[LineCounter].address,&Data);
                   }
                 break;
                 }

                 if(RetValue!=BSP_ERROR_NONE) {
                   SMARTAG2_PRINTF("Error loading the Program to LIS2DUXS12 [%ld]->%lx\n\r",LineCounter,RetValue);
                   STNFC_Error_Handler(STNFC_INIT_ERROR);
                 }
               }
               SMARTAG2_PRINTF("MLC program Loaded on LIS2DUXS12 [%ld]\r\n",TotalNumberOfLine);
               SMARTAG2_PRINTF("    Asset Tracking Low Power\r\n");
             }

             AccInit_LIS2DUXS12_Done= 1;
          } else {
            AccInit_LIS2DUXS12_Done= 0;
            if(FirstTime_LIS2DUXS12)
            {
              BSP_MOTION_SENSOR_DeInit(LIS2DUXS12_0);
            }
          }
     }
  return Success;
}

/**
* @brief  Checks if there is a new configuration.
* @param  None
* @retval None
*/
static void CheckIfNewConfiguration(void)
{
  SNFC_LogDefinition_t LocalLogDefinition;

  SMARTAG2_PRINTF("Check if there is a new Configuration\r\n");
  ReadConfiguration(&LocalLogDefinition,LogDefinition.StartDateTime,1);

  if(LocalLogDefinition.StartDateTime!=0) {
    int32_t SensorNum;
    SMARTAG2_PRINTF("Restart the Log\r\n");
    /* Disable all the Previous Enabled Virtual Sensors */
    for(SensorNum=0;SensorNum<LogDefinition.VirtualSensorsNum;SensorNum++) {
      ConfiguratedVirtualSensorsArray[SensorNum]->Enable=0;
      ConfiguratedVirtualSensorsArray[SensorNum]->SampleDeltaDateTime=0;
    }

    /* Read again and update the configuration restarting the log */
    ReadConfiguration(&LogDefinition,LogDefinition.StartDateTime,0);

    /* Set the Date&Time */
    if(STNFC_SetDateTime(LogDefinition.StartDateTime,&hrtc,&LogDefinition)==0) {
      SMARTAG2_PRINTF("Error: Setting RTC\r\n");
    } else {
       SMARTAG2_PRINTF("Set RTC Date&Time\r\n");
    }

    /* Reset the Max/Min For each Sensor */
    ResetMaxMinValuesAllVirtualSensors();

    /* Save the Max/Min for each Sensor */
    SaveMaxMinValuesForVirtualSensors();

    /* Write Sample Counter and Last Sample Pointer*/
    LogDefinition.SampleCounterAddress = LogDefinition.LastSamplePointer;
    LogDefinition.SampleCounter=0;
    UpdateLastSamplePointerAndSampleCounter(&LogDefinition);
    LogDefinition.LastSamplePointer+=8;

    NfcType5_SetInitialNDEFPayLoadLengthValue(LogDefinition.LastSamplePointer);

    NfcType5_ComputeNDEFPayLoadSize(LogDefinition.LastSamplePointer,LogDefinition.SampleCounter);

    /* Power On/Off the Intertial sensors */
    if((AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Enable) |
       (AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].Enable) |
       (AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].Enable) |
       (AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].Enable)  ) {
       BSP_MOTION_SENSOR_PowerOn();
       SMARTAG2_PRINTF("VDD ACC On\r\n");
     } else {
        BSP_MOTION_SENSOR_PowerOff();
       SMARTAG2_PRINTF("VDD ACC Off\r\n");
     }

    /* Rise time required by VDD_SENS and MEMS sensor startup time */
    HAL_Delay(200);

    /* lsm6dso32x WakeUp/6D Rec or MLC: Init/DeInit */
    InitDeInitAccEventThreshold();

    /* we go in ACTIVE Mode */
    LogMode = SMARTAG2_LOGMODE_ACTIVE;
  }
}

/**
* @brief  Reads the active log configuration
* @param  SNFC_LogDefinition_t *LogDefinition
* @param  uint32_t CurrentStartDateTime  Actual Start Date&Time Value
* @param  int32_t OnlyChecks if ==1 it controls only if the configuration is valid
* @retval None
*/
static void ReadConfiguration(SNFC_LogDefinition_t *LogDefinition,uint32_t CurrentStartDateTime,int32_t OnlyChecks)
{
  uint32_t DataBuf32;
  uint8_t *DataBuf8 = (uint8_t *)&DataBuf32;
  uint16_t *DataBuf16 = (uint16_t *)&DataBuf32;
  SNFC_ValidConfiguration_t ValidConf=STNFC_VALID_CONFIG;
  SNFC_CodeDefinition_t *LocalSmarTag2CodeHeader = (SNFC_CodeDefinition_t *)&DataBuf32;

  LogDefinition->LastSamplePointer = SMARTAG2_START_ADDR_OFFSET;
  if(BSP_NFCTAG_ReadData( BSP_NFCTAG_INSTANCE, DataBuf8, LogDefinition->LastSamplePointer, 4 )!=NFCTAG_OK) {
    STNFC_Error_Handler(STNFC_READING_ERROR);
    ValidConf=STNFC_ERROR_READING_CONFIG;
  }

  if(ValidConf==STNFC_VALID_CONFIG) {
    LogDefinition->LastSamplePointer+=4;

    /* Check the protocol header */
    if((LocalSmarTag2CodeHeader->ProtVersion  != SMARTAG2_RECORD_VERSION) |
       (LocalSmarTag2CodeHeader->ProtRevision != SMARTAG2_RECORD_REVISION) |
       (LocalSmarTag2CodeHeader->BoardId      != SMARTAG2_BOARD_ID) |
       (LocalSmarTag2CodeHeader->FirmwareId   != SMARTAG2_FIRMWARE_ID)) {
             SMARTAG2_PRINTF("Error: Protocol Header not valid\r\n");
             ValidConf=STNFC_NOT_VALID_CONFIG;
           } else if(OnlyChecks==0){
             //If we are not making only the check... update the configuration
             memcpy(&SmarTag2CodeHeader,LocalSmarTag2CodeHeader,sizeof(SNFC_CodeDefinition_t));
           }
  }

  if(ValidConf==STNFC_VALID_CONFIG) {
    if(BSP_NFCTAG_ReadData( BSP_NFCTAG_INSTANCE, DataBuf8, LogDefinition->LastSamplePointer, 4 )!=NFCTAG_OK) {
      STNFC_Error_Handler(STNFC_READING_ERROR);
      ValidConf=STNFC_ERROR_READING_CONFIG;
    }
    if(ValidConf==STNFC_VALID_CONFIG) {
      LogDefinition->LastSamplePointer+=4;

      LogDefinition->SingleShotDone = DataBuf8[0];
      LogDefinition->VirtualSensorsNum = DataBuf8[1];
      LogDefinition->SampleTime = DataBuf16[1];

      SMARTAG2_PRINTF("\tVn=%d SampleTime=%d\r\n",LogDefinition->VirtualSensorsNum,LogDefinition->SampleTime);

      if(LogDefinition->VirtualSensorsNum==0) {
        SMARTAG2_PRINTF("\tError: VirtualSensorsNum==0\r\n");
        ValidConf=STNFC_NOT_VALID_CONFIG;
      }

      if(LogDefinition->VirtualSensorsNum>SMARTAG2_VIRTUAL_SENSORS_NUM) {
        SMARTAG2_PRINTF("\tError: VirtualSensorsNum >%d\r\n",SMARTAG2_VIRTUAL_SENSORS_NUM);
        ValidConf=STNFC_NOT_VALID_CONFIG;
      }

      if((LogDefinition->SampleTime<1) | (LogDefinition->SampleTime> (UINT16_MAX-1))) {
        SMARTAG2_PRINTF("\tError: SampleTime =%d Not Valid\r\n",LogDefinition->SampleTime);
        ValidConf=STNFC_NOT_VALID_CONFIG;
      }
    }
  }

  if(ValidConf==STNFC_VALID_CONFIG) {
    if(BSP_NFCTAG_ReadData( BSP_NFCTAG_INSTANCE, DataBuf8, LogDefinition->LastSamplePointer, 4 )!=NFCTAG_OK) {
      STNFC_Error_Handler(STNFC_READING_ERROR);
      ValidConf=STNFC_ERROR_READING_CONFIG;
    }
    if(ValidConf==STNFC_VALID_CONFIG) {
      LogDefinition->LastSamplePointer+=4;

      LogDefinition->StartDateTime=DataBuf32;
      //If we have a valid not null Start Date&Time and different respect the Current one used...-> New Configuration
      if(LogDefinition->StartDateTime!=0) {
        if(LogDefinition->StartDateTime ==CurrentStartDateTime) {
          SMARTAG2_PRINTF("Start Time not Changed\r\n");
          ValidConf=STNFC_NOT_CHANGED_CONFIG;
        }
      } else {
        SMARTAG2_PRINTF("Start Time ==0\r\n");
        ValidConf=STNFC_NOT_VALID_CONFIG;
      }
    }
  }

  if(ValidConf==STNFC_VALID_CONFIG) {
    /* We need to read the Virtual Sensor Configuration */
    int32_t SensorNum;
    for(SensorNum=0;((SensorNum<LogDefinition->VirtualSensorsNum)&(ValidConf==STNFC_VALID_CONFIG));SensorNum++) {
      if(BSP_NFCTAG_ReadData( BSP_NFCTAG_INSTANCE, DataBuf8, LogDefinition->LastSamplePointer, 4 )!=NFCTAG_OK) {
        STNFC_Error_Handler(STNFC_READING_ERROR);
        ValidConf=STNFC_ERROR_READING_CONFIG;
      }
      if(ValidConf==STNFC_VALID_CONFIG) {
        LogDefinition->LastSamplePointer+=4;
        switch(DataBuf32&0xF) {
        case LSM6DSOX32_MLC_VS_ID:
          SMARTAG2_PRINTF("\tFound LSM6DSOX32_MLC_VS_ID:\r\n");
          if(OnlyChecks==0) {
            ConfiguratedVirtualSensorsArray[SensorNum] = &AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID];
            ConfiguratedVirtualSensorsArray[SensorNum]->Enable=1;
            ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType = (SNFC_ThresholdsUsage_t)((DataBuf32>>4)&0x3);
            ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui8Value = (DataBuf32>>(4+2))&0xFF;
            SMARTAG2_PRINTF("\tThsUsageType=%s\r\n",ThresholdsUsageName[ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType]);
            SMARTAG2_PRINTF("\tTh1.Ui8Value=%d\r\n",ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui8Value);
            if(ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType<TH_LESS) {
              ConfiguratedVirtualSensorsArray[SensorNum]->Th2.Ui8Value = (DataBuf32>>(4+2+8))&0xFF;
              SMARTAG2_PRINTF("\tTh2.Ui8Value=%d\r\n",ConfiguratedVirtualSensorsArray[SensorNum]->Th2.Ui8Value);
            }

            /* Check Virtual Sensors Incompatibility */
            if(AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Enable) {
              SMARTAG2_PRINTF("\tWarning: Incompatibility with WakeUp\r\n\tDisable WakeUp\r\n");
              AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Enable=0;
            }
            if(AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].Enable) {
               SMARTAG2_PRINTF("\tWarning Incompatibility with 6D Orientation\r\n\tDisable 6D Orientation\r\n");
               AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].Enable=0;
            }
          }
          break;

        case LIS2DUXS12_MLC_VS_ID:
          SMARTAG2_PRINTF("\tFound LIS2DUXS12_MLC_VS_ID:\r\n");
          if(OnlyChecks==0) {
            ConfiguratedVirtualSensorsArray[SensorNum] = &AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID];
            ConfiguratedVirtualSensorsArray[SensorNum]->Enable=1;
            ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType = (SNFC_ThresholdsUsage_t)((DataBuf32>>4)&0x3);
            ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui8Value = (DataBuf32>>(4+2))&0xFF;
            SMARTAG2_PRINTF("\tThsUsageType=%s\r\n",ThresholdsUsageName[ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType]);
            SMARTAG2_PRINTF("\tTh1.Ui8Value=%d\r\n",ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui8Value);
            if(ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType<TH_LESS) {
              ConfiguratedVirtualSensorsArray[SensorNum]->Th2.Ui8Value = (DataBuf32>>(4+2+8))&0xFF;
              SMARTAG2_PRINTF("\tTh2.Ui8Value=%d\r\n",ConfiguratedVirtualSensorsArray[SensorNum]->Th2.Ui8Value);
            }

            /* Check Virtual Sensors Incompatibility */
            if(AllVirtualSensorsArray[LIS2DUXS12_VS_ID].Enable) {
              SMARTAG2_PRINTF("\tWarning: Incompatibility with WakeUp\r\n\tDisable WakeUp\r\n");
              AllVirtualSensorsArray[LIS2DUXS12_VS_ID].Enable=0;
            }
            if(AllVirtualSensorsArray[LIS2DUXS12_6D_VS_ID].Enable) {
               SMARTAG2_PRINTF("\tWarning Incompatibility with 6D Orientation\r\n\tDisable 6D Orientation\r\n");
               AllVirtualSensorsArray[LIS2DUXS12_6D_VS_ID].Enable=0;
            }
          }
          break;

        case STTS22H_VS_ID:
          SMARTAG2_PRINTF("\tFound STTS22H_VS_ID:\r\n");
          if(OnlyChecks==0) {
            ConfiguratedVirtualSensorsArray[SensorNum] = &AllVirtualSensorsArray[STTS22H_VS_ID];
            ConfiguratedVirtualSensorsArray[SensorNum]->Enable=1;
            ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType = (SNFC_ThresholdsUsage_t)((DataBuf32>>4)&0x3);
            ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui16Value = (DataBuf32>>(4+2))&0x1FF;
            SMARTAG2_PRINTF("\tThsUsageType=%s\r\n",ThresholdsUsageName[ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType]);
            SMARTAG2_PRINTF("\tTh1.Ui16Value=%f\r\n",STTS22H_CODED_TO_SAMPLE(ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui16Value));
            if(ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType<TH_LESS) {
              ConfiguratedVirtualSensorsArray[SensorNum]->Th2.Ui16Value = (DataBuf32>>(4+2+9))&0x1FF;
              SMARTAG2_PRINTF("\tTh2.Ui16Value=%f\r\n",STTS22H_CODED_TO_SAMPLE(ConfiguratedVirtualSensorsArray[SensorNum]->Th2.Ui16Value));
            }
          }
          break;

        case LPS22DF_VS_ID:
          SMARTAG2_PRINTF("\tFound LPS22DF_VS_ID:\r\n");
          if(OnlyChecks==0) {
            ConfiguratedVirtualSensorsArray[SensorNum] = &AllVirtualSensorsArray[LPS22DF_VS_ID];
            ConfiguratedVirtualSensorsArray[SensorNum]->Enable=1;
            ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType = (SNFC_ThresholdsUsage_t)((DataBuf32>>4)&0x3);
            ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui16Value = (DataBuf32>>(4+2))&0x7FF;
            SMARTAG2_PRINTF("\tThsUsageType=%s\r\n",ThresholdsUsageName[ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType]);
            SMARTAG2_PRINTF("\tTh1.Ui16Value=%f\r\n",LPS22DF_CODED_TO_SAMPLE(ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui16Value));
            if(ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType<TH_LESS) {
              ConfiguratedVirtualSensorsArray[SensorNum]->Th2.Ui16Value = (DataBuf32>>(4+2+11))&0x7FF;
              SMARTAG2_PRINTF("\tTh2.Ui16Value=%f\r\n",LPS22DF_CODED_TO_SAMPLE(ConfiguratedVirtualSensorsArray[SensorNum]->Th2.Ui16Value));
            }
          }
          break;

        case VD6283_LUX_VS_ID:
          SMARTAG2_PRINTF("\tFound VD6283_LUX_VS_ID:\r\n");
          if(OnlyChecks==0) {
            ConfiguratedVirtualSensorsArray[SensorNum] = &AllVirtualSensorsArray[VD6283_LUX_VS_ID];
            ConfiguratedVirtualSensorsArray[SensorNum]->Enable=1;
            ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType = (SNFC_ThresholdsUsage_t)((DataBuf32>>4)&0x3);
            ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui32Value = (DataBuf32>>(4+2))&0x1FFFFFF;
            SMARTAG2_PRINTF("\tThsUsageType=%s\r\n",ThresholdsUsageName[ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType]);
            SMARTAG2_PRINTF("\tTh1.Ui32Value=%f\r\n",VD6283_LUX_CODED_TO_SAMPLE(ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui32Value));
          }
          break;

        case LSM6DSOX32_VS_ID:
          SMARTAG2_PRINTF("\tFound LSM6DSOX32_VS_ID:\r\n");
          if(OnlyChecks==0) {
            ConfiguratedVirtualSensorsArray[SensorNum] = &AllVirtualSensorsArray[LSM6DSOX32_VS_ID];
            ConfiguratedVirtualSensorsArray[SensorNum]->Enable=1;
            ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType = (SNFC_ThresholdsUsage_t)((DataBuf32>>4)&0x3);
            ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui16Value = (DataBuf32>>(4+2))&0x7FFFF;
            SMARTAG2_PRINTF("\tThsUsageType=%s\r\n",ThresholdsUsageName[ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType]);
            SMARTAG2_PRINTF("\tTh1.Ui16Value=%d\r\n",ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui16Value);

            /* Check Virtual Sensors Incompatibility */
            if(AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].Enable) {
               SMARTAG2_PRINTF("\tWarning Incompatibility with MLC\r\n\tDisable MLC\r\n");
               AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].Enable=0;
            }
          }
          break;

        case LSM6DSOX32_6D_VS_ID:
          SMARTAG2_PRINTF("\tFound LSM6DSOX32_6D_VS_ID:\r\n");
          if(OnlyChecks==0) {
            ConfiguratedVirtualSensorsArray[SensorNum] = &AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID];
            ConfiguratedVirtualSensorsArray[SensorNum]->Enable=1;
            ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType = (SNFC_ThresholdsUsage_t)((DataBuf32>>4)&0x3);
            ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui8Value = (DataBuf32>>(4+2))&0xFF;
            SMARTAG2_PRINTF("\tThsUsageType=%s\r\n",ThresholdsUsageName[ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType]);
            SMARTAG2_PRINTF("\tTh1.Ui8Value=%d\r\n",ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui8Value);
            if(ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType<TH_LESS) {
              ConfiguratedVirtualSensorsArray[SensorNum]->Th2.Ui8Value = (DataBuf32>>(4+2+8))&0xFF;
              SMARTAG2_PRINTF("\tTh2.Ui8Value=%d\r\n",ConfiguratedVirtualSensorsArray[SensorNum]->Th2.Ui8Value);
            }

            /* Check Virtual Sensors Incompatibility */
            if(AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].Enable) {
               SMARTAG2_PRINTF("\tWarning Incompatibility with MLC\r\n\tDisable MLC\r\n");
               AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].Enable=0;
            }
          }
          break;
        default:
          ValidConf=STNFC_NOT_VALID_CONFIG;
          SMARTAG2_PRINTF("Error: Not recognized VirtualSensorID=%ld\r\n",DataBuf32&0xF);
        }
      }
    }
  }

  switch(ValidConf) {
    case STNFC_NOT_VALID_CONFIG:
      SMARTAG2_PRINTF("Not Valid Configuration present on NFC Skip it\r\n");
      LogDefinition->StartDateTime=0;
    break;
    case STNFC_ERROR_READING_CONFIG:
      SMARTAG2_PRINTF("Error Reading Configuration present on NFC Skip it\r\nTry again to write the new configuration\r\n");
      LogDefinition->StartDateTime=0;
    break;
    case STNFC_VALID_CONFIG:
      if(OnlyChecks) {
        SMARTAG2_PRINTF("Valid Configuration present on NFC\r\n");
      }
    break;
    case STNFC_NOT_CHANGED_CONFIG:
      if(OnlyChecks) {
        SMARTAG2_PRINTF("Going on with the current Configuration\r\n");
      }
      LogDefinition->StartDateTime=0;
    break;
  }
}

/**
* @brief  Set all Avaialbles Virtual Sensor Configuration
* @param  None
* @retval None
*/
static void SetAllAvailableVirtualSensors(void)
{
  //Initialize the virtual sensors
  //Number 0
  AllVirtualSensorsArray[STTS22H_VS_ID].VirtualSensorId = STTS22H_VS_ID;
  AllVirtualSensorsArray[STTS22H_VS_ID].Enable=0;
  AllVirtualSensorsArray[STTS22H_VS_ID].SensorType=VST_UI16;
  AllVirtualSensorsArray[STTS22H_VS_ID].ThsUsageType=TH_INT;
  AllVirtualSensorsArray[STTS22H_VS_ID].Th1.Ui16Value = STTS22H_SAMPLE_TO_CODED(22);   //'C
  AllVirtualSensorsArray[STTS22H_VS_ID].Th2.Ui16Value = STTS22H_SAMPLE_TO_CODED(43.2); //'C
  AllVirtualSensorsArray[STTS22H_VS_ID].MaxLimit.Ui16Value = STTS22H_SAMPLE_TO_CODED(60); //'C
  AllVirtualSensorsArray[STTS22H_VS_ID].MinLimit.Ui16Value = STTS22H_SAMPLE_TO_CODED(-10); //'C

  //Number 1
  AllVirtualSensorsArray[LPS22DF_VS_ID].VirtualSensorId = LPS22DF_VS_ID;
  AllVirtualSensorsArray[LPS22DF_VS_ID].Enable=0;
  AllVirtualSensorsArray[LPS22DF_VS_ID].SensorType=VST_UI16;
  AllVirtualSensorsArray[LPS22DF_VS_ID].ThsUsageType=TH_BIGGER;
  AllVirtualSensorsArray[LPS22DF_VS_ID].Th1.Ui16Value = LPS22DF_SAMPLE_TO_CODED(960); //hPa
  AllVirtualSensorsArray[LPS22DF_VS_ID].MaxLimit.Ui16Value = LPS22DF_SAMPLE_TO_CODED(1260); //hPa
  AllVirtualSensorsArray[LPS22DF_VS_ID].MinLimit.Ui16Value = LPS22DF_SAMPLE_TO_CODED(260); //hPa

  //Number 2
  AllVirtualSensorsArray[VD6283_LUX_VS_ID].VirtualSensorId = VD6283_LUX_VS_ID;
  AllVirtualSensorsArray[VD6283_LUX_VS_ID].Enable=0;
  AllVirtualSensorsArray[VD6283_LUX_VS_ID].SensorType=VST_UI32;
  AllVirtualSensorsArray[VD6283_LUX_VS_ID].ThsUsageType=TH_BIGGER;
  AllVirtualSensorsArray[VD6283_LUX_VS_ID].Th1.Ui32Value = VD6283_LUX_SAMPLE_TO_CODED(100); //KLux
  AllVirtualSensorsArray[VD6283_LUX_VS_ID].MaxLimit.Ui32Value = VD6283_LUX_SAMPLE_TO_CODED(30*1024); //KLux
  AllVirtualSensorsArray[VD6283_LUX_VS_ID].MinLimit.Ui32Value = VD6283_LUX_SAMPLE_TO_CODED(0); //KLux

  //Number 3
  //For the moment we don't use the VD6283_CCT_VS_ID */
  AllVirtualSensorsArray[VD6283_CCT_VS_ID].VirtualSensorId = VD6283_CCT_VS_ID;
  AllVirtualSensorsArray[VD6283_CCT_VS_ID].Enable=0;

  //Number 4
  AllVirtualSensorsArray[LSM6DSOX32_VS_ID].VirtualSensorId = LSM6DSOX32_VS_ID;
  AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Enable=0;
  AllVirtualSensorsArray[LSM6DSOX32_VS_ID].SensorType=VST_UI16;
  AllVirtualSensorsArray[LSM6DSOX32_VS_ID].ThsUsageType=TH_BIGGER;
  AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Th1.Ui16Value = 4096; //mg
  AllVirtualSensorsArray[LSM6DSOX32_VS_ID].MaxLimit.Ui16Value = 32000; //mg
  AllVirtualSensorsArray[LSM6DSOX32_VS_ID].MinLimit.Ui16Value =0; //mg

  //Number 5
  AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].VirtualSensorId = LSM6DSOX32_6D_VS_ID;
  AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].Enable=0;
  AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].SensorType=VST_UI8;
  AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].ThsUsageType=TH_EXT;
  AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].MaxLimit.Ui8Value = ORIENTATION_DOWN; //position
  AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].MinLimit.Ui8Value = ORIENTATION_UNDEF; //position

  //Number 6
  AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].VirtualSensorId = LSM6DSOX32_MLC_VS_ID;
  AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].Enable=0;
  AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].SensorType=VST_UI8;
  AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].ThsUsageType=TH_INT;
  AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].Th1.Ui8Value = 30;
  AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].Th2.Ui8Value = 50;
  AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].MaxLimit.Ui8Value = 90;
  AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].MinLimit.Ui8Value = 0;

  //Number 7
  AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].VirtualSensorId = LIS2DUXS12_MLC_VS_ID;
  AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].Enable=0;
  AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].SensorType=VST_UI8;
  AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].ThsUsageType=TH_EXT;
  AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].Th1.Ui8Value = AT_STATIONARY_UPRIGHT;
  AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].Th2.Ui8Value = AT_STATIONARY_UPRIGHT;
  AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].MaxLimit.Ui8Value = AT_SHAKEN;
  AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].MinLimit.Ui8Value = AT_STATIONARY_UPRIGHT;

  //Number 8
  AllVirtualSensorsArray[LIS2DUXS12_VS_ID].VirtualSensorId = LIS2DUXS12_VS_ID;
  AllVirtualSensorsArray[LIS2DUXS12_VS_ID].Enable=0;

  //Number 9
  AllVirtualSensorsArray[LIS2DUXS12_6D_VS_ID].VirtualSensorId = LIS2DUXS12_6D_VS_ID;
  AllVirtualSensorsArray[LIS2DUXS12_6D_VS_ID].Enable=0;

}

/**
* @brief  Set and Save the Default log configuration
* @param  None
* @retval None
*/
static void SaveDefaultConfiguration(void)
{
  uint32_t DataBuf32;
  uint8_t *DataBuf8 = (uint8_t *)&DataBuf32;
  uint16_t *DataBuf16 = (uint16_t *)&DataBuf32;

  SMARTAG2_PRINTF("SaveDefaultConfiguration\r\n");

  //Initialize the Protocol header
  SmarTag2CodeHeader.ProtVersion  = SMARTAG2_RECORD_VERSION;
  SmarTag2CodeHeader.ProtRevision = SMARTAG2_RECORD_REVISION;
  SmarTag2CodeHeader.BoardId      = SMARTAG2_BOARD_ID;
  SmarTag2CodeHeader.FirmwareId   = SMARTAG2_FIRMWARE_ID;

  DataBuf8[0] = SmarTag2CodeHeader.ProtVersion;
  DataBuf8[1] = SmarTag2CodeHeader.ProtRevision;
  DataBuf8[2] = SmarTag2CodeHeader.BoardId;
  DataBuf8[3] = SmarTag2CodeHeader.FirmwareId;

  /* Save Protocol/Board and Fw Id */
  LogDefinition.LastSamplePointer = SMARTAG2_START_ADDR_OFFSET;
  if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
    STNFC_Error_Handler(STNFC_WRITING_ERROR);
  }
  /* Update Last Sample Pointer (pointer to next Sample...) */
  LogDefinition.LastSamplePointer+=4;

  //Initialize the Log Header
  LogDefinition.ConfigSectionPositionPointer = LogDefinition.LastSamplePointer;
  LogDefinition.SampleTime=DATA_DEFAULT_SAMPLE_INT; // seconds
  LogDefinition.StartDateTime=SMARTAG2_DEFAULT_EPOCH_START_TIME; //No Date/Time Present
  LogDefinition.SingleShotDone = 0; /* Single Shot Not Used */
  LogDefinition.VirtualSensorsNum = 4;

  /* Write Virtual Sensor Number and Polling Sample Time */
  DataBuf8[0] = LogDefinition.SingleShotDone;
  DataBuf8[1] = LogDefinition.VirtualSensorsNum;
  DataBuf16[1] = LogDefinition.SampleTime;
  if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
    STNFC_Error_Handler(STNFC_WRITING_ERROR);
  }
  LogDefinition.LastSamplePointer+=4;

  /* Write TimeStamp==0 meaning Default Configuration */
  DataBuf32 = LogDefinition.StartDateTime;
  if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
    STNFC_Error_Handler(STNFC_WRITING_ERROR);
  }
  LogDefinition.LastSamplePointer+=4;

  //Initialize the Default virtual sensors
  //Number 0
  AllVirtualSensorsArray[STTS22H_VS_ID].Enable=1;
  ConfiguratedVirtualSensorsArray[0] = &AllVirtualSensorsArray[STTS22H_VS_ID];

  //Number 1
  AllVirtualSensorsArray[LPS22DF_VS_ID].Enable=1;
  ConfiguratedVirtualSensorsArray[1] = &AllVirtualSensorsArray[LPS22DF_VS_ID];

  //Number 2
  AllVirtualSensorsArray[VD6283_LUX_VS_ID].Enable=1;
  ConfiguratedVirtualSensorsArray[2] = &AllVirtualSensorsArray[VD6283_LUX_VS_ID];

  //Number 3
  AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].Enable=1;
  ConfiguratedVirtualSensorsArray[3] = &AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID];

  /* Write Virtual Sensors Configuration */
  SaveVirtualSensorsConfiguration();

  /* Reset the Max/Min For each Sensor */
  ResetMaxMinValuesAllVirtualSensors();

  /* Save the Max/Min for each Sensor */
  SaveMaxMinValuesForVirtualSensors();

  /* Write Sample Counter and Last Sample Pointer*/
  LogDefinition.SampleCounterAddress = LogDefinition.LastSamplePointer;
  LogDefinition.SampleCounter=0;
  UpdateLastSamplePointerAndSampleCounter(&LogDefinition);
  LogDefinition.LastSamplePointer+=8; /* We use 4bytes for each one */
}

/**
* @brief  Save the Last Sample Pointer and Sample Counter
* @param  SNFC_LogDefinition_t *LogDefinition Pointer to Log definition structure
* @retval None
*/
static void UpdateLastSamplePointerAndSampleCounter(SNFC_LogDefinition_t *LogDefinition)
{
  uint32_t DataBuf32;

  /* Write Sample Counter */
  DataBuf32 = LogDefinition->SampleCounter;
  if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition->SampleCounterAddress, 4)!=NFCTAG_OK){
    STNFC_Error_Handler(STNFC_WRITING_ERROR);
  }

  /*  Write LastSamplePointer */
  DataBuf32 = LogDefinition->LastSamplePointer;
  if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition->SampleCounterAddress+4, 4)!=NFCTAG_OK){
    STNFC_Error_Handler(STNFC_WRITING_ERROR);
  }
}

/**
* @brief  Reset the Max/Min Values
* @param  None
* @retval None
*/
static void ResetMaxMinValuesAllVirtualSensors(void)
{
  SMARTAG2_PRINTF("ResetMaxMinValuesAllVirtualSensors\r\n");

  if(AllVirtualSensorsArray[STTS22H_VS_ID].Enable) {
    AllVirtualSensorsArray[STTS22H_VS_ID].MinValue.Ui16Value = STTS22H_SAMPLE_TO_CODED(60); //'C
    AllVirtualSensorsArray[STTS22H_VS_ID].MaxValue.Ui16Value = STTS22H_SAMPLE_TO_CODED(-10); //'C
    AllVirtualSensorsArray[STTS22H_VS_ID].MaxDeltaDateTime = 0;
    AllVirtualSensorsArray[STTS22H_VS_ID].MinDeltaDateTime = 0;
  }

  if(AllVirtualSensorsArray[LPS22DF_VS_ID].Enable) {
    AllVirtualSensorsArray[LPS22DF_VS_ID].MinValue.Ui16Value =LPS22DF_SAMPLE_TO_CODED(1260); //hPa
    AllVirtualSensorsArray[LPS22DF_VS_ID].MaxValue.Ui16Value = LPS22DF_SAMPLE_TO_CODED(260); //hPa
    AllVirtualSensorsArray[LPS22DF_VS_ID].MaxDeltaDateTime = 0;
    AllVirtualSensorsArray[LPS22DF_VS_ID].MinDeltaDateTime = 0;
  }

  if(AllVirtualSensorsArray[VD6283_LUX_VS_ID].Enable) {
    AllVirtualSensorsArray[VD6283_LUX_VS_ID].MaxValue.Ui32Value = VD6283_LUX_SAMPLE_TO_CODED(0); //KLux
    AllVirtualSensorsArray[VD6283_LUX_VS_ID].MaxDeltaDateTime = 0;
    AllVirtualSensorsArray[VD6283_LUX_VS_ID].MinDeltaDateTime = 0;
  }

  if(AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Enable) {
    AllVirtualSensorsArray[LSM6DSOX32_VS_ID].MaxValue.Ui16Value = 0; //mg
    AllVirtualSensorsArray[LSM6DSOX32_VS_ID].MaxDeltaDateTime = 0;
    AllVirtualSensorsArray[LSM6DSOX32_VS_ID].MinDeltaDateTime = 0;
  }
  // For LSM6DSOX32_MLC_VS_ID, LSM6DSOX32_6D_VS_ID and LIS2DUXS12_MLC_VS_ID there are not Max/Min
}

/**
* @brief  Save the Max/Min Values
* @param  None
* @retval None
*/
static void SaveMaxMinValuesForVirtualSensors(void)
{
  int32_t SensorNum;
  uint32_t DataBuf32;

  SMARTAG2_PRINTF("SaveMaxMinValuesForVirtualSensors\r\n");

  for(SensorNum=0;SensorNum<LogDefinition.VirtualSensorsNum;SensorNum++) {

    if(ConfiguratedVirtualSensorsArray[SensorNum] == &AllVirtualSensorsArray[STTS22H_VS_ID]) {

      AllVirtualSensorsArray[STTS22H_VS_ID].MinPositionPointer = LogDefinition.LastSamplePointer;
      DataBuf32 = (0 /* ShortDeltaTime */) | (((uint32_t)AllVirtualSensorsArray[STTS22H_VS_ID].MinValue.Ui16Value)<<20);
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      AllVirtualSensorsArray[STTS22H_VS_ID].MaxPositionPointer = LogDefinition.LastSamplePointer;
      DataBuf32 = (0 /* ShortDeltaTime */) | (((uint32_t)AllVirtualSensorsArray[STTS22H_VS_ID].MaxValue.Ui16Value)<<20);
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

    } else if(ConfiguratedVirtualSensorsArray[SensorNum] == &AllVirtualSensorsArray[LPS22DF_VS_ID]) {

      AllVirtualSensorsArray[LPS22DF_VS_ID].MinPositionPointer = LogDefinition.LastSamplePointer;
      DataBuf32 = (0 /* ShortDeltaTime */) | (((uint32_t)AllVirtualSensorsArray[LPS22DF_VS_ID].MinValue.Ui16Value)<<20);
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      AllVirtualSensorsArray[LPS22DF_VS_ID].MaxPositionPointer = LogDefinition.LastSamplePointer;
      DataBuf32 = (0 /* ShortDeltaTime */) | (((uint32_t)AllVirtualSensorsArray[LPS22DF_VS_ID].MaxValue.Ui16Value)<<20);
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

    } else if(ConfiguratedVirtualSensorsArray[SensorNum] == &AllVirtualSensorsArray[VD6283_LUX_VS_ID]) {

      AllVirtualSensorsArray[VD6283_LUX_VS_ID].MaxPositionPointer = LogDefinition.LastSamplePointer;
      DataBuf32 = (0 /* ShortDeltaTime */);
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      DataBuf32 = AllVirtualSensorsArray[VD6283_LUX_VS_ID].MaxValue.Ui32Value;
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

    } else if(ConfiguratedVirtualSensorsArray[SensorNum] == &AllVirtualSensorsArray[LSM6DSOX32_VS_ID]) {

      AllVirtualSensorsArray[LSM6DSOX32_VS_ID].MaxPositionPointer = LogDefinition.LastSamplePointer;
      DataBuf32 = (0 /* ShortDeltaTime */) ;
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      DataBuf32 = AllVirtualSensorsArray[LSM6DSOX32_VS_ID].MaxValue.Ui16Value;
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

//    } else if(ConfiguratedVirtualSensorsArray[SensorNum] == &AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID]) {
//      //There are not Max/Min for 6D orientation
    }
  }
}

/**
* @brief  Save the Virtual Sensors Configurations
* @param  None
* @retval None
*/
static void SaveVirtualSensorsConfiguration(void)
{
  int32_t SensorNum;
  uint32_t DataBuf32;

  SMARTAG2_PRINTF("SaveVirtualSensorsConfiguration\r\n");

  for(SensorNum=0;SensorNum<LogDefinition.VirtualSensorsNum;SensorNum++) {

    if(ConfiguratedVirtualSensorsArray[SensorNum] == &AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID]) {

      DataBuf32 = LSM6DSOX32_MLC_VS_ID |
        (((uint32_t)AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].ThsUsageType)<<4) |
        (((uint32_t)AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].Th1.Ui8Value)<<(4+2)) |
        (((uint32_t)AllVirtualSensorsArray[LSM6DSOX32_MLC_VS_ID].Th2.Ui8Value)<<(4+2+8));
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      SMARTAG2_PRINTF("Save LSM6DSOX32_MLC_VS_ID:\r\n");
      SMARTAG2_PRINTF("\tThsUsageType=%s\r\n",ThresholdsUsageName[ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType]);
      SMARTAG2_PRINTF("\tTh1.Ui8Value=%d\r\n",ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui8Value);
       if(ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType<TH_LESS) {
        SMARTAG2_PRINTF("\tTh2.Ui8Value=%d\r\n",ConfiguratedVirtualSensorsArray[SensorNum]->Th2.Ui8Value);
      }

    } else if(ConfiguratedVirtualSensorsArray[SensorNum] == &AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID]) {

      DataBuf32 = LIS2DUXS12_MLC_VS_ID |
        (((uint32_t)AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].ThsUsageType)<<4) |
        (((uint32_t)AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].Th1.Ui8Value)<<(4+2)) |
        (((uint32_t)AllVirtualSensorsArray[LIS2DUXS12_MLC_VS_ID].Th2.Ui8Value)<<(4+2+8));
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      SMARTAG2_PRINTF("Save LIS2DUXS12_MLC_VS_ID:\r\n");
      SMARTAG2_PRINTF("\tThsUsageType=%s\r\n",ThresholdsUsageName[ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType]);
      SMARTAG2_PRINTF("\tTh1.Ui8Value=%d\r\n",ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui8Value);
       if(ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType<TH_LESS) {
        SMARTAG2_PRINTF("\tTh2.Ui8Value=%d\r\n",ConfiguratedVirtualSensorsArray[SensorNum]->Th2.Ui8Value);
      }

    } else if(ConfiguratedVirtualSensorsArray[SensorNum] == &AllVirtualSensorsArray[STTS22H_VS_ID]) {

      DataBuf32 = STTS22H_VS_ID |
        (((uint32_t)AllVirtualSensorsArray[STTS22H_VS_ID].ThsUsageType)<<4) |
          (((uint32_t)AllVirtualSensorsArray[STTS22H_VS_ID].Th1.Ui16Value)<<(4+2)) |
            (((uint32_t)AllVirtualSensorsArray[STTS22H_VS_ID].Th2.Ui16Value)<<(4+2+9));
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      SMARTAG2_PRINTF("Save STTS22H_VS_ID:\r\n");
      SMARTAG2_PRINTF("\tThsUsageType=%s\r\n",ThresholdsUsageName[ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType]);
      SMARTAG2_PRINTF("\tTh1.Ui16Value=%f\r\n",STTS22H_CODED_TO_SAMPLE(ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui16Value));
      if(ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType<TH_LESS) {
        SMARTAG2_PRINTF("\tTh2.Ui16Value=%f\r\n",STTS22H_CODED_TO_SAMPLE(ConfiguratedVirtualSensorsArray[SensorNum]->Th2.Ui16Value));
      }
    } else if(ConfiguratedVirtualSensorsArray[SensorNum] == &AllVirtualSensorsArray[LPS22DF_VS_ID]) {

      DataBuf32 = LPS22DF_VS_ID |
        (((uint32_t)AllVirtualSensorsArray[LPS22DF_VS_ID].ThsUsageType)<<4) |
          (((uint32_t)AllVirtualSensorsArray[LPS22DF_VS_ID].Th1.Ui16Value)<<(4+2)) |
            (((uint32_t)AllVirtualSensorsArray[LPS22DF_VS_ID].Th2.Ui16Value)<<(4+2+11));
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      SMARTAG2_PRINTF("Save LPS22DF_VS_ID:\r\n");
      SMARTAG2_PRINTF("\tThsUsageType=%s\r\n",ThresholdsUsageName[ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType]);
      SMARTAG2_PRINTF("\tTh1.Ui16Value=%f\r\n",LPS22DF_CODED_TO_SAMPLE(ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui16Value));
      if(ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType<TH_LESS) {
        SMARTAG2_PRINTF("\tTh2.Ui16Value=%f\r\n",LPS22DF_CODED_TO_SAMPLE(ConfiguratedVirtualSensorsArray[SensorNum]->Th2.Ui16Value));
      }

    } else if(ConfiguratedVirtualSensorsArray[SensorNum] == &AllVirtualSensorsArray[VD6283_LUX_VS_ID]) {

      DataBuf32 = VD6283_LUX_VS_ID |
        (((uint32_t)AllVirtualSensorsArray[VD6283_LUX_VS_ID].ThsUsageType)<<4) |
          (((uint32_t)AllVirtualSensorsArray[VD6283_LUX_VS_ID].Th1.Ui32Value)<<(4+2));
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      SMARTAG2_PRINTF("Save VD6283_LUX_VS_ID:\r\n");
      SMARTAG2_PRINTF("\tThsUsageType=%s\r\n",ThresholdsUsageName[ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType]);
      SMARTAG2_PRINTF("\tTh1.Ui32Value=%f\r\n",VD6283_LUX_CODED_TO_SAMPLE(ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui32Value));

    } else if(ConfiguratedVirtualSensorsArray[SensorNum] == &AllVirtualSensorsArray[LSM6DSOX32_VS_ID]) {

      DataBuf32 = LSM6DSOX32_VS_ID |
        (((uint32_t)AllVirtualSensorsArray[LSM6DSOX32_VS_ID].ThsUsageType)<<4) |
          (((uint32_t)AllVirtualSensorsArray[LSM6DSOX32_VS_ID].Th1.Ui16Value)<<(4+2));
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      SMARTAG2_PRINTF("Save LSM6DSOX32_VS_ID:\r\n");
      SMARTAG2_PRINTF("\tThsUsageType=%s\r\n",ThresholdsUsageName[ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType]);
      SMARTAG2_PRINTF("\tTh1.Ui16Value=%d\r\n",ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui16Value);

    } else if(ConfiguratedVirtualSensorsArray[SensorNum] == &AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID]) {

      DataBuf32 = LSM6DSOX32_6D_VS_ID |
        (((uint32_t)AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].ThsUsageType)<<4) |
          (((uint32_t)AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].Th1.Ui8Value)<<(4+2)) |
            (((uint32_t)AllVirtualSensorsArray[LSM6DSOX32_6D_VS_ID].Th2.Ui8Value)<<(4+2+8));
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      SMARTAG2_PRINTF("Save LSM6DSOX32_6D_VS_ID:\r\n");
      SMARTAG2_PRINTF("\tThsUsageType=%s\r\n",ThresholdsUsageName[ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType]);
      SMARTAG2_PRINTF("\tTh1.Ui8Value=%d\r\n",ConfiguratedVirtualSensorsArray[SensorNum]->Th1.Ui8Value);
      if(ConfiguratedVirtualSensorsArray[SensorNum]->ThsUsageType<TH_LESS) {
        SMARTAG2_PRINTF("\tTh2.Ui8Value=%d\r\n",ConfiguratedVirtualSensorsArray[SensorNum]->Th2.Ui8Value);
      }
    }
  }
}

/**
* @brief  Evaluates accelerometer normalization vector approximation in mg value
*         (as it is done in LIS3DSH)
* @param  BSP_MOTION_SENSOR_Axes_t Value_XYZ_mg
* @retval int32_t Normalization value
*/
static int32_t AccNormVectorApproxEvaluator(BSP_MOTION_SENSOR_Axes_t Value_XYZ_mg)
{
#define DATA_ACC_MIN     0
#define DATA_ACC_MAX 32000

  int32_t Ax,Ay,Az,SumABS,MaxABS;
  int32_t AccNormVector;

  Ax =  Value_XYZ_mg.x;
  if (Ax<0) {
    Ax=-Ax;
  }
  MaxABS=Ax;

  Ay =  Value_XYZ_mg.y;
  if (Ay<0) {
    Ay=-Ay;
  }
  if(Ay>MaxABS) {
    MaxABS=Ay;
  }

  Az =  Value_XYZ_mg.z;
  if (Az<0) {
    Az=-Az;
  }
  if(Az>MaxABS) {
    MaxABS=Az;
  }

  SumABS = Ax+Ay+Az;

  /* Vector norm approximation in mg (error +/-7.5%) */
  AccNormVector = (45*SumABS + 77*MaxABS) >> 7;

  if (AccNormVector<(DATA_ACC_MIN)) {
    AccNormVector = DATA_ACC_MIN;
  }

  if (AccNormVector>(DATA_ACC_MAX)) {
    AccNormVector = DATA_ACC_MAX;
  }

  return AccNormVector;
}

/**
* @brief  Reads the mens sensor data values
* @param  None
* @retval None
*/
static void MEMS_Sensors_ReadData(void)
{
  uint32_t DataBuf32;

  BSP_ENV_SENSOR_PowerOn();
  SMARTAG2_PRINTF("\tPowered on ambient sensors (VDD AMB On)\r\n");

  /* Wait Raise time */
  HAL_Delay(400);

  /* Initialize Environmental sensors */
  InitSmarTagEnvSensors();

  SMARTAG2_PRINTF("\tRead Sensor Data\r\n");

  if (AllVirtualSensorsArray[LPS22DF_VS_ID].Enable) {
    float Value;
    uint16_t ValueToCheck;
    uint8_t Status;
    do {
      HAL_Delay(2);
      BSP_ENV_SENSOR_Get_DRDY_Status(LPS22DF_0,ENV_PRESSURE,&Status);
    }while(Status!=1);
    BSP_ENV_SENSOR_GetValue(LPS22DF_0, ENV_PRESSURE, &Value);
    SMARTAG2_PRINTF("\t\tLPS22DF:\tPress= %f\r\n",Value);
    ValueToCheck = LPS22DF_SAMPLE_TO_CODED(Value);
    /* Check the Value respect Min and Max Limit Values*/
    MCR_STNFC_CompareWithLimits(Ui16,AllVirtualSensorsArray[LPS22DF_VS_ID],ValueToCheck);
    /* Compare with Ths and Update the Max/Min Sample Value */
    STNFC_ComputeMaxMinCompareTHsUi16t(&AllVirtualSensorsArray[LPS22DF_VS_ID],&LogDefinition,&hrtc);

#ifdef SMARTAG2_DONT_SAVE_EQUAL_SAMPLES
    if(AllVirtualSensorsArray[LPS22DF_VS_ID].SampleDeltaDateTime!=0) {
      static uint16_t LastSendValue=0xDEAD;
      if(AllVirtualSensorsArray[LPS22DF_VS_ID].Sample.Ui16Value==LastSendValue) {
        SMARTAG2_PRINTF("\tSkip this LPS22DF Value\r\n");
        AllVirtualSensorsArray[LPS22DF_VS_ID].SampleDeltaDateTime=0;
      } else {
        LastSendValue = AllVirtualSensorsArray[LPS22DF_VS_ID].Sample.Ui16Value;
      }
    }
#endif /* SMARTAG2_DONT_SAVE_EQUAL_SAMPLES */

    if(AllVirtualSensorsArray[LPS22DF_VS_ID].SampleDeltaDateTime!=0) {
#ifdef SMARTAG2_VERBOSE_PRINTF
      SMARTAG2_PRINTF("\tSave LPS22DF=%d\r\n", AllVirtualSensorsArray[LPS22DF_VS_ID].Sample.Ui16Value);
#else /* SMARTAG2_VERBOSE_PRINTF */
      SMARTAG2_PRINTF("\tSave LPS22DF\r\n");
#endif /* SMARTAG2_VERBOSE_PRINTF */

      DataBuf32 = LPS22DF_VS_ID |
        (((uint32_t)AllVirtualSensorsArray[LPS22DF_VS_ID].SampleDeltaDateTime)<<4);
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      DataBuf32 = AllVirtualSensorsArray[LPS22DF_VS_ID].Sample.Ui16Value;
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      /* Increment the new Sample Counter until the end of the Tag */
      NfcType5_UpdateSampleCounter(&LogDefinition,8);

      /* Update Sample Counter and Last Sample Pointer */
      UpdateLastSamplePointerAndSampleCounter(&LogDefinition);

      AllVirtualSensorsArray[LPS22DF_VS_ID].SampleDeltaDateTime=0;
    }

    if(AllVirtualSensorsArray[LPS22DF_VS_ID].MinDeltaDateTime!=0)  {
#ifdef SMARTAG2_VERBOSE_PRINTF
      SMARTAG2_PRINTF("\tSave Min Value for LPS22DF=%f\r\n", LPS22DF_CODED_TO_SAMPLE(AllVirtualSensorsArray[LPS22DF_VS_ID].MinValue.Ui16Value));
#else /* SMARTAG2_VERBOSE_PRINTF */
       SMARTAG2_PRINTF("\tSave Min Value for LPS22DF\r\n");
#endif /* SMARTAG2_VERBOSE_PRINTF */

      DataBuf32 = STNFC_ToShortDeltaDateTime(AllVirtualSensorsArray[LPS22DF_VS_ID].MinDeltaDateTime) |
        (((uint32_t)AllVirtualSensorsArray[LPS22DF_VS_ID].MinValue.Ui16Value)<<20);
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32,
                              AllVirtualSensorsArray[LPS22DF_VS_ID].MinPositionPointer, 4)!=NFCTAG_OK){
                                STNFC_Error_Handler(STNFC_WRITING_ERROR);
                              }
      AllVirtualSensorsArray[LPS22DF_VS_ID].MinDeltaDateTime=0;
    }

    if(AllVirtualSensorsArray[LPS22DF_VS_ID].MaxDeltaDateTime!=0)  {
#ifdef SMARTAG2_VERBOSE_PRINTF
      SMARTAG2_PRINTF("\tSave Max Value for LPS22DF=%f\r\n", LPS22DF_CODED_TO_SAMPLE(AllVirtualSensorsArray[LPS22DF_VS_ID].MaxValue.Ui16Value));
#else /* SMARTAG2_VERBOSE_PRINTF */
      SMARTAG2_PRINTF("\tSave Max Value for LPS22DF\r\n");
#endif /* SMARTAG2_VERBOSE_PRINTF */

      DataBuf32 = STNFC_ToShortDeltaDateTime(AllVirtualSensorsArray[LPS22DF_VS_ID].MaxDeltaDateTime) |
        (((uint32_t)AllVirtualSensorsArray[LPS22DF_VS_ID].MaxValue.Ui16Value)<<20);
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32,
                              AllVirtualSensorsArray[LPS22DF_VS_ID].MaxPositionPointer, 4)!=NFCTAG_OK){
                                STNFC_Error_Handler(STNFC_WRITING_ERROR);
                              }
      AllVirtualSensorsArray[LPS22DF_VS_ID].MaxDeltaDateTime=0;
    }

  }

  if (AllVirtualSensorsArray[STTS22H_VS_ID].Enable) {
    float Value;
    uint16_t ValueToCheck;
    uint8_t Status;
    do {
      HAL_Delay(10);
      BSP_ENV_SENSOR_Get_One_Shot_Status(STTS22H_0,&Status);
    }while(Status!=1);

    BSP_ENV_SENSOR_GetValue(STTS22H_0, ENV_TEMPERATURE, &Value);
    SMARTAG2_PRINTF("\t\tSTTS22H:\tTemp= %f\r\n", Value);
    ValueToCheck = STTS22H_SAMPLE_TO_CODED(Value);
    /* Check the Value respect Min and Max Limit Values*/
    MCR_STNFC_CompareWithLimits(Ui16,AllVirtualSensorsArray[STTS22H_VS_ID],ValueToCheck);
    /* Compare with Ths and Update the Max/Min Sample Value */
    STNFC_ComputeMaxMinCompareTHsUi16t(&AllVirtualSensorsArray[STTS22H_VS_ID],&LogDefinition,&hrtc);

#ifdef SMARTAG2_DONT_SAVE_EQUAL_SAMPLES
    if(AllVirtualSensorsArray[STTS22H_VS_ID].SampleDeltaDateTime!=0) {
      static uint16_t LastSendValue=0xDEAD;
      if(AllVirtualSensorsArray[STTS22H_VS_ID].Sample.Ui16Value==LastSendValue) {
        SMARTAG2_PRINTF("\tSkip this STS22H Value\r\n");
        AllVirtualSensorsArray[STTS22H_VS_ID].SampleDeltaDateTime=0;
      } else {
        LastSendValue = AllVirtualSensorsArray[STTS22H_VS_ID].Sample.Ui16Value;
      }
    }
#endif /* SMARTAG2_DONT_SAVE_EQUAL_SAMPLES */

    if(AllVirtualSensorsArray[STTS22H_VS_ID].SampleDeltaDateTime!=0) {
#ifdef SMARTAG2_VERBOSE_PRINTF
      SMARTAG2_PRINTF("\tSave STS22H=%d\r\n", AllVirtualSensorsArray[STTS22H_VS_ID].Sample.Ui16Value);
#else /* SMARTAG2_VERBOSE_PRINTF */
      SMARTAG2_PRINTF("\tSave STS22H\r\n");
#endif /* SMARTAG2_VERBOSE_PRINTF */

      DataBuf32 = STTS22H_VS_ID |
        (((uint32_t)AllVirtualSensorsArray[STTS22H_VS_ID].SampleDeltaDateTime)<<4);

      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      DataBuf32 = AllVirtualSensorsArray[STTS22H_VS_ID].Sample.Ui16Value;
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      /* Increment the new Sample Counter until the end of the Tag */
      NfcType5_UpdateSampleCounter(&LogDefinition,8);

      /* Update Sample Counter and Last Sample Pointer */
      UpdateLastSamplePointerAndSampleCounter(&LogDefinition);

      AllVirtualSensorsArray[STTS22H_VS_ID].SampleDeltaDateTime=0;
    }

    if(AllVirtualSensorsArray[STTS22H_VS_ID].MinDeltaDateTime!=0)  {
#ifdef SMARTAG2_VERBOSE_PRINTF
      SMARTAG2_PRINTF("\tSave Min Value for STS22H=%f\r\n", STTS22H_CODED_TO_SAMPLE(AllVirtualSensorsArray[STTS22H_VS_ID].MinValue.Ui16Value));
#else /* SMARTAG2_VERBOSE_PRINTF */
      SMARTAG2_PRINTF("\tSave Min Value for STS22H\r\n");
#endif /* SMARTAG2_VERBOSE_PRINTF */

      DataBuf32 = STNFC_ToShortDeltaDateTime(AllVirtualSensorsArray[STTS22H_VS_ID].MinDeltaDateTime) |
        (((uint32_t)AllVirtualSensorsArray[STTS22H_VS_ID].MinValue.Ui16Value)<<20);
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32,
                              AllVirtualSensorsArray[STTS22H_VS_ID].MinPositionPointer, 4)!=NFCTAG_OK){
                                STNFC_Error_Handler(STNFC_WRITING_ERROR);
                              }
      AllVirtualSensorsArray[STTS22H_VS_ID].MinDeltaDateTime=0;
    }

    if(AllVirtualSensorsArray[STTS22H_VS_ID].MaxDeltaDateTime!=0)  {
#ifdef SMARTAG2_VERBOSE_PRINTF
      SMARTAG2_PRINTF("\tSave Max Value for STS22H=%f\r\n", STTS22H_CODED_TO_SAMPLE(AllVirtualSensorsArray[STTS22H_VS_ID].MaxValue.Ui16Value));
#else /* SMARTAG2_VERBOSE_PRINTF */
      SMARTAG2_PRINTF("\tSave Max Value for STS22H\r\n");
#endif /* SMARTAG2_VERBOSE_PRINTF */

      DataBuf32 = STNFC_ToShortDeltaDateTime(AllVirtualSensorsArray[STTS22H_VS_ID].MaxDeltaDateTime) |
        (((uint32_t)AllVirtualSensorsArray[STTS22H_VS_ID].MaxValue.Ui16Value)<<20);
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32,
                              AllVirtualSensorsArray[STTS22H_VS_ID].MaxPositionPointer, 4)!=NFCTAG_OK){
                                STNFC_Error_Handler(STNFC_WRITING_ERROR);
                              }
      AllVirtualSensorsArray[STTS22H_VS_ID].MaxDeltaDateTime=0;
    }

  }

  if((AllVirtualSensorsArray[VD6283_CCT_VS_ID].Enable) | (AllVirtualSensorsArray[VD6283_LUX_VS_ID].Enable)) {
    uint32_t AlsResults[LIGHT_SENSOR_MAX_CHANNELS] = {0};
    ResultCCT_t CCT_Result;
    uint32_t current_exposure;
    int32_t status;
    BSP_LIGHT_SENSOR_Start(0, LIGHT_SENSOR_MODE_SINGLESHOT);
    do {
      status = BSP_LIGHT_SENSOR_GetValues(0, AlsResults);
    } while (status != BSP_ERROR_NONE);

    BSP_LIGHT_SENSOR_Stop(0);
    BSP_LIGHT_SENSOR_GetExposureTime(0, &current_exposure);

#ifdef SMARTAG2_VERBOSE_PRINTF
    {
      static const char *AlsChannelStr[] = {
        [LIGHT_SENSOR_RED_CHANNEL] = "RED",
        [LIGHT_SENSOR_VISIBLE_CHANNEL] = "VISIBLE",
        [LIGHT_SENSOR_BLUE_CHANNEL] = "BLUE",
        [LIGHT_SENSOR_GREEN_CHANNEL] = "GREEN",
        [LIGHT_SENSOR_IR_CHANNEL] = "IR",
        [LIGHT_SENSOR_CLEAR_CHANNEL] = "CLEAR"
      };
      uint8_t channel;

      for (channel = 0; channel < LIGHT_SENSOR_MAX_CHANNELS; channel++)
      {
        SMARTAG2_PRINTF("\tChannel %u - %8s", channel + 1, AlsChannelStr[channel]);
        SMARTAG2_PRINTF("\tValue: %lu\r\n", (unsigned long)AlsResults[channel]);
      }
    }
#endif /* SMARTAG2_VERBOSE_PRINTF */

    ComputeCCT(current_exposure, AlsResults, &CCT_Result);
    if(AllVirtualSensorsArray[VD6283_LUX_VS_ID].Enable) {
      uint32_t ValueToCheck;
      SMARTAG2_PRINTF("\t\tVD6283:\t\tKLux= %f\r\n", CCT_Result.Y);
      ValueToCheck = VD6283_LUX_SAMPLE_TO_CODED(CCT_Result.Y);
      /* Check the Value respect Min and Max Limit Values*/
      MCR_STNFC_CompareWithLimits(Ui32,AllVirtualSensorsArray[VD6283_LUX_VS_ID],ValueToCheck);
      /* Compare with Ths and Update the Max/Min Sample Value */
      STNFC_ComputeMaxMinCompareTHsUi32t(&AllVirtualSensorsArray[VD6283_LUX_VS_ID],&LogDefinition,&hrtc);

#ifdef SMARTAG2_DONT_SAVE_EQUAL_SAMPLES
    if(AllVirtualSensorsArray[VD6283_LUX_VS_ID].SampleDeltaDateTime!=0) {
      static uint32_t LastSendValue=0xDEADBEEF;
      if(AllVirtualSensorsArray[VD6283_LUX_VS_ID].Sample.Ui32Value==LastSendValue) {
        SMARTAG2_PRINTF("\tSkip this VD6283 Lux Value\r\n");
        AllVirtualSensorsArray[VD6283_LUX_VS_ID].SampleDeltaDateTime=0;
      } else {
        LastSendValue = AllVirtualSensorsArray[VD6283_LUX_VS_ID].Sample.Ui32Value;
      }
    }
#endif /* SMARTAG2_DONT_SAVE_EQUAL_SAMPLES */

      if(AllVirtualSensorsArray[VD6283_LUX_VS_ID].SampleDeltaDateTime!=0) {
#ifdef SMARTAG2_VERBOSE_PRINTF
        SMARTAG2_PRINTF("\tSave VD6283 KLux=%f\r\n", VD6283_LUX_CODED_TO_SAMPLE(AllVirtualSensorsArray[VD6283_LUX_VS_ID].Sample.Ui32Value));
#else /* SMARTAG2_VERBOSE_PRINTF */
        SMARTAG2_PRINTF("\tSave VD6283 KLux\r\n");
#endif /* SMARTAG2_VERBOSE_PRINTF */

        DataBuf32 = VD6283_LUX_VS_ID |
          (((uint32_t)AllVirtualSensorsArray[VD6283_LUX_VS_ID].SampleDeltaDateTime)<<4);
        if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
          STNFC_Error_Handler(STNFC_WRITING_ERROR);
        }
        LogDefinition.LastSamplePointer+=4;

        DataBuf32 = AllVirtualSensorsArray[VD6283_LUX_VS_ID].Sample.Ui32Value;
        if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
          STNFC_Error_Handler(STNFC_WRITING_ERROR);
        }
        LogDefinition.LastSamplePointer+=4;

        /* Increment the new Sample Counter until the end of the Tag */
        NfcType5_UpdateSampleCounter(&LogDefinition,8);

        /* Update Sample Counter and Last Sample Pointer */
        UpdateLastSamplePointerAndSampleCounter(&LogDefinition);

        AllVirtualSensorsArray[VD6283_LUX_VS_ID].SampleDeltaDateTime=0;
      }

      if(AllVirtualSensorsArray[VD6283_LUX_VS_ID].MaxDeltaDateTime!=0)  {
#ifdef SMARTAG2_VERBOSE_PRINTF
        SMARTAG2_PRINTF("\tSave Max Value for VD6283=%d\r\n", AllVirtualSensorsArray[VD6283_LUX_VS_ID].MaxValue.Ui32Value);
#else  /* SMARTAG2_VERBOSE_PRINTF */
        SMARTAG2_PRINTF("\tSave Max Value for VD6283\r\n");
#endif /* SMARTAG2_VERBOSE_PRINTF */

        DataBuf32 = STNFC_ToShortDeltaDateTime(AllVirtualSensorsArray[VD6283_LUX_VS_ID].MaxDeltaDateTime);
        if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32,
                                AllVirtualSensorsArray[VD6283_LUX_VS_ID].MaxPositionPointer, 4)!=NFCTAG_OK){
                                  STNFC_Error_Handler(STNFC_WRITING_ERROR);
                                }

        DataBuf32 = AllVirtualSensorsArray[VD6283_LUX_VS_ID].MaxValue.Ui32Value;
        if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32,
                                AllVirtualSensorsArray[VD6283_LUX_VS_ID].MaxPositionPointer+4, 4)!=NFCTAG_OK){
                                  STNFC_Error_Handler(STNFC_WRITING_ERROR);
                                }

        AllVirtualSensorsArray[VD6283_LUX_VS_ID].MaxDeltaDateTime=0;
      }

    }

    if(AllVirtualSensorsArray[VD6283_CCT_VS_ID].Enable) {
      SMARTAG2_PRINTF("\t\tVD6283:\t\tCCT=%f\r\n",CCT_Result.cct);
    }

    /* Sensor DeInit */
    BSP_LIGHT_SENSOR_DeInit(0);
  }

   if((AllVirtualSensorsArray[VD6283_LUX_VS_ID].Enable) |
      (AllVirtualSensorsArray[VD6283_CCT_VS_ID].Enable) |
      (AllVirtualSensorsArray[STTS22H_VS_ID].Enable) |
      (AllVirtualSensorsArray[LPS22DF_VS_ID].Enable)) {
     BSP_ENV_SENSOR_PowerOff();
     SMARTAG2_PRINTF("\r\nPowered off ambient sensors (VDD AMB Off)\r\n");
   }
}

/*
* @brief compute cct value from RGB channels values
*/
static void ComputeCCT(uint32_t TimeExposure, uint32_t *AlsResults,ResultCCT_t *Result)
{
  /* correlation matrix used in order to convert RBG values to XYZ space */

  /*
  * (X)   (G)   (Cx1 Cx2 Cx3)
  * (Y) = (B) * (Cy1 Cy2 Cy3)
  * (Z)   (R)   (Cz1 Cz2 Cz3)
  *
  * X = G * Cx1 + B * Cx2 + R * Cx3
  * Y = G * Cy1 + B * Cy2 + R * Cy3
  * Z = G * Cz1 + B * Cz2 + R * Cz3
  *
  * */

  static const double_t Cx[] = {0.416700, -0.143816, 0.205570};
  static const double_t Cy[] = {0.506372, -0.120614, -0.028752};
  static const double_t Cz[] = {0.335866, 0.494781, -0.552625};

  uint8_t i;

  double_t data[3U];
  double_t X_tmp = 0, Y_tmp = 0, Z_tmp = 0;
  double_t xyNormFactor;
  double_t m_xNormCoeff;
  double_t m_yNormCoeff;
  double_t nCoeff;
  double_t expo_scale = 100800.0 / TimeExposure;

  /* normalize and prepare RGB channels values for cct computation */
  data[0] = (double_t)AlsResults[LIGHT_SENSOR_GREEN_CHANNEL] / 256.0;
  data[1] = (double_t)AlsResults[LIGHT_SENSOR_BLUE_CHANNEL] / 256.0;
  data[2] = (double_t)AlsResults[LIGHT_SENSOR_RED_CHANNEL] / 256.0;

  /* apply correlation matrix to RGB channels to obtain (X,Y,Z) */
  for (i = 0; i < 3U; i++)
  {
    X_tmp += Cx[i] * data[i];
    Y_tmp += Cy[i] * data[i];
    Z_tmp += Cz[i] * data[i];
  }

  /* transform (X,Y,Z) to (x,y) */
  xyNormFactor = X_tmp + Y_tmp + Z_tmp;
  m_xNormCoeff = X_tmp / xyNormFactor;
  m_yNormCoeff = Y_tmp / xyNormFactor;

  /* rescale X, Y, Z according to expo. Reference is G1x and 100.8ms */
  Result->X = expo_scale * X_tmp;
  Result->Y = expo_scale * Y_tmp;
  Result->Z = expo_scale * Z_tmp;

  /* apply McCamy's formula to obtain CCT value (expressed in °K) */
  nCoeff = (m_xNormCoeff - 0.3320) / (0.1858 - m_yNormCoeff);
  Result->cct = (449 * pow(nCoeff, 3) + 3525 * pow(nCoeff, 2) + 6823.3 * nCoeff + 5520.33);
}

/**
* @brief  Init SmarTag Environmental Sensors
* @param  None
* @retval None
*/
static void InitSmarTagEnvSensors(void)
{
  /* lps22df sensor: Init */
  if(AllVirtualSensorsArray[LPS22DF_VS_ID].Enable) {
    BSP_ENV_SENSOR_Init(LPS22DF_0, ENV_PRESSURE);
    /* lps22df sensor: Set output data rate for pressure */
    BSP_ENV_SENSOR_SetOutputDataRate(LPS22DF_0, ENV_PRESSURE, 75.0f);
  }

  /* stts22h sensor: Init */
  if(AllVirtualSensorsArray[STTS22H_VS_ID].Enable) {
    BSP_ENV_SENSOR_Init(STTS22H_0, ENV_TEMPERATURE);
    /* stts22h sensor: Set One Shot Mode */
    BSP_ENV_SENSOR_Set_One_Shot(STTS22H_0);
  }

  /* vd6283 sensor: Init */
  if((AllVirtualSensorsArray[VD6283_CCT_VS_ID].Enable) | (AllVirtualSensorsArray[VD6283_LUX_VS_ID].Enable)) {
#ifdef SMARTAG2_ENABLE_DEBUG
    uint32_t current_exposure;
    uint32_t current_gain;
#endif /* SMARTAG2_ENABLE_DEBUG */
    uint8_t channel;
    BSP_LIGHT_SENSOR_Init(0);
    BSP_LIGHT_SENSOR_SetExposureTime(0, 100000); /* microseconds */
#ifdef SMARTAG2_ENABLE_DEBUG
    BSP_LIGHT_SENSOR_GetExposureTime(0, &current_exposure);
    SMARTAG2_PRINTF("vd6283 Component (Exposure set to %lu us)\r\n", (unsigned long)current_exposure);
#endif /* SMARTAG2_ENABLE_DEBUG */

    /* initialize gains */
    for (channel = 0; channel < LIGHT_SENSOR_MAX_CHANNELS; channel++) {
      BSP_LIGHT_SENSOR_SetGain(0, channel, 256);
#ifdef SMARTAG2_ENABLE_DEBUG
      BSP_LIGHT_SENSOR_GetGain(0, channel, &current_gain);
      SMARTAG2_PRINTF("Channel %d gain set to %f\r\n", channel + 1,(1.0f*current_gain)/256);
#endif /* SMARTAG2_ENABLE_DEBUG */
    }
  }

}

/**
* @brief  Detector of the vector norm approximation in mg value for accelerometer event
* @param  None
* @retval return new Max Acc Value
*/
static int32_t DetectorValueForAccEvent(void)
{
  int i;
  BSP_MOTION_SENSOR_Axes_t AccValue_XYZ_mg;
  int32_t AccNorm;
  int32_t AccEventVmax=0;

  uint16_t NumSamples;
  /* Reads the FIFO status */
  BSP_MOTION_SENSOR_FIFO_Get_Num_Samples(LSM6DSO32X_0, &NumSamples);

  /* Read all the FIFO samples (they should be 32 at this point) */
  for(i=0; i<NumSamples; i++) {
    BSP_MOTION_SENSOR_GetAxes(LSM6DSO32X_0, MOTION_ACCELERO, &AccValue_XYZ_mg);

    AccNorm = AccNormVectorApproxEvaluator(AccValue_XYZ_mg);

    if(AccNorm > AccEventVmax) {
      AccEventVmax = AccNorm;
    }
  }
  return AccEventVmax;
}

/**
* @brief  Detects actual accelerometer 6D orientation
* @param  None
* @retval uint8_t Orientation
*/
static uint8_t Understand6DOrientation(void)
{
  uint8_t xl,yl,zl,xh,yh,zh;
  uint8_t SmarTagPosition;

  /* Read the Orientation registers */
  BSP_MOTION_SENSOR_Get_6D_Orientation_XL( LSM6DSO32X_0, &xl );
  BSP_MOTION_SENSOR_Get_6D_Orientation_YL( LSM6DSO32X_0, &yl );
  BSP_MOTION_SENSOR_Get_6D_Orientation_ZL( LSM6DSO32X_0, &zl );
  BSP_MOTION_SENSOR_Get_6D_Orientation_XH( LSM6DSO32X_0, &xh );
  BSP_MOTION_SENSOR_Get_6D_Orientation_YH( LSM6DSO32X_0, &yh );
  BSP_MOTION_SENSOR_Get_6D_Orientation_ZH( LSM6DSO32X_0, &zh );

  /* Understand the Orientation */
  if ( xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0 ) {
    SmarTagPosition= ORIENTATION_UP;
  } else if ( xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0 ) {
    SmarTagPosition= ORIENTATION_RIGHT;
  } else if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0 ) {
    SmarTagPosition= ORIENTATION_LEFT;
  } else if ( xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0 ) {
    SmarTagPosition= ORIENTATION_DOWN;
  } else if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1 ) {
    SmarTagPosition= ORIENTATION_TOP;
  } else if ( xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0 ) {
    SmarTagPosition= ORIENTATION_BOTTOM;
  } else {
    SmarTagPosition= ORIENTATION_UNDEF;
  }

  return SmarTagPosition;
}

/**
  * @brief  BSP ACC_INT callback
  * @param  None
  * @retval None.
  */
static void BSP_ACC_INT_Callback(void)
{
  MemsInterrupt = 1;
}
