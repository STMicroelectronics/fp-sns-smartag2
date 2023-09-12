/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    AppOneShot.c
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.2.0
  * @date    30-May-2023
  * @brief   Application Process
  ******************************************************************************
  * @attention
  *
  * Copyright 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE END Header */

#include "AppOneShot.h"
#include "SMARTAG2_config.h"
#include "main.h"

#include <string.h>
#include <math.h>

#include "SmarTag2_env_sensors.h"
#include "SmarTag2_env_sensors_ex.h"
#include "SmarTag2_nfctag.h"
#include "SmarTag2_nfctag_ex.h"
#include "Smartag2_light_sensor.h"
#include "st25dvxxkc.h"
#include "SmartNFC.h"
#include "TagType5.h"

/* Private Defines -----------------------------------------------------------*/

#define STTS22H_SAMPLE_TO_CODED(Value)        ((uint16_t)(((Value)*5) - ((-10)*5)))
#define LPS22DF_SAMPLE_TO_CODED(Value)       ((uint16_t)(((Value)*2) - ((260)*2)))
#define VD6283_LUX_SAMPLE_TO_CODED(Value)    ((uint32_t)((Value)*1000))

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

/* Private variables ---------------------------------------------------------*/

//ST Smart NFC Protocol
SNFC_CodeDefinition_t SmarTag2CodeHeader;
SNFC_LogDefinition_t LogDefinition;
SNFC_VirtualSensor_t AllVirtualSensorsArray[SMARTAG2_VIRTUAL_SENSORS_NUM];
SNFC_VirtualSensor_t *ConfiguratedVirtualSensorsArray[SMARTAG2_VIRTUAL_SENSORS_NUM];

/* Private function prototypes -----------------------------------------------*/
static void InitSmarTagEnvSensors(void);
static void SetNFCBehavior(void);
static void ComputeCCT(uint32_t TimeExposure, uint32_t *AlsResults,ResultCCT_t *Result);

//Manage the Log configuration
static void SetAllAvailableVirtualSensors(void);
static void SaveDefaultConfiguration(void);
static void SetMaxMinValuePositionsForVirtualSensors(void);
static void SaveVirtualSensorsConfiguration(void);
static void UpdateLastSamplePointerAndSampleCounter(SNFC_LogDefinition_t *LogDefinition);
static void SaveConfigSection(void);

/**
* @brief  SmarTag Application Start
* @param  None
* @retval None
*/
void OneShotAppStart(void)
{
  BSP_NFCTAG_EEP_PowerOn();
  BSP_ENV_SENSOR_PowerOn();

  /* Rise time required by VDD_EEPROM for NFC */
  HAL_Delay(200);

  /* Init I2C interface */
  {
    int32_t Initialized =0;
    while(!Initialized) {
      if(BSP_NFCTAG_Init(BSP_NFCTAG_INSTANCE)!=NFCTAG_OK ) {
        SMARTAG2_PRINTF("Error NFCTAG Initialized\r\n");
        STNFC_Error_Handler(STNFC_CONFIG_ERROR);
      } else {
        SMARTAG2_PRINTF("NFCTAG Initialized\r\n");
        Initialized=1;
      }
      HAL_Delay(10);
    }
  }

  ST25_RETRY(BSP_NFCTAG_SetRFSleep_Dyn(BSP_NFCTAG_INSTANCE));
  SMARTAG2_PRINTF("Sleep RF\r\n");

  /* Set the NFC behavior */
  SetNFCBehavior();

 // BSP_LPD_Off();

  InitSTSmartNFC();

  SetAllAvailableVirtualSensors();

  /* Set the Default Configuration */
  SaveDefaultConfiguration();

  NfcType5_SetInitialNDEFPayLoadLengthValue(LogDefinition.LastSamplePointer);

  NfcType5_ComputeNDEFPayLoadSize(LogDefinition.LastSamplePointer,LogDefinition.SampleCounter);

}

/**
* @brief  Set the NFC Security Level, GPO behavior and Energy harvesting mode
* @param  None
* @retval None
*/
static void SetNFCBehavior(void)
{

  ST25DVxxKC_EH_MODE_STATUS_E EHMode = ST25DVXXKC_EH_ACTIVE_AFTER_BOOT;

  if(BSP_NFCTAG_CheckEHMODE(&EHMode)!=NFCTAG_OK ) {
    STNFC_Error_Handler(STNFC_CONFIG_ERROR);
  } else {
    if(EHMode!=ST25DVXXKC_EH_ACTIVE_AFTER_BOOT) {
      /* Setting the New Password for I2C protection */
      if(BSP_NFCTAG_ChangeI2CPassword(0x90ABCDEF,0x12345678)!=NFCTAG_OK ) {
        STNFC_Error_Handler(STNFC_CONFIG_ERROR);
      }

      /* Setting the Energy harvesting mode */
      if(BSP_NFCTAG_ChangeEHMODE(0x90ABCDEF,0x12345678,ST25DVXXKC_EH_ACTIVE_AFTER_BOOT)!=NFCTAG_OK ) {
        STNFC_Error_Handler(STNFC_CONFIG_ERROR);
      }
      SMARTAG2_PRINTF("Changed Harvesting mode\r\n");
    }
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
  LogDefinition.SingleShotDone = 0; /* Single Shot not done */
  LogDefinition.VirtualSensorsNum = 3;

  /* Write Single Shot Done, Virtual Sensor Number and Polling Sample Time */
  SaveConfigSection();
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

  /* Write Virtual Sensors Configuration */
  SaveVirtualSensorsConfiguration();

  /* Set the Max/Min Value positions for each Sensor */
  SetMaxMinValuePositionsForVirtualSensors();

  /* Write Sample Counter and Last Sample Pointer*/
  LogDefinition.SampleCounterAddress = LogDefinition.LastSamplePointer;
  LogDefinition.SampleCounter=0;
  UpdateLastSamplePointerAndSampleCounter(&LogDefinition);
  LogDefinition.LastSamplePointer+=8; /* We use 4bytes for each one */
}

/**
* @brief  Write Single Shot Done, Virtual Sensor Number and Polling Sample Time
* @param  None
* @retval None
*/
static void SaveConfigSection(void)
{
  uint32_t DataBuf32;
  uint8_t *DataBuf8 = (uint8_t *)&DataBuf32;
  uint16_t *DataBuf16 = (uint16_t *)&DataBuf32;

  DataBuf8[0] = LogDefinition.SingleShotDone;
  DataBuf8[1] = LogDefinition.VirtualSensorsNum;
  DataBuf16[1] = LogDefinition.SampleTime;
  if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.ConfigSectionPositionPointer, 4)!=NFCTAG_OK){
    STNFC_Error_Handler(STNFC_WRITING_ERROR);
  }
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
* @brief  Set the Max Min Values positions for all the Virtual Sensors
* @param  None
* @retval None
*/
static void SetMaxMinValuePositionsForVirtualSensors(void)
{
  int32_t SensorNum;

  //SMARTAG2_PRINTF("SetMaxMinValuePositionsForVirtualSensors\r\n");

  for(SensorNum=0;SensorNum<LogDefinition.VirtualSensorsNum;SensorNum++) {

    if(ConfiguratedVirtualSensorsArray[SensorNum] == &AllVirtualSensorsArray[STTS22H_VS_ID]) {

      AllVirtualSensorsArray[STTS22H_VS_ID].MinPositionPointer = LogDefinition.LastSamplePointer;
      LogDefinition.LastSamplePointer+=4;

      AllVirtualSensorsArray[STTS22H_VS_ID].MaxPositionPointer = LogDefinition.LastSamplePointer;
      LogDefinition.LastSamplePointer+=4;

    } else if(ConfiguratedVirtualSensorsArray[SensorNum] == &AllVirtualSensorsArray[LPS22DF_VS_ID]) {

      AllVirtualSensorsArray[LPS22DF_VS_ID].MinPositionPointer = LogDefinition.LastSamplePointer;
      LogDefinition.LastSamplePointer+=4;

      AllVirtualSensorsArray[LPS22DF_VS_ID].MaxPositionPointer = LogDefinition.LastSamplePointer;
      LogDefinition.LastSamplePointer+=4;

    } else if(ConfiguratedVirtualSensorsArray[SensorNum] == &AllVirtualSensorsArray[VD6283_LUX_VS_ID]) {

      AllVirtualSensorsArray[VD6283_LUX_VS_ID].MaxPositionPointer = LogDefinition.LastSamplePointer;
      //We use 8 bytes for this section
      LogDefinition.LastSamplePointer+=4;
      LogDefinition.LastSamplePointer+=4;
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

    if(ConfiguratedVirtualSensorsArray[SensorNum] == &AllVirtualSensorsArray[STTS22H_VS_ID]) {

      DataBuf32 = STTS22H_VS_ID |
        (((uint32_t)AllVirtualSensorsArray[STTS22H_VS_ID].ThsUsageType)<<3) |
          (((uint32_t)AllVirtualSensorsArray[STTS22H_VS_ID].Th1.Ui16Value)<<(3+2)) |
            (((uint32_t)AllVirtualSensorsArray[STTS22H_VS_ID].Th2.Ui16Value)<<(3+2+9));
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

    } else if(ConfiguratedVirtualSensorsArray[SensorNum] == &AllVirtualSensorsArray[LPS22DF_VS_ID]) {

      DataBuf32 = LPS22DF_VS_ID |
        (((uint32_t)AllVirtualSensorsArray[LPS22DF_VS_ID].ThsUsageType)<<3) |
          (((uint32_t)AllVirtualSensorsArray[LPS22DF_VS_ID].Th1.Ui16Value)<<(3+2)) |
            (((uint32_t)AllVirtualSensorsArray[LPS22DF_VS_ID].Th2.Ui16Value)<<(3+2+11));
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

    } else if(ConfiguratedVirtualSensorsArray[SensorNum] == &AllVirtualSensorsArray[VD6283_LUX_VS_ID]) {

      DataBuf32 = VD6283_LUX_VS_ID |
        (((uint32_t)AllVirtualSensorsArray[VD6283_LUX_VS_ID].ThsUsageType)<<3) |
          (((uint32_t)AllVirtualSensorsArray[VD6283_LUX_VS_ID].Th1.Ui32Value)<<(3+2));
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;
    }
  }
}

/**
* @brief  Reads the mens sensor data values
* @param  None
* @retval None
*/
void MEMS_Sensors_ReadData(void)
{
  uint32_t DataBuf32;

  /* Initialize Environmental sensors */
  InitSmarTagEnvSensors();

  //HAL_Delay(1000);

  if (AllVirtualSensorsArray[LPS22DF_VS_ID].Enable) {
    float Value;
    uint16_t ValueToCheck;
    uint8_t Status;
    do {
      HAL_Delay(2);
      BSP_ENV_SENSOR_Get_DRDY_Status(LPS22DF_0,ENV_PRESSURE,&Status);
    }while(Status!=1);
    BSP_ENV_SENSOR_GetValue(LPS22DF_0, ENV_PRESSURE, &Value);
    SMARTAG2_PRINTF("LPS22DF: Press= %f\r\n",Value);
    ValueToCheck = LPS22DF_SAMPLE_TO_CODED(Value);
    /* Check the Value respect Min and Max Limit Values*/
    MCR_STNFC_CompareWithLimits(Ui16,AllVirtualSensorsArray[LPS22DF_VS_ID],ValueToCheck);
    /* Compare with Ths and Update the Max/Min Sample Value */
    STNFC_ComputeMaxMinCompareTHsUi16t(&AllVirtualSensorsArray[LPS22DF_VS_ID],&LogDefinition);

    if(AllVirtualSensorsArray[LPS22DF_VS_ID].SampleDeltaDateTime!=0) {

      DataBuf32 = LPS22DF_VS_ID |
        (((uint32_t)AllVirtualSensorsArray[LPS22DF_VS_ID].SampleDeltaDateTime)<<3);
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      DataBuf32 = AllVirtualSensorsArray[LPS22DF_VS_ID].Sample.Ui16Value;
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      /* Update Sample Counter and Last Sample Pointer */
      UpdateLastSamplePointerAndSampleCounter(&LogDefinition);

      /* Increment the new Sample Counter until the end of the Tag */
      NfcType5_UpdateSampleCounter(&LogDefinition,8);

      AllVirtualSensorsArray[LPS22DF_VS_ID].SampleDeltaDateTime=0;
    }

    if(AllVirtualSensorsArray[LPS22DF_VS_ID].MinDeltaDateTime!=0)  {
      DataBuf32 = 1 /* Fake */|
        (((uint32_t)AllVirtualSensorsArray[LPS22DF_VS_ID].MinValue.Ui16Value)<<20);
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32,
                              AllVirtualSensorsArray[LPS22DF_VS_ID].MinPositionPointer, 4)!=NFCTAG_OK){
                                STNFC_Error_Handler(STNFC_WRITING_ERROR);
                              }
      AllVirtualSensorsArray[LPS22DF_VS_ID].MinDeltaDateTime=0;
    }

    if(AllVirtualSensorsArray[LPS22DF_VS_ID].MaxDeltaDateTime!=0)  {
      DataBuf32 = 1 /* Fake */|
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
    SMARTAG2_PRINTF("STTS22H: Temp= %f\r\n", Value);
    ValueToCheck = STTS22H_SAMPLE_TO_CODED(Value);
    /* Check the Value respect Min and Max Limit Values*/
    MCR_STNFC_CompareWithLimits(Ui16,AllVirtualSensorsArray[STTS22H_VS_ID],ValueToCheck);
    /* Compare with Ths and Update the Max/Min Sample Value */
    STNFC_ComputeMaxMinCompareTHsUi16t(&AllVirtualSensorsArray[STTS22H_VS_ID],&LogDefinition);

    if(AllVirtualSensorsArray[STTS22H_VS_ID].SampleDeltaDateTime!=0) {
      DataBuf32 = STTS22H_VS_ID |
        (((uint32_t)AllVirtualSensorsArray[STTS22H_VS_ID].SampleDeltaDateTime)<<3);

      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      DataBuf32 = AllVirtualSensorsArray[STTS22H_VS_ID].Sample.Ui16Value;
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
        STNFC_Error_Handler(STNFC_WRITING_ERROR);
      }
      LogDefinition.LastSamplePointer+=4;

      /* Update Sample Counter and Last Sample Pointer */
      UpdateLastSamplePointerAndSampleCounter(&LogDefinition);

      /* Increment the new Sample Counter until the end of the Tag */
      NfcType5_UpdateSampleCounter(&LogDefinition,8);

      AllVirtualSensorsArray[STTS22H_VS_ID].SampleDeltaDateTime=0;
    }

    if(AllVirtualSensorsArray[STTS22H_VS_ID].MinDeltaDateTime!=0)  {

      DataBuf32 = 1 /* Fake */|
        (((uint32_t)AllVirtualSensorsArray[STTS22H_VS_ID].MinValue.Ui16Value)<<20);
      if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32,
                              AllVirtualSensorsArray[STTS22H_VS_ID].MinPositionPointer, 4)!=NFCTAG_OK){
                                STNFC_Error_Handler(STNFC_WRITING_ERROR);
                              }
      AllVirtualSensorsArray[STTS22H_VS_ID].MinDeltaDateTime=0;
    }

    if(AllVirtualSensorsArray[STTS22H_VS_ID].MaxDeltaDateTime!=0)  {

      DataBuf32 = 1 /* Fake */|
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

    ComputeCCT(current_exposure, AlsResults, &CCT_Result);
    if(AllVirtualSensorsArray[VD6283_LUX_VS_ID].Enable) {
      uint32_t ValueToCheck;
      SMARTAG2_PRINTF("VD6283: KLux= %f\r\n", CCT_Result.Y);
      ValueToCheck = VD6283_LUX_SAMPLE_TO_CODED(CCT_Result.Y);
      /* Check the Value respect Min and Max Limit Values*/
      MCR_STNFC_CompareWithLimits(Ui32,AllVirtualSensorsArray[VD6283_LUX_VS_ID],ValueToCheck);
      /* Compare with Ths and Update the Max/Min Sample Value */
      STNFC_ComputeMaxMinCompareTHsUi32t(&AllVirtualSensorsArray[VD6283_LUX_VS_ID],&LogDefinition);

      if(AllVirtualSensorsArray[VD6283_LUX_VS_ID].SampleDeltaDateTime!=0) {

        DataBuf32 = VD6283_LUX_VS_ID |
          (((uint32_t)AllVirtualSensorsArray[VD6283_LUX_VS_ID].SampleDeltaDateTime)<<3);
        if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
          STNFC_Error_Handler(STNFC_WRITING_ERROR);
        }
        LogDefinition.LastSamplePointer+=4;

        DataBuf32 = AllVirtualSensorsArray[VD6283_LUX_VS_ID].Sample.Ui32Value;
        if(BSP_NFCTAG_WriteData( BSP_NFCTAG_INSTANCE, (uint8_t *)&DataBuf32, LogDefinition.LastSamplePointer, 4)!=NFCTAG_OK){
          STNFC_Error_Handler(STNFC_WRITING_ERROR);
        }
        LogDefinition.LastSamplePointer+=4;

        /* Update Sample Counter and Last Sample Pointer */
        UpdateLastSamplePointerAndSampleCounter(&LogDefinition);

        /* Increment the new Sample Counter until the end of the Tag */
        NfcType5_UpdateSampleCounter(&LogDefinition,8);

        AllVirtualSensorsArray[VD6283_LUX_VS_ID].SampleDeltaDateTime=0;
      }

      if(AllVirtualSensorsArray[VD6283_LUX_VS_ID].MaxDeltaDateTime!=0)  {

        DataBuf32 = 1;
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
      SMARTAG2_PRINTF("VD6283: CCT=%f\r\n",CCT_Result.cct);
    }

    /* Sensor DeInit */
    BSP_LIGHT_SENSOR_DeInit(0);
  }

  SMARTAG2_PRINTF("Write One Shot Done\r\n");
  LogDefinition.SingleShotDone = 1; /* Single Shot Done */
  SaveConfigSection();
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

