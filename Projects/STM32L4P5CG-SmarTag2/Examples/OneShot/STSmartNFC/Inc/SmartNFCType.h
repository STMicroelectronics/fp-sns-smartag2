/**
  ******************************************************************************
  * @file    SmartNFCType.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.1.0
  * @date    22-February-2023
  * @brief   Common Types used
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMART_NFC_TYPE_H
#define __SMART_NFC_TYPE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "SMARTAG2_config.h"
#include <time.h>
   
/* Exported Types ------------------------------------------------------------*/

/**
  * @brief Header Structure for identifying the code running on the board
  */
typedef struct
{
  uint8_t ProtVersion; /* Protocol Version */
  uint8_t ProtRevision; /* Protocol Revision */
  uint8_t BoardId;     /* Board Id */
  uint8_t FirmwareId;  /* FirmareId: 0x01->254, 0x01 ... 0xFE, 0xFF->Custom */
} SNFC_CodeDefinition_t;

/**
  * @brief Header Structure for specifying the Log Section
  */
typedef struct
{
  uint32_t StartDateTime;     /* Starting Date and Time (Compact format) */
  
  time_t StartTimeStamp;
  
  uint32_t SampleCounter;     /* Number of Sample Saved on TAG */
  uint32_t LastSamplePointer; /* Pointer to the Next Sample position */ 
  
  uint32_t SampleCounterAddress; /* Tag Address where is saved the LastSamplePointer */
  
  /* Config section */
  uint32_t ConfigSectionPositionPointer; /* pointer to this section */  
  uint16_t SampleTime;       /* Sample Time in Seconds 1seconds -> 3600seconds */
  uint8_t SingleShotDone;    /* Single Shot Done */
  uint8_t VirtualSensorsNum; /* Number Virtual Sensors To Use */
} SNFC_LogDefinition_t;

/**
  * @brief  Virtual Sensor Thresholds Usage Type
  */
typedef enum
{
  STNFC_READING_ERROR = 0,
  STNFC_WRITING_ERROR,
  STNFC_CONFIG_ERROR,
  STNFC_INIT_ERROR,
  STNFC_RUNTIME_ERROR
} SNFC_ErrorCode_t;


/**
  * @brief Error code type
  */
typedef enum
{
  TH_EXT = 0, /* External Range: -->TH1 TH2<-- */
  TH_INT,     /* Internal Range: TH1<---->TH2 */
  TH_LESS,    /* Less than     : -->TH1 */
  TH_BIGGER   /* Bigger than   : TH1<-- */
} SNFC_ThresholdsUsage_t;



/**
  * @brief Virtual Sensor Enumerative Supported Type
  */
typedef enum
{
  VST_UI8 = 0,
  VST_I8,
  VST_UI16,
  VST_I16,
  VST_UI32,
  VST_I32,
  VST_F,
  VST_D
} SNFC_VirtualSensorEnum_t;

/**
  * @brief Virtual Sensor Union Supported Type
  */
typedef union
{
  uint8_t Ui8Value;
  uint16_t Ui16Value;
  uint32_t Ui32Value;
}SNFC_VirtualSensorValue_t;

/**
  * @brief Thresholds definition for a Virtual Sensor
  */
typedef struct
{
  /* Configuration Section */
  uint8_t VirtualSensorId; /* Virtual Sensor Id */
  uint8_t Enable;          /* Enable flag */
  
  SNFC_VirtualSensorEnum_t SensorType; /* Sensor/Threshods type */
  
  /* Thresholds Section */
  SNFC_ThresholdsUsage_t ThsUsageType; /* How Use the Threshold */
  SNFC_VirtualSensorValue_t Th1; /* Lower Threshold */
  SNFC_VirtualSensorValue_t Th2; /* Higher Threshold */
  
  /* Min Section */
  uint32_t MinPositionPointer; 
  time_t MinDeltaDateTime;
  SNFC_VirtualSensorValue_t MinValue;
  
  /* Max Section */
  uint32_t MaxPositionPointer;
  time_t MaxDeltaDateTime;
  SNFC_VirtualSensorValue_t MaxValue;
  
  /* Measurable Limit Section */
  SNFC_VirtualSensorValue_t MaxLimit;
  SNFC_VirtualSensorValue_t MinLimit;
  
  /* Sample Section */
  time_t SampleDeltaDateTime;
  SNFC_VirtualSensorValue_t Sample;

} SNFC_VirtualSensor_t;


#ifdef __cplusplus
}
#endif

#endif /* __SMART_NFC_TYPE_H */

