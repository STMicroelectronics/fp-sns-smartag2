/**
  ******************************************************************************
  * @file    Smartag2_light_sensor.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.0.0
  * @date    31-August-2022
  * @brief   This file contains the common defines and functions prototypes
  *          for the BSP light sensor interface for SmarTag2 board
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMARTAG2_LIGHT_SENSOR_H__
#define __SMARTAG2_LIGHT_SENSOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "SmarTag2_conf.h"
#include "light_sensor.h"

#include "vd6283tx.h"

/** @addtogroup BSP
  * @{
  */

/** @defgroup BSP_LIGHT_SENSOR LIGHT_SENSOR
  * @{
  */

/** @defgroup BSP_LIGHT_SENSOR_Exported_Types Exported Types
  * @{
  */
typedef struct
{
  uint8_t NumberOfChannels;  /*!< Max: LIGHT_SENSOR_MAX_CHANNELS */
  uint8_t FlickerDetection;  /*!< Not available: 0, Available: 1 */
  uint8_t Autogain;          /*!< Not available: 0, Available: 1 */
  uint8_t Flicker;           /*!< Not available: 0, Available: 1 */
} LIGHT_SENSOR_Capabilities_t;
/**
  * @}
  */

/** @defgroup BSP_LIGHT_SENSOR_Exported_Constants Exported Constants
  * @{
  */
#define LIGHT_SENSOR_VD6283TX_ADDRESS   (0x40U)
#define LIGHT_SENSOR_MAX_CHANNELS       VD6283TX_MAX_CHANNELS
#define LIGHT_SENSOR_ALL_CHANNELS       VD6283TX_ALL_CHANNELS

#define LIGHT_SENSOR_MODE_SINGLESHOT    VD6283TX_MODE_SINGLESHOT
#define LIGHT_SENSOR_MODE_CONTINUOUS    VD6283TX_MODE_CONTINUOUS
#define LIGHT_SENSOR_FLICKER_ANALOG     VD6283TX_FLICKER_ANALOG
#define LIGHT_SENSOR_CTRL_DARK          VD6283TX_CTRL_DARK

#define LIGHT_SENSOR_RED_CHANNEL        VD6283TX_RED_CHANNEL
#define LIGHT_SENSOR_VISIBLE_CHANNEL    VD6283TX_VISIBLE_CHANNEL
#define LIGHT_SENSOR_BLUE_CHANNEL       VD6283TX_BLUE_CHANNEL
#define LIGHT_SENSOR_GREEN_CHANNEL      VD6283TX_GREEN_CHANNEL
#define LIGHT_SENSOR_IR_CHANNEL         VD6283TX_IR_CHANNEL
#define LIGHT_SENSOR_CLEAR_CHANNEL      VD6283TX_CLEAR_CHANNEL
/**
  * @}
 */

/** @defgroup BSP_LIGHT_SENSOR_Exported_Variables Exported Variables
  * @{
  */
extern void *BSP_LIGHT_SENSOR_CompObj[LIGHT_SENSOR_INSTANCES_NBR];
/**
  * @}
  */

/** @defgroup BSP_LIGHT_SENSOR_Exported_Functions Exported Functions
  * @{
  */
int32_t BSP_LIGHT_SENSOR_Init(uint32_t Instance);
int32_t BSP_LIGHT_SENSOR_DeInit(uint32_t Instance);
int32_t BSP_LIGHT_SENSOR_ReadID(uint32_t Instance, uint32_t *pId);
int32_t BSP_LIGHT_SENSOR_GetCapabilities(uint32_t Instance, LIGHT_SENSOR_Capabilities_t *pCapabilities);
int32_t BSP_LIGHT_SENSOR_SetExposureTime(uint32_t Instance, uint32_t ExposureTime);
int32_t BSP_LIGHT_SENSOR_GetExposureTime(uint32_t Instance, uint32_t *pExposureTime);
int32_t BSP_LIGHT_SENSOR_SetGain(uint32_t Instance, uint8_t Channel, uint32_t Gain);
int32_t BSP_LIGHT_SENSOR_GetGain(uint32_t Instance, uint8_t Channel, uint32_t *pGain);
int32_t BSP_LIGHT_SENSOR_SetInterMeasurementTime(uint32_t Instance, uint32_t InterMeasurementTime);
int32_t BSP_LIGHT_SENSOR_GetInterMeasurementTime(uint32_t Instance, uint32_t *pInterMeasurementTime);
int32_t BSP_LIGHT_SENSOR_Start(uint32_t Instance, uint8_t Mode);
int32_t BSP_LIGHT_SENSOR_Stop(uint32_t Instance);
int32_t BSP_LIGHT_SENSOR_StartFlicker(uint32_t Instance, uint8_t Channel, uint8_t OutputMode);
int32_t BSP_LIGHT_SENSOR_StopFlicker(uint32_t Instance);
int32_t BSP_LIGHT_SENSOR_GetValues(uint32_t Instance, uint32_t *pResult);
int32_t BSP_LIGHT_SENSOR_SetControlMode(uint32_t Instance, uint32_t ControlMode, uint32_t Value);

int32_t BSP_LIGHT_SENSOR_GetSaturation(uint32_t Instance, uint32_t *pValue);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __SMARTAG2_LIGHT_SENSOR_H__ */

