/**
  ******************************************************************************
  * @file    Smartag2_env_sensors.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.0.0
  * @date    31-August-2022
  * @brief   This file provides BSP Environmental Sensors interface for
  *          SmarTag2 board
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
#ifndef __ENV_SENSORS_H__
#define __ENV_SENSORS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "SmarTag2_conf.h"
#include "env_sensor.h"

#ifndef USE_ENV_SENSOR_STTS22H_0
#define USE_ENV_SENSOR_STTS22H_0         1
#endif
  
#ifndef USE_ENV_SENSOR_LPS22DF_0
#define USE_ENV_SENSOR_LPS22DF_0         0
#endif

#if (USE_ENV_SENSOR_STTS22H_0 == 1)
#include "stts22h.h"
#endif

#if (USE_ENV_SENSOR_LPS22DF_0 == 1)
#include "lps22df.h"
#endif

#if (USE_ENV_SENSOR_STTS22H_0 == 1)
#define STTS22H_0 (0)
#endif

#if (USE_ENV_SENSOR_LPS22DF_0 == 1)
#define LPS22DF_0 (USE_ENV_SENSOR_STTS22H_0)
#endif
  
/** @defgroup SMARTAG2_VDD_AMB_MCU_PIN SMARTAG2 VDD AMB MCU PIN
 * @{
 */
#define VDD_AMB_MCU_Pin         GPIO_PIN_14
#define VDD_AMB_MCU_GPIO_Port   GPIOB
/**
 * @}
 */

/* Environmental Sensor instance Info */
typedef struct
{
  uint8_t Temperature;
  uint8_t Pressure;
  uint8_t Humidity;
  uint8_t LowPower;
  float   HumMaxOdr;
  float   TempMaxOdr;
  float   PressMaxOdr;
} ENV_SENSOR_Capabilities_t;

typedef struct
{
  uint32_t Functions;
} ENV_SENSOR_Ctx_t;

#ifndef ENV_TEMPERATURE
#define ENV_TEMPERATURE      1U
#endif
#ifndef ENV_PRESSURE
#define ENV_PRESSURE         2U
#endif
#ifndef ENV_HUMIDITY
#define ENV_HUMIDITY         4U
#endif

#define ENV_FUNCTIONS_NBR    2U
#define ENV_INSTANCES_NBR    (USE_ENV_SENSOR_STTS22H_0 + USE_ENV_SENSOR_LPS22DF_0)

#if (ENV_INSTANCES_NBR == 0)
#error "No environmental sensor instance has been selected"
#endif

void BSP_ENV_SENSOR_PowerOff(void);
void BSP_ENV_SENSOR_PowerOn(void);

int32_t BSP_ENV_SENSOR_Init(uint32_t Instance, uint32_t Functions);
int32_t BSP_ENV_SENSOR_DeInit(uint32_t Instance);
int32_t BSP_ENV_SENSOR_GetCapabilities(uint32_t Instance, ENV_SENSOR_Capabilities_t *Capabilities);
int32_t BSP_ENV_SENSOR_ReadID(uint32_t Instance, uint8_t *Id);
int32_t BSP_ENV_SENSOR_Enable(uint32_t Instance, uint32_t Function);
int32_t BSP_ENV_SENSOR_Disable(uint32_t Instance, uint32_t Function);
int32_t BSP_ENV_SENSOR_GetOutputDataRate(uint32_t Instance, uint32_t Function, float *Odr);
int32_t BSP_ENV_SENSOR_SetOutputDataRate(uint32_t Instance, uint32_t Function, float Odr);
int32_t BSP_ENV_SENSOR_GetValue(uint32_t Instance, uint32_t Function, float *Value);

#ifdef __cplusplus
}
#endif

#endif /* __ENV_SENSORS_H__ */

