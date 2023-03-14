/**
  ******************************************************************************
  * @file    SmarTag2_conf.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.0.2
  * @date    30-January-2023
  * @brief   This file contains definitions of the MEMS components bus
  *          interfaces for SmarTag2 board
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
#ifndef __SMARTAG2_CONF_H__
#define __SMARTAG2_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "SmarTag2_bus.h"
#include "SmarTag2_errno.h"
#include "stm32l4xx_hal_exti.h"

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#define USE_ENV_SENSOR_STTS22H_0        0U
#define USE_ENV_SENSOR_LPS22DF_0        0U

#define USE_MOTION_SENSOR_LIS2DUXS12_0  0U
#define USE_MOTION_SENSOR_H3LIS331DL_0  0U
#define USE_MOTION_SENSOR_LSM6DSO32X_0  0U
  
#define LIGHT_SENSOR_INSTANCES_NBR      0U

#define BSP_NFCTAG_INSTANCE         (0)
#define BSP_NFCTAG_GPO_PRIORITY     (0)
  
#define BSP_USART1_BOUNDRATE 115200

#ifdef __cplusplus
}
#endif

#endif /* __SMARTAG2_CONF_H__*/

