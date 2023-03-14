/**
  ******************************************************************************
  * @file    SmarTag2_conf_template.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.0.0
  * @date    31-August-2022
  * @brief   SmarTag2 configuration template file.
  *          This file should be copied to the application folder and renamed
  *          to SmarTag2_conf.h.
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
#ifndef __SMARTGA2_CONF_H__
#define __SMARTGA2_CONF_H__

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

#define USE_ENV_SENSOR_STTS22H_0        1U
#define USE_ENV_SENSOR_LPS22DF_0        1U

#define USE_MOTION_SENSOR_LIS2DUXS12_0    0U
#define USE_MOTION_SENSOR_H3LIS331DL_0  1U
#define USE_MOTION_SENSOR_LSM6DSO32X_0  1U
  
#define LIGHT_SENSOR_INSTANCES_NBR      1U

#define BSP_NFCTAG_INSTANCE		0U
#define BSP_NFCTAG_GPO_PRIORITY		0U
#define BSP_USART1_BOUNDRATE 9600


#ifdef __cplusplus
}
#endif

#endif /* __SMARTGA2_CONF_H__*/

