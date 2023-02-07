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

#define USE_ENV_SENSOR_HTS221_0         1U
#define USE_ENV_SENSOR_STTS22H_0        1U
#define USE_ENV_SENSOR_LPS22DF_0        1U

#define USE_MOTION_SENSOR_H3LIS331DL_0  1U
#define USE_MOTION_SENSOR_LSM6DSO32X_0  1U
  
#define LIGHT_SENSOR_INSTANCES_NBR      1U

#define BSP_NFCTAG_INSTANCE         (0)
#define BSP_NFCTAG_GPO_PRIORITY     (0)
  
#define BSP_USART1_BOUNDRATE 115200

/* hts221 Temperature and Humidity Sensor */
#define BSP_HTS221_0_I2C_Init           BSP_I2C1_Init
#define BSP_HTS221_0_I2C_DeInit         BSP_I2C1_DeInit
#define BSP_HTS221_0_I2C_ReadReg        BSP_I2C1_ReadReg
#define BSP_HTS221_0_I2C_WriteReg       BSP_I2C1_WriteReg

/* ODR = 1.0Hz */
#define HTS221_ODR                      HTS221_ODR_12Hz5

/* stts22hh Temperature Sensor */
#define BSP_STTS22H_0_I2C_Init          BSP_I2C1_Init
#define BSP_STTS22H_0_I2C_DeInit        BSP_I2C1_DeInit
#define BSP_STTS22H_0_I2C_ReadReg       BSP_I2C1_ReadReg
#define BSP_STTS22H_0_I2C_WriteReg      BSP_I2C1_WriteReg

 /* ODR = 1.0Hz */
#define STTS22H_ODR                     STTS22H_1Hz

/* lps22df Temperature and Pressure Sensor */
#define BSP_LPS22DF_0_I2C_Init          BSP_I2C1_Init
#define BSP_LPS22DF_0_I2C_DeInit        BSP_I2C1_DeInit
#define BSP_LPS22DF_0_I2C_ReadReg       BSP_I2C1_ReadReg
#define BSP_LPS22DF_0_I2C_WriteReg      BSP_I2C1_WriteReg

/* ODR = 75.0Hz */
#define LPS22DF_ODR     LPS22DF_75Hz 

/* h3lis331dl acc Sensor */
#define BSP_H3LIS331DL_0_SPI_Init       BSP_SPI1_Init
#define BSP_H3LIS331DL_0_SPI_DeInit     BSP_SPI1_DeInit
#define BSP_H3LIS331DL_0_SPI_Send       BSP_SPI1_Send
#define BSP_H3LIS331DL_0_SPI_Recv       BSP_SPI1_Recv

#define BSP_H3LIS331DL_0_CS_PORT        GPIOB
#define BSP_H3LIS331DL_0_CS_PIN         GPIO_PIN_1

#define HIGH_G_CS_GPIO_Port             BSP_H3LIS331DL_0_CS_PORT  
#define HIGH_G_CS_Pin                   BSP_H3LIS331DL_0_CS_PIN

#define H3LIS331DL_ACC_ODR              100.0f /* ODR = 100Hz */
#define H3LIS331DL_ACC_FS               100 /* FS = 100g */

/* lsm6dso32x acc and gyro Sensor */
#define BSP_LSM6DSO32X_0_SPI_Init       BSP_SPI1_Init
#define BSP_LSM6DSO32X_0_SPI_DeInit     BSP_SPI1_DeInit
#define BSP_LSM6DSO32X_0_SPI_Send       BSP_SPI1_Send
#define BSP_LSM6DSO32X_0_SPI_Recv       BSP_SPI1_Recv

#define BSP_LSM6DSO32X_0_CS_PORT        GPIOH
#define BSP_LSM6DSO32X_0_CS_PIN         GPIO_PIN_0

#define IMU_CS_GPIO_Port                BSP_LSM6DSO32X_0_CS_PORT
#define IMU_CS_Pin                      BSP_LSM6DSO32X_0_CS_PIN

/* ODR = 104Hz */
#define LSM6DSO32X_ACC_ODR              LSM6DSO32X_XL_ODR_104Hz
/* FS = 16g */
#define LSM6DSO32X_ACC_FS               LSM6DSO32X_16g
/* ODR = 104Hz */
#define LSM6DSO32X_GYRO_ODR             LSM6DSO32X_GY_ODR_104Hz
/* FS = 2000dps */
#define LSM6DSO32X_GYRO_FS              LSM6DSO32X_2000dps

/* st25dv Dual Interface EEPROM */
#define BSP_ST25DVxxKC_I2C_Init             BSP_I2C3_Init
#define BSP_ST25DVxxKC_I2C_DeInit           BSP_I2C3_DeInit
#define BSP_ST25DVxxKC_I2C_ReadReg16        BSP_I2C3_ReadReg16
#define BSP_ST25DVxxKC_I2C_WriteReg16       BSP_I2C3_WriteReg16
#define BSP_ST25DVxxKC_I2C_Recv             BSP_I2C3_Recv
#define BSP_ST25DVxxKC_I2C_IsReady          BSP_I2C3_IsReady
#define BSP_ST25DVxxKC_GetTick              HAL_GetTick
  
/* vd6283tx Light Sensor */
#define BSP_LIGHT_SENSOR_I2C_Init       BSP_I2C1_Init
#define BSP_LIGHT_SENSOR_I2C_DeInit     BSP_I2C1_DeInit
#define BSP_LIGHT_SENSOR_I2C_ReadReg    BSP_I2C1_ReadReg
#define BSP_LIGHT_SENSOR_I2C_WriteReg   BSP_I2C1_WriteReg
#define BSP_LIGHT_SENSOR_GetTick        BSP_GetTick

#ifdef __cplusplus
}
#endif

#endif /* __SMARTAG2_CONF_H__*/

