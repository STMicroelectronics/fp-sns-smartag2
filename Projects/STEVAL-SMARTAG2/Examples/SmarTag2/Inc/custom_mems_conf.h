/**
  ******************************************************************************
  * @file    custom_mems_conf.h
  * @author  MEMS Application Team
  * @brief   This file contains definitions of the MEMS components bus interfaces for custom boards
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
#ifndef __CUSTOM_MEMS_CONF_H__
#define __CUSTOM_MEMS_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "steval_smartag2_bus.h"
#include "steval_smartag2_errno.h"

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#define USE_CUSTOM_ENV_SENSOR_STTS22H_0           0U

#define USE_CUSTOM_MOTION_SENSOR_H3LIS331DL_0     0U

#define USE_CUSTOM_ENV_SENSOR_LPS22DF_0           0U

#define USE_CUSTOM_MOTION_SENSOR_LSM6DSO32X_0     0U

#define USE_CUSTOM_MOTION_SENSOR_LIS2DUXS12_0     0U

#define CUSTOM_STTS22H_0_I2C_Init BSP_I2C1_Init
#define CUSTOM_STTS22H_0_I2C_DeInit BSP_I2C1_DeInit
#define CUSTOM_STTS22H_0_I2C_ReadReg BSP_I2C1_ReadReg
#define CUSTOM_STTS22H_0_I2C_WriteReg BSP_I2C1_WriteReg

#define CUSTOM_H3LIS331DL_0_SPI_Init BSP_SPI1_Init
#define CUSTOM_H3LIS331DL_0_SPI_DeInit BSP_SPI1_DeInit
#define CUSTOM_H3LIS331DL_0_SPI_Send BSP_SPI1_Send
#define CUSTOM_H3LIS331DL_0_SPI_Recv BSP_SPI1_Recv

#define CUSTOM_H3LIS331DL_0_CS_PORT GPIOB
#define CUSTOM_H3LIS331DL_0_CS_PIN GPIO_PIN_1

#define CUSTOM_LPS22DF_0_I2C_Init BSP_I2C1_Init
#define CUSTOM_LPS22DF_0_I2C_DeInit BSP_I2C1_DeInit
#define CUSTOM_LPS22DF_0_I2C_ReadReg BSP_I2C1_ReadReg
#define CUSTOM_LPS22DF_0_I2C_WriteReg BSP_I2C1_WriteReg

#define CUSTOM_LSM6DSO32X_0_SPI_Init BSP_SPI1_Init
#define CUSTOM_LSM6DSO32X_0_SPI_DeInit BSP_SPI1_DeInit
#define CUSTOM_LSM6DSO32X_0_SPI_Send BSP_SPI1_Send
#define CUSTOM_LSM6DSO32X_0_SPI_Recv BSP_SPI1_Recv

#define CUSTOM_LSM6DSO32X_0_CS_PORT GPIOH
#define CUSTOM_LSM6DSO32X_0_CS_PIN GPIO_PIN_0

#define CUSTOM_LIS2DUXS12_0_SPI_Init BSP_SPI1_Init
#define CUSTOM_LIS2DUXS12_0_SPI_DeInit BSP_SPI1_DeInit
#define CUSTOM_LIS2DUXS12_0_SPI_Send BSP_SPI1_Send
#define CUSTOM_LIS2DUXS12_0_SPI_Recv BSP_SPI1_Recv

#define CUSTOM_LIS2DUXS12_0_CS_PORT GPIOA
#define CUSTOM_LIS2DUXS12_0_CS_PIN GPIO_PIN_15

#ifdef __cplusplus
}
#endif

#endif /* __CUSTOM_MEMS_CONF_H__*/
