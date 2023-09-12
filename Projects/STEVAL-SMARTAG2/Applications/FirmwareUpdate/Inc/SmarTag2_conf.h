/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    SmarTag2_conf.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.2.0
  * @date    30-May-2023
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

/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMARTAG2_CONF_H__
#define __SMARTAG2_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "steval_smartag2_bus.h"
#include "steval_smartag2_errno.h"
#include "custom_mems_conf.h"

/* USER CODE BEGIN 1 */

/* Select if H3LIS331DL motion sensor is used in the application */
#define USE_MOTION_SENSOR_H3LIS331DL_0      0
/* Select if LIS2DUXS12 motion sensor is used in the application */
#define USE_MOTION_SENSOR_LIS2DUXS12_0      0
/* Select if STTS22H environmental sensor is used in the application */
#define USE_ENV_SENSOR_STTS22H_0      0
/* Select if LPS22DF environmental sensor is used in the application */
#define USE_ENV_SENSOR_LPS22DF_0      0
/* Select if LSM6DSO32X motion sensor is used in the application */
#define USE_MOTION_SENSOR_LSM6DSO32X_0      0

#define BSP_USART1_BOUNDRATE 115200
/* USER CODE END 1 */

#define LIGHT_SENSOR_INSTANCES_NBR      1U

#define BSP_NFCTAG_INSTANCE         (0)
#define BSP_NFCTAG_GPO_PRIORITY     (0)

#if ( (USE_ENV_SENSOR_LPS22DF_0 + USE_ENV_SENSOR_STTS22H_0) == 0)
  #undef USE_ENV_SENSOR_LPS22DF_0
  #define USE_ENV_SENSOR_LPS22DF_0      1
#endif

#if ( (USE_MOTION_SENSOR_LIS2DUXS12_0 + USE_MOTION_SENSOR_H3LIS331DL_0 + USE_MOTION_SENSOR_LSM6DSO32X_0) == 0)
  #undef USE_MOTION_SENSOR_LIS2DUXS12_0
  #define USE_MOTION_SENSOR_LIS2DUXS12_0      1
#endif

/* stts22hh Temperature Sensor */
#define BSP_STTS22H_0_I2C_Init          CUSTOM_STTS22H_0_I2C_Init
#define BSP_STTS22H_0_I2C_DeInit        CUSTOM_STTS22H_0_I2C_DeInit
#define BSP_STTS22H_0_I2C_ReadReg       CUSTOM_STTS22H_0_I2C_ReadReg
#define BSP_STTS22H_0_I2C_WriteReg      CUSTOM_STTS22H_0_I2C_WriteReg

/* lps22df Temperature and Pressure Sensor */
#define BSP_LPS22DF_0_I2C_Init          CUSTOM_LPS22DF_0_I2C_Init
#define BSP_LPS22DF_0_I2C_DeInit        CUSTOM_LPS22DF_0_I2C_DeInit
#define BSP_LPS22DF_0_I2C_ReadReg       CUSTOM_LPS22DF_0_I2C_ReadReg
#define BSP_LPS22DF_0_I2C_WriteReg      CUSTOM_LPS22DF_0_I2C_WriteReg

/* h3lis331dl acc Sensor */
#define BSP_H3LIS331DL_0_SPI_Init       CUSTOM_H3LIS331DL_0_SPI_Init
#define BSP_H3LIS331DL_0_SPI_DeInit     CUSTOM_H3LIS331DL_0_SPI_DeInit
#define BSP_H3LIS331DL_0_SPI_Send       CUSTOM_H3LIS331DL_0_SPI_Send
#define BSP_H3LIS331DL_0_SPI_Recv       CUSTOM_H3LIS331DL_0_SPI_Recv

#define BSP_H3LIS331DL_0_CS_PORT        CUSTOM_H3LIS331DL_0_CS_PORT
#define BSP_H3LIS331DL_0_CS_PIN         CUSTOM_H3LIS331DL_0_CS_PIN

/* LIS2DUXS12 acc Sensor */
#define BSP_LIS2DUXS12_0_SPI_Init       CUSTOM_LIS2DUXS12_0_SPI_Init
#define BSP_LIS2DUXS12_0_SPI_DeInit     CUSTOM_LIS2DUXS12_0_SPI_DeInit
#define BSP_LIS2DUXS12_0_SPI_Send       CUSTOM_LIS2DUXS12_0_SPI_Send
#define BSP_LIS2DUXS12_0_SPI_Recv       CUSTOM_LIS2DUXS12_0_SPI_Recv

#define BSP_LIS2DUXS12_0_CS_PORT        CUSTOM_LIS2DUXS12_0_CS_PORT
#define BSP_LIS2DUXS12_0_CS_PIN         CUSTOM_LIS2DUXS12_0_CS_PIN

/* lsm6dso32x acc and gyro Sensor */
#define BSP_LSM6DSO32X_0_SPI_Init       CUSTOM_LSM6DSO32X_0_SPI_Init
#define BSP_LSM6DSO32X_0_SPI_DeInit     CUSTOM_LSM6DSO32X_0_SPI_DeInit
#define BSP_LSM6DSO32X_0_SPI_Send       CUSTOM_LSM6DSO32X_0_SPI_Send
#define BSP_LSM6DSO32X_0_SPI_Recv       CUSTOM_LSM6DSO32X_0_SPI_Recv

#define BSP_LSM6DSO32X_0_CS_PORT        CUSTOM_LSM6DSO32X_0_CS_PORT
#define BSP_LSM6DSO32X_0_CS_PIN         CUSTOM_LSM6DSO32X_0_CS_PIN

#define HANDLE_SPI                      hspi1

/* vd6283tx Light Sensor */
#define BUS_I2C_SCL_GPIO_PORT           BUS_I2C1_SCL_GPIO_PORT
#define BUS_I2C_SDA_GPIO_PORT           BUS_I2C1_SDA_GPIO_PORT
#define BUS_I2C_SCL_GPIO_PIN            BUS_I2C1_SCL_GPIO_PIN
#define BUS_I2C_SDA_GPIO_PIN            BUS_I2C1_SDA_GPIO_PIN

#define BSP_LIGHT_SENSOR_I2C_Init       BSP_I2C1_Init
#define BSP_LIGHT_SENSOR_I2C_DeInit     BSP_I2C1_DeInit
#define BSP_LIGHT_SENSOR_I2C_ReadReg    BSP_I2C1_ReadReg
#define BSP_LIGHT_SENSOR_I2C_WriteReg   BSP_I2C1_WriteReg

/* nfctag GPO pin */
#define BSP_GPO_Pin             GPIO_PIN_14
#define BSP_GPO_GPIO_Port	GPIOC
#define BSP_GPO_EXTI_LINE	EXTI_LINE_14
#define BSP_GPO_EXTI_IRQn	EXTI15_10_IRQn

extern EXTI_HandleTypeDef GPO_EXTI;
#define H_EXTI_14  GPO_EXTI

/* nfctag LPD pin */
#define BSP_LPD_Pin         GPIO_PIN_8
#define BSP_LPD_GPIO_Port   GPIOB

/* st25dv Dual Interface EEPROM */
#define BSP_ST25DVxxKC_I2C_Init         BSP_I2C3_Init
#define BSP_ST25DVxxKC_I2C_DeInit       BSP_I2C3_DeInit
#define BSP_ST25DVxxKC_I2C_ReadReg16    BSP_I2C3_ReadReg16
#define BSP_ST25DVxxKC_I2C_WriteReg16   BSP_I2C3_WriteReg16
#define BSP_ST25DVxxKC_I2C_Recv         BSP_I2C3_Recv
#define BSP_ST25DVxxKC_I2C_IsReady      BSP_I2C3_IsReady
#define BSP_ST25DVxxKC_GetTick          HAL_GetTick

#define HANDLE_I2C                      hi2c3

#ifdef __cplusplus
}
#endif

#endif /* __SMARTAG2_CONF_H__*/

