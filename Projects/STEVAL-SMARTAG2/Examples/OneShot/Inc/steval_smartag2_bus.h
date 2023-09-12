/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    steval_smartag2_bus.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.2.0
  * @date    30-May-2023
  * @brief   header file for the BSP BUS IO driver
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
#ifndef STEVAL_SMARTAG2_BUS_H
#define STEVAL_SMARTAG2_BUS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "steval_smartag2_conf.h"
#include "steval_smartag2_errno.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STEVAL_SMARTAG2
  * @{
  */

/** @defgroup STEVAL_SMARTAG2_BUS STEVAL_SMARTAG2 BUS
  * @{
  */

/** @defgroup STEVAL_SMARTAG2_BUS_Exported_Constants STEVAL_SMARTAG2 BUS Exported Constants
  * @{
  */

#define BUS_I2C3_INSTANCE I2C3
#define BUS_I2C3_SCL_GPIO_AF GPIO_AF4_I2C3
#define BUS_I2C3_SCL_GPIO_PIN GPIO_PIN_7
#define BUS_I2C3_SCL_GPIO_CLK_DISABLE() __HAL_RCC_GPIOA_CLK_DISABLE()
#define BUS_I2C3_SCL_GPIO_PORT GPIOA
#define BUS_I2C3_SCL_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define BUS_I2C3_SDA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_I2C3_SDA_GPIO_PORT GPIOB
#define BUS_I2C3_SDA_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()
#define BUS_I2C3_SDA_GPIO_AF GPIO_AF4_I2C3
#define BUS_I2C3_SDA_GPIO_PIN GPIO_PIN_4

#ifndef BUS_I2C3_POLL_TIMEOUT
   #define BUS_I2C3_POLL_TIMEOUT                0x1000U
#endif
/* I2C3 Frequency in Hz  */
#ifndef BUS_I2C3_FREQUENCY
   #define BUS_I2C3_FREQUENCY  1000000U /* Frequency of I2Cn = 100 KHz*/
#endif

#define BUS_SPI1_INSTANCE SPI1
#define BUS_SPI1_MISO_GPIO_PIN GPIO_PIN_11
#define BUS_SPI1_MISO_GPIO_PORT GPIOA
#define BUS_SPI1_MISO_GPIO_CLK_DISABLE() __HAL_RCC_GPIOA_CLK_DISABLE()
#define BUS_SPI1_MISO_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define BUS_SPI1_MISO_GPIO_AF GPIO_AF5_SPI1
#define BUS_SPI1_MOSI_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define BUS_SPI1_MOSI_GPIO_PORT GPIOA
#define BUS_SPI1_MOSI_GPIO_PIN GPIO_PIN_12
#define BUS_SPI1_MOSI_GPIO_CLK_DISABLE() __HAL_RCC_GPIOA_CLK_DISABLE()
#define BUS_SPI1_MOSI_GPIO_AF GPIO_AF5_SPI1
#define BUS_SPI1_SCK_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()
#define BUS_SPI1_SCK_GPIO_PORT GPIOB
#define BUS_SPI1_SCK_GPIO_PIN GPIO_PIN_3
#define BUS_SPI1_SCK_GPIO_AF GPIO_AF5_SPI1
#define BUS_SPI1_SCK_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

#ifndef BUS_SPI1_POLL_TIMEOUT
  #define BUS_SPI1_POLL_TIMEOUT                   0x1000U
#endif
/* SPI1 Baud rate in bps  */
#ifndef BUS_SPI1_BAUDRATE
   #define BUS_SPI1_BAUDRATE   10000000U /* baud rate of SPIn = 10 Mbps*/
#endif

#define BUS_I2C1_INSTANCE I2C1
#define BUS_I2C1_SCL_GPIO_PORT GPIOB
#define BUS_I2C1_SCL_GPIO_AF GPIO_AF4_I2C1
#define BUS_I2C1_SCL_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_I2C1_SCL_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()
#define BUS_I2C1_SCL_GPIO_PIN GPIO_PIN_6
#define BUS_I2C1_SDA_GPIO_PIN GPIO_PIN_7
#define BUS_I2C1_SDA_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()
#define BUS_I2C1_SDA_GPIO_PORT GPIOB
#define BUS_I2C1_SDA_GPIO_AF GPIO_AF4_I2C1
#define BUS_I2C1_SDA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

#ifndef BUS_I2C1_POLL_TIMEOUT
   #define BUS_I2C1_POLL_TIMEOUT                0x1000U
#endif
/* I2C1 Frequency in Hz  */
#ifndef BUS_I2C1_FREQUENCY
   #define BUS_I2C1_FREQUENCY  1000000U /* Frequency of I2Cn = 100 KHz*/
#endif

/**
  * @}
  */

/** @defgroup STEVAL_SMARTAG2_BUS_Private_Types STEVAL_SMARTAG2 BUS Private types
  * @{
  */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)
typedef struct
{
  pI2C_CallbackTypeDef  pMspInitCb;
  pI2C_CallbackTypeDef  pMspDeInitCb;
}BSP_I2C_Cb_t;
#endif /* (USE_HAL_I2C_REGISTER_CALLBACKS == 1U) */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
typedef struct
{
  pSPI_CallbackTypeDef  pMspInitCb;
  pSPI_CallbackTypeDef  pMspDeInitCb;
}BSP_SPI_Cb_t;
#endif /* (USE_HAL_SPI_REGISTER_CALLBACKS == 1U) */
/**
  * @}
  */

/** @defgroup STEVAL_SMARTAG2_LOW_LEVEL_Exported_Variables LOW LEVEL Exported Constants
  * @{
  */

extern I2C_HandleTypeDef hi2c3;
extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;

/**
  * @}
  */

/** @addtogroup STEVAL_SMARTAG2_BUS_Exported_Functions
  * @{
  */

/* BUS IO driver over I2C Peripheral */
HAL_StatusTypeDef MX_I2C3_Init(I2C_HandleTypeDef* hi2c);
int32_t BSP_I2C3_Init(void);
int32_t BSP_I2C3_DeInit(void);
int32_t BSP_I2C3_IsReady(uint16_t DevAddr, uint32_t Trials);
int32_t BSP_I2C3_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C3_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C3_WriteReg16(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C3_ReadReg16(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C3_Send(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C3_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C3_SendRecv(uint16_t DevAddr, uint8_t *pTxdata, uint8_t *pRxdata, uint16_t Length);
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)
int32_t BSP_I2C3_RegisterDefaultMspCallbacks (void);
int32_t BSP_I2C3_RegisterMspCallbacks (BSP_I2C_Cb_t *Callbacks);
#endif /* (USE_HAL_I2C_REGISTER_CALLBACKS == 1U) */
/* BUS IO driver over SPI Peripheral */
HAL_StatusTypeDef MX_SPI1_Init(SPI_HandleTypeDef* hspi);
int32_t BSP_SPI1_Init(void);
int32_t BSP_SPI1_DeInit(void);
int32_t BSP_SPI1_Send(uint8_t *pData, uint16_t Length);
int32_t BSP_SPI1_Recv(uint8_t *pData, uint16_t Length);
int32_t BSP_SPI1_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t Length);
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
int32_t BSP_SPI1_RegisterDefaultMspCallbacks (void);
int32_t BSP_SPI1_RegisterMspCallbacks (BSP_SPI_Cb_t *Callbacks);
#endif /* (USE_HAL_SPI_REGISTER_CALLBACKS == 1U) */

HAL_StatusTypeDef MX_I2C1_Init(I2C_HandleTypeDef* hi2c);
int32_t BSP_I2C1_Init(void);
int32_t BSP_I2C1_DeInit(void);
int32_t BSP_I2C1_IsReady(uint16_t DevAddr, uint32_t Trials);
int32_t BSP_I2C1_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_WriteReg16(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_ReadReg16(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_Send(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_SendRecv(uint16_t DevAddr, uint8_t *pTxdata, uint8_t *pRxdata, uint16_t Length);
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)
int32_t BSP_I2C1_RegisterDefaultMspCallbacks (void);
int32_t BSP_I2C1_RegisterMspCallbacks (BSP_I2C_Cb_t *Callbacks);
#endif /* (USE_HAL_I2C_REGISTER_CALLBACKS == 1U) */

int32_t BSP_GetTick(void);

/**
  * @}
  */

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

#endif /* STEVAL_SMARTAG2_BUS_H */

