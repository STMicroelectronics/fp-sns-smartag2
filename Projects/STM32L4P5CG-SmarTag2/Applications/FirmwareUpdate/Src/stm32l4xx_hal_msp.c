/**
  ******************************************************************************
  * @file    stm32l4xx_hal_msp.c
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.0.2
  * @date    30-January-2023
  * @brief   This file provides code for the MSP Initialization
  *          and de-Initialization codes.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

}

/**
  * @brief  Initializes the low level hardware: GPIO, CLOCK, NVIC for CRC.
  * @param  hcrc: pointer to a CRC_HandleTypeDef structure that contains
  *         the configuration information for CRC module.
  * @retval None
  */
void HAL_CRC_MspInit( CRC_HandleTypeDef *hcrc )
{
  __HAL_RCC_CRC_CLK_ENABLE( );
}

/**
  * @brief  DeInitializes the low level hardware: GPIO, CLOCK, NVIC for CRC.
  * @param  hcrc: pointer to a CRC_HandleTypeDef structure that contains
  *         the configuration information for CRC module.
  * @retval None
  */
void HAL_CRC_MspDeInit( CRC_HandleTypeDef *hcrc )
{
  __HAL_RCC_CRC_CLK_DISABLE( );
}

/**
  * @brief  Initializes the low level hardware: GPIO, CLOCK, NVIC for DMA.
  * @param  None
  * @retval None
  */
void HAL_DMA_MspInit( void )
{
  /* Peripheral clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
}

/**
  * @brief  DeInitializes the low level hardware: GPIO, CLOCK, NVIC for DMA.
  * @param  None
  * @retval None
  */
void HAL_DMA_MspDeInit( void )
{
  /* Peripheral clock enable */
  __HAL_RCC_DMA1_CLK_DISABLE();
}

