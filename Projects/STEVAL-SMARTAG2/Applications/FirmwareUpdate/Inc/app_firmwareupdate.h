/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_firmwareupdate.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.2.0
  * @date    30-May-2023
  * @brief   This file provides code for the configuration FP-SNS-SMARTAG2
  *          application instances.
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
#ifndef __APP_FP_SNS_SMARTAG2_H
#define __APP_FP_SNS_SMARTAG2_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_exti.h"

#include "SmarTag2_nfctag_ex.h"
#include "Firmware_conf.h"
#include "flashl4_if.h"

/* Exported Define -----------------------------------------------------------*/
/* Error Codes */
#define NFC_READING_ERROR 0
#define NFC_WRITING_ERROR 1
#define NFC_CONFIG_ERROR  2

/* Exported Functions --------------------------------------------------------*/
void MX_FirmwareUpdate_Init(void);
void MX_FirmwareUpdate_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_FP_SNS_SMARTAG2_H */

