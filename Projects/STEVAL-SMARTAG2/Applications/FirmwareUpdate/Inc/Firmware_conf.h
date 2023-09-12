/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Firmware_conf.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.2.0
  * @date    30-May-2023
  * @brief   Configuration file for FirmwareUpdate application
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
#ifndef __FIRMWARE_CONF_H
#define __FIRMWARE_CONF_H

/* Uncomment the following define for disabling the LowPowerMode for Debug */
//#define SMARTAG2_ENABLE_DEBUG

/* Uncomment the following define for enabling the VCOM printf */
#define SMARTAG2_ENABLE_PRINTF

/******************  Don't change the following section ***********************/
/* st25dv tag sizes */
#define NFCTAG_64K_SIZE           ((uint32_t) 0x2000)

/* Eval-SmarTag2 includes the st25dv64k */
#define ST25DV_MAX_SIZE           NFCTAG_64K_SIZE
/* Dimension of the CC file in bytes */
#define ST25DV_CC_SIZE            8

/* Package Version only numbers 0->9 */
#define SMARTAG2_VERSION_MAJOR  1
#define SMARTAG2_VERSION_MINOR  2
#define SMARTAG2_VERSION_PATCH  0

#ifdef SMARTAG2_ENABLE_PRINTF
  #include <stdio.h>
  #define SMARTAG2_PRINTF(...) printf(__VA_ARGS__)
#else /* SMARTAG2_ENABLE_PRINTF */
  #define SMARTAG2_PRINTF(...)
#endif /* SMARTAG2_ENABLE_PRINTF */

#endif /* __FIRMWARE_CONF_H */

