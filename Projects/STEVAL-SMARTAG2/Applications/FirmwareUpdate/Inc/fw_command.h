/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fw_command.h
  * @author  MMY Application Team
  * @brief   Header file for fw_command.c
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
#ifndef FW_COMMAND_H
#define FW_COMMAND_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "flashl4_if.h"

/* Exported types ------------------------------------------------------------*/
  typedef  void (*pFunction)(void);
/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
#if defined   (__GNUC__)        /* GNU Compiler */
  #define __ALIGN64_END    __attribute__ ((aligned (8)))
  #define __ALIGN64_BEGIN
#else
  #define __ALIGN64_END
  #if defined   (__CC_ARM)      /* ARM Compiler */
    #define __ALIGN64_BEGIN    __align(8)
  #elif defined (__ICCARM__)    /* IAR Compiler */
    #define __ALIGN64_BEGIN
  #elif defined  (__TASKING__)  /* TASKING Compiler */
    #define __ALIGN64_BEGIN    __align(8)
  #endif /* __CC_ARM */
#endif /* __GNUC__ */
/* Exported functions ------------------------------------------------------- */
uint32_t COMMAND_EraseFlash( const uint32_t Address );
uint32_t COMMAND_EraseFlashFirstTime(void);
uint32_t Command_WriteBufferToFlash( const uint32_t StartAddress, const uint32_t offset, const uint8_t * const pData, const uint32_t size );
void COMMAND_Jump( void );

#ifdef __cplusplus
}
#endif

#endif  /* FW_COMMAND_H */

