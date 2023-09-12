/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    flashl4_if.h
  * @author  MMY Application Team
  * @brief   Header file for flash_if.c
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
#ifndef FLASH_IF_H
#define FLASH_IF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_firmwareupdate.h"

/** @addtogroup Flash_Interface
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Define the address from where firmware will be stored. */
#define FIRMWARE_ADDRESS        0x08080008      /* Upgraded firmware base address */

/* Last Page Address for firmware */
#define FIRMWARE_FLASH_LAST_PAGE_ADDRESS  0x080FFFFF - 0x2000      /* Upgraded firmware last address */

/* Define the user application size */
#define FIRMWARE_FLASH_SIZE   (FIRMWARE_FLASH_LAST_PAGE_ADDRESS - FIRMWARE_ADDRESS + 1)     /* Upgraded firmware area size */

/* Define the address from where user data will be stored. */
#define USER_DATA_ADDRESS        (0x08080008)      /* User data base address */

/* Last Page Address for data */
#define USER_DATA_FLASH_LAST_PAGE_ADDRESS  (FIRMWARE_FLASH_LAST_PAGE_ADDRESS)   /* User data last address */

/* Define the user data size */
#define USER_DATA_FLASH_SIZE   (USER_DATA_FLASH_LAST_PAGE_ADDRESS - USER_DATA_ADDRESS + 1)    /* User data area size */

/* Board  FW OTA Magic Number */
#define OTA_MAGIC_NUM 0xDEADBEEF

/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void FLASH_If_FlashLock( void );
void FLASH_If_FlashUnlock( void );
FlagStatus FLASH_If_ReadOutProtectionStatus( void );
uint32_t FLASH_If_GetPage(uint32_t Addr);
uint32_t FLASH_If_GetBank(uint32_t Addr);
uint32_t FLASH_If_MassErase( const uint32_t Address );
uint32_t FLASH_If_PageErase( const uint32_t Address, const uint32_t LastAddress );
void FLASH_If_DMA_Init(void);
void FLASH_If_DMA_DeInit( void );
uint32_t FLASH_If_DMA_WriteBuffer( const uint32_t Address, const uint32_t * const pData , const uint32_t Size );
uint32_t FLASH_If_DMA_IT_WriteBuffer( const uint32_t Address, const uint32_t * const pData , const uint32_t Size );
uint32_t FLASH_If_WriteDWord( const uint32_t Address, const uint64_t Data );
uint32_t FLASH_If_WriteBuffer( const uint32_t Address, const uint64_t * const pData , const uint32_t Size );
void FLASH_If_WriteMagicNumber( void );

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* FLASH_IF_H */

