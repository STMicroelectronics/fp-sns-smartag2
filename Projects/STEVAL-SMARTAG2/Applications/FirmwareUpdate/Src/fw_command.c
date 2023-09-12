/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fw_command.c
  * @author  MMY Application Team
  * @brief   This file provides all the flash programming command functions.
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

/* Includes ------------------------------------------------------------------*/
#include "fw_command.h"

/** @addtogroup ST25_Discovery_Demo
  * @{
  */

/** @defgroup Flash_Command Flash Command
  * @brief This module implements high level functions to write firmware or data to flash.
  * @details The module covers following functions:
  * - Erase Flash command.
  * - Write buffer to flash.
  * - Jump to firmware command.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t              JumpAddress;
pFunction             Jump_To_Application;

extern void Error_Handler( void );

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Command To erase specific Flash memory area.
  * @param  Address Start address for erasing data.
  * @retval 0 Erase sectors done with success.
  * @retval 1 Erase error.
  */
uint32_t COMMAND_EraseFlash( const uint32_t Address )
{
  uint32_t ret = 0;
  FLASH_If_FlashUnlock( );
  /* Erase FLASH sectors to download image */
  if( Address == FIRMWARE_ADDRESS )
  {
      if( FLASH_If_PageErase( FIRMWARE_ADDRESS, FIRMWARE_FLASH_LAST_PAGE_ADDRESS ) != 0x00 )
    {
      /* Error Erase */
      ret = 1;
    }
  }
//  else
//  {
//      if( FLASH_If_EraseSectors( USER_DATA_ADDRESS, USER_DATA_FLASH_LAST_PAGE_ADDRESS ) != 0x00 )
//    {
//      /* Error Erase */
//      ret = 1;
//    }
//  }
  FLASH_If_FlashLock( );

  return ret;
}

/**
  * @brief  Command To erase the full Flash Region.
  * @param  None.
  * @retval 0 Erase sectors done with success.
  * @retval 1 Erase error.
  */
uint32_t COMMAND_EraseFlashFirstTime(void)
{
  uint32_t ret = 0;
  FLASH_If_FlashUnlock( );
  /* Erase FLASH sectors to download image */
  if( FLASH_If_PageErase( 0x08080000, FIRMWARE_FLASH_LAST_PAGE_ADDRESS ) != 0x00 )
  {
    /* Error Erase */
    ret = 1;
  }
  FLASH_If_FlashLock( );

  return ret;
}

/**
  * @brief  Writes buffer to Flash memory.
  * @param  StartAddress Start address for writing data.
  * @param  offset Offset of data to write.
  * @param  pData Buffer pointer to write.
  * @param  size Size of data to write.
  * @retval 0 Write Success.
  * @retval 1 Write Error.
  */
uint32_t Command_WriteBufferToFlash( const uint32_t StartAddress, const uint32_t offset, const uint8_t * const pData, const uint32_t size )
{
  uint32_t ret = 0;
  uint64_t *tmp_pdata = (uint64_t *)pData;
  if( StartAddress == FIRMWARE_ADDRESS )
  {
    ret = FLASH_If_WriteBuffer( (FIRMWARE_ADDRESS + offset), tmp_pdata, size / 8 );
  }
  else
  {
    ret = FLASH_If_WriteBuffer( (USER_DATA_ADDRESS + offset), tmp_pdata, size / 8 );
  }

  return ret;
}

/**
  * @brief  Jump to user program.
  * @param  None No parameters.
  * @return None.
  */
void COMMAND_Jump( void )
{
  FLASH_If_WriteMagicNumber();
}

/**
  * @}
  */

/**
  * @}
  */

