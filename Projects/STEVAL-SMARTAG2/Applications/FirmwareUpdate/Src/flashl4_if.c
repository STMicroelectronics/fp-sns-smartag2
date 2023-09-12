/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    flashf4_if.c
  * @author  MMY Application Team
  * @brief   This file provides all the flash layer functions for L4 MCU.
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
#include "flashl4_if.h"

/** @addtogroup ST25_Discovery_Demo
  * @{
  */

/** @defgroup Flash_Interface Flash memory api
  * @brief   This module defines an API to access the internal flash memory.
  * @details The module covers following functions:
  * - Lock Flash modification.
  * - Unlock Flash modification.
  * - Read Flash protection status.
  * - Erase Flash sectors.
  * - Write Flash data.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t PageError = 0;
uint32_t OB_RDP_LEVEL;
DMA_HandleTypeDef DmaHandle;
static FLASH_OBProgramInitTypeDef FLASH_OBProgramInitStruct;
static FLASH_EraseInitTypeDef FLASH_EraseInitStruct;

/* Private function prototypes -----------------------------------------------*/
static void TransferComplete(DMA_HandleTypeDef *DmaHandle);
static void TransferError(DMA_HandleTypeDef *DmaHandle);

extern void Error_Handler( void );
extern void HAL_DMA_MspInit( void );
extern void HAL_DMA_MspDeInit( void );

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Locks the Flash to disable the flash control register access.
  * @param  None No parameters
  * @return None.
  */
void FLASH_If_FlashLock( void )
{
  HAL_FLASH_Lock( );
}

/**
  * @brief  Unlocks the Flash to enable the flash control register access.
  * @param  None No parameters.
  * @return None.
  */
void FLASH_If_FlashUnlock( void )
{
  HAL_FLASH_Unlock( );
}

/**
  * @brief  Gets Flash readout protection status.
  * @param  None No parameters.
  * @retval ReadOut protection status.
  */
FlagStatus FLASH_If_ReadOutProtectionStatus( void )
{
  FlagStatus readoutstatus = RESET;

  FLASH_OBProgramInitStruct.RDPLevel = OB_RDP_LEVEL;

  HAL_FLASHEx_OBGetConfig( &FLASH_OBProgramInitStruct );

  if( OB_RDP_LEVEL == SET )
  {
    readoutstatus = SET;
  }
  else
  {
    readoutstatus = RESET;
  }

  return readoutstatus;
}

/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
uint32_t FLASH_If_GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
uint32_t FLASH_If_GetBank(uint32_t Addr)
{
  uint32_t bank = 0;

  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
  	/* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
  	/* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }

  return bank;
}

/**
  * @brief  Erases the required FLASH Sectors computed with destination address.
  * @param  Address Start address for erasing data.
  * @param  LastAddress End address of flash area.
  * @retval 0 Erase sectors done with success.
  * @retval 1 Erase error.
  */
uint32_t FLASH_If_MassErase( const uint32_t Address )
{
  uint32_t flashbank = 0;

  flashbank = FLASH_If_GetBank( Address );
  /* Mass Erase flash bank */

  FLASH_EraseInitStruct.TypeErase = FLASH_TYPEERASE_MASSERASE;
  FLASH_EraseInitStruct.Banks = flashbank;

  CLEAR_BIT(FLASH->CR, FLASH_CR_PG);

  if( HAL_FLASHEx_Erase(&FLASH_EraseInitStruct, &PageError) != HAL_OK )
    return (1);

  return (0);
}

/**
  * @brief  Erases the required FLASH Pages computed with destination address.
  * @param  Address Start address for erasing data.
  * @param  LastAddress End address of flash area.
  * @retval 0 Erase sectors done with success.
  * @retval 1 Erase error.
  */
uint32_t FLASH_If_PageErase( const uint32_t Address, const uint32_t LastAddress )
{
  uint32_t flashbank = 0;
  uint32_t flashpage = 0;

  flashbank = FLASH_If_GetBank( Address );
  flashpage = FLASH_If_GetPage( Address );
  /* Mass Erase flash bank */

  FLASH_EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  FLASH_EraseInitStruct.Banks = flashbank;
  FLASH_EraseInitStruct.Page = flashpage;
  FLASH_EraseInitStruct.NbPages = ((LastAddress - Address) / FLASH_PAGE_SIZE) + 1;

  CLEAR_BIT(FLASH->CR, FLASH_CR_PG);

  if( HAL_FLASHEx_Erase(&FLASH_EraseInitStruct, &PageError) != HAL_OK )
    return (1);

  return (0);
}

/**
  * @brief  Configure the DMA controller for SRAM to FLASH transfer
  * @note  This function is used to :
  *        -1- Enable DMA clock
  *        -2- Select the DMA functional Parameters
  *        -3- Select the DMA instance to be used for the transfer
  *        -4- Select Callbacks functions called after Transfer complete and
               Transfer error interrupt detection
  *        -5- Initialize the DMA Channel
  *        -6- Configure NVIC for DMA transfer complete/error interrupts
  *        -7- Start the DMA transfer using the interrupt mode
  * @param  None
  * @retval None
  */
void FLASH_If_DMA_Init( void )
{
  DmaHandle.Init.Direction = DMA_MEMORY_TO_MEMORY;          /* M2M transfer mode                */
  DmaHandle.Init.PeriphInc = DMA_PINC_ENABLE;               /* Peripheral increment mode Enable */
  DmaHandle.Init.MemInc = DMA_MINC_ENABLE;                  /* Memory increment mode Enable     */
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; /* Peripheral data alignment : Word */
  DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;    /* memory data alignment : Word     */
  DmaHandle.Init.Mode = DMA_NORMAL;                         /* Normal DMA mode                  */
  DmaHandle.Init.Priority = DMA_PRIORITY_HIGH;              /* priority level : high            */

  DmaHandle.Instance = DMA1_Channel1;

  DmaHandle.XferCpltCallback  = TransferComplete;
  DmaHandle.XferErrorCallback = TransferError;

  HAL_DMA_MspInit( );
  if(HAL_DMA_Init(&DmaHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);

  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief  UnConfigure the DMA controller for SRAM to FLASH transfer
  * @param  None
  * @retval None
  */
void FLASH_If_DMA_DeInit( void )
{
  DmaHandle.Instance = DMA1_Channel1;

  HAL_DMA_DeInit( &DmaHandle );

  HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);

  HAL_DMA_MspDeInit( );
}

/**
  * @brief  DMA Transfer complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
static void TransferComplete(DMA_HandleTypeDef *DmaHandle)
{
  CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
  FLASH_If_FlashLock( );
}

/**
  * @brief  DMA Transfer error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
static void TransferError(DMA_HandleTypeDef *DmaHandle)
{
  CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
  FLASH_If_FlashLock( );
}

/**
  * @brief  Writes a data buffer in the flash through the DMA.
  * @param  Address Start address for writing data buffer (must be DWORD aligned).
  * @param  pData Pointer on data buffer.
  * @param  Size Size of the data.
  * @retval 0 Data successfully written to Flash memory.
  * @retval 1 Error occurred while writing data in Flash memory.
  */
uint32_t FLASH_If_DMA_WriteBuffer( const uint32_t Address, const uint32_t * const pData , const uint32_t Size )
{
  uint32_t ret = 0;

  FLASH_If_FlashUnlock( );
  /* Set PG bit */
  SET_BIT(FLASH->CR, FLASH_CR_PG);
  if( HAL_DMA_Start( &DmaHandle, (uint32_t)pData, Address, Size ) != HAL_OK )
  {
    ret = 1;
  }
  HAL_DMA_PollForTransfer( &DmaHandle, HAL_DMA_FULL_TRANSFER, 0xFFF);
  CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
  FLASH_If_FlashLock( );

  return ret;
}

/**
  * @brief  Writes a data buffer in the flash through the DMA.
  * @param  Address Start address for writing data buffer (must be DWORD aligned).
  * @param  pData Pointer on data buffer.
  * @param  Size Size of the data.
  * @retval 0 Data successfully written to Flash memory.
  * @retval 1 Error occurred while writing data in Flash memory.
  */
uint32_t FLASH_If_DMA_IT_WriteBuffer( const uint32_t Address, const uint32_t * const pData , const uint32_t Size )
{
  uint32_t ret = 0;

  FLASH_If_FlashUnlock( );
  /* Set PG bit */
  SET_BIT(FLASH->CR, FLASH_CR_PG);
  if( HAL_DMA_Start_IT( &DmaHandle, (uint32_t)pData, Address, Size ) != HAL_OK )
  {
    ret = 1;
  }

  return ret;
}

/**
  * @brief  Writes a data in flash (data are 32-bit aligned).
  * @param  Address Start address for writing data buffer.
  * @param  LastAddress End address of flash area.
  * @param  Data Word data value to write.
  * @retval 0 Data successfully written to Flash memory.
  * @retval 1 Error occurred while writing data in Flash memory.
  */
uint32_t FLASH_If_WriteDWord( const uint32_t Address, const uint64_t pData )
{
  uint32_t ret = 1;

  /* Program the user Flash area word by word
    (area defined by Address and LastAddress) ***********/

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, pData) == HAL_OK)
      ret = 0;

  return ret;
}

/**
  * @brief  Writes a data buffer in the flash.
  * @param  Address Start address for writing data buffer.
  * @param  LastAddress End address of flash area.
  * @param  pData Pointer on data buffer.
  * @param  Size Size of the data.
  * @retval 0 Data successfully written to Flash memory.
  * @retval 1 Error occurred while writing data in Flash memory.
  */
uint32_t FLASH_If_WriteBuffer( const uint32_t Address, const uint64_t * const pData , const uint32_t Size )
{
  uint32_t cnt = 0;
  uint32_t ret = 0;

  FLASH_If_FlashUnlock( );
  for( cnt = 0; cnt < Size; cnt++ )
  {
    ret |= FLASH_If_WriteDWord( (Address + (8 * cnt)), *(pData + cnt) );
  }
  FLASH_If_FlashLock( );

  return ret;
}

/**
  * @brief  Write Magic Number for BootLoader
  * @retval None
  */
void FLASH_If_WriteMagicNumber( void )
{
  uint64_t ValueToWrite;
  HAL_FLASH_Unlock();

  /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

  ValueToWrite = 0 /*(((uint64_t)SizeOfUpdateBlueFW)<<32) */ | (OTA_MAGIC_NUM);

  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x08080000, ValueToWrite) != HAL_OK) {
    SMARTAG2_PRINTF("Error writing the Magic number\r\n");
  }

  HAL_FLASH_Lock();

  HAL_NVIC_SystemReset();
}

/**
  * @}
  */

/**
  * @}
  */

