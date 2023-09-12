/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_simplebootloader.c
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.2.0
  * @date    30-May-2023
  * @brief   This file provides code for FP-SNS-SMARTAG2 application.
  *
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
#include "app_simplebootloader.h"

#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define OTA_MAGIC_NUM ((uint32_t)0xDEADBEEF)

/* Running program Position */
#define PROG_ADDRESS_START 0x08002000

/* Max Program Size */
#define MAX_PROG_SIZE (0x7FFFF-0x1FFF)

/* Board  FW OTA Magic Number Position */
#define OTA_ADDRESS_START  0x08080000

#define MCR_PAGE_BANK1(Addr)  (((Addr) - FLASH_BASE) / FLASH_PAGE_SIZE)
#define MCR_PAGE_BANK2(Addr)  (((Addr) - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE)

/* USER CODE BEGIN PD */

/* USER CODE END PD*/

/* Exported Variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Imported function prototypes ----------------------------------------------*/
extern void SystemClock_Config(void);

/* USER CODE BEGIN IFP */

/* USER CODE END IFP */

/* Private function prototypes -----------------------------------------------*/
static void SimpleBootLoader(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

void MX_SimpleBootLoader_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN SimpleBootLoader_Init_PreTreatment */

  /* USER CODE END SimpleBootLoader_Init_PreTreatment */

  /* Initialize SIMPLEBOOTLOADER application */

  SimpleBootLoader();

  /* USER CODE BEGIN SimpleBootLoader_Init_PostTreatment */

  /* USER CODE END SimpleBootLoader_Init_PostTreatment */
}

/**
  * @brief  Boot Loader Procedure
  * @param  None
  * @retval None
  */
static void SimpleBootLoader(void)
{
  uint32_t SourceAddress = OTA_ADDRESS_START;
  uint32_t data32 = *((uint32_t*) SourceAddress);

  /* Check if there is Full/Partial Firmware Update */
  if(data32==OTA_MAGIC_NUM){
    /* Make the Firmware Update*/

    /* Update Size */
    uint32_t SizeOfUpdate;

    /* First Destination Address to change */
    uint32_t FwDestAddress;

    /* This is the last Address to change */
    uint32_t LastFwDestAddress;

    /* Configure the System clock */
    SystemClock_Config();

    /* Update Size */
    SizeOfUpdate = *(uint32_t*) (SourceAddress+4);

    if((SizeOfUpdate==0) | (SizeOfUpdate>MAX_PROG_SIZE)){
      /* If there is not the dimension of the Update... we set the Full program Size */
      SizeOfUpdate = MAX_PROG_SIZE;
    }

    FwDestAddress=PROG_ADDRESS_START;

    /* Last Destination Address that we need to change */
    LastFwDestAddress = FwDestAddress + SizeOfUpdate;

    /* Source Address */
    SourceAddress +=8;
    {
      /*********************** Flash Erase *********************************/
      FLASH_EraseInitTypeDef EraseInitStruct;
      uint32_t SectorError = 0;

      EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
      EraseInitStruct.Banks       = FLASH_BANK_1;
      EraseInitStruct.Page        = MCR_PAGE_BANK1(FwDestAddress);

      EraseInitStruct.NbPages = MCR_PAGE_BANK1(LastFwDestAddress)-EraseInitStruct.Page;

      /* Unlock the Flash */
      HAL_FLASH_Unlock();

       /* Clear OPTVERR bit set on virgin samples */
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
      /* Clear PEMPTY bit set (as the code is executed from Flash which is not empty) */
      if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PEMPTY) != 0) {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PEMPTY);
      }

      if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK){
        Error_Handler();
      }
    }

    /* Loop Cycle for writing the Firmare Update  */
    while(FwDestAddress<LastFwDestAddress) {
      if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FwDestAddress,*((uint64_t*)SourceAddress)) != HAL_OK) {
        Error_Handler();
      }

      FwDestAddress+=8;
      SourceAddress+=8;
    }

    /******************* Delete Magic Number ***************************/
    {
      FLASH_EraseInitTypeDef EraseInitStruct;
      uint32_t SectorError = 0;

      /* Reset the Second half Flash */
      EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
      EraseInitStruct.Banks       = FLASH_BANK_2;
      EraseInitStruct.Page        = MCR_PAGE_BANK2(OTA_ADDRESS_START);
      EraseInitStruct.NbPages     = 1;

      /* Clear OPTVERR bit set on virgin samples */
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
      /* Clear PEMPTY bit */
      if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PEMPTY) != 0) {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PEMPTY);
      }

      /* Delete the Magic Number Used for FOTA */
      if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
        Error_Handler();
      }
    }

    /* Lock the Flash */
    HAL_FLASH_Lock();

    /* System Reboot */
    HAL_NVIC_SystemReset();
  } else {
    /* Jump To Normal boot */
    typedef  void (*pFunction)(void);

    pFunction JumpToApplication;
    uint32_t JumpAddress;

    HAL_DeInit();

    /* reset all interrupts to default */
    // __disable_irq();

    /* Jump to system memory */
    JumpAddress = *(__IO uint32_t*) (PROG_ADDRESS_START + 4);
    JumpToApplication = (pFunction) JumpAddress;
    /* Initialize user application's Stack Pointer */
    __set_MSP(*(__IO uint32_t*) PROG_ADDRESS_START);
    JumpToApplication();
  }
}

