/**
  ******************************************************************************
  * @file    main.c
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.0.2
  * @date    30-January-2023
  * @brief   Main program body
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

/* Private includes ----------------------------------------------------------*/
#include "stm32l4xx_hal.h"

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


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
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

    HAL_Init();

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
        return 1;
      }
    }

    /* Loop Cycle for writing the Firmare Update  */
    while(FwDestAddress<LastFwDestAddress) {
      if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FwDestAddress,*((uint64_t*)SourceAddress)) != HAL_OK) {
        return 1;
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
        return 1;
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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK){
    while(1);
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    while(1);
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    while(1);
  }
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

