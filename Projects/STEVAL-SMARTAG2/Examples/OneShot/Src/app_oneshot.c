/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_oneshot.c
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
#include "app_oneshot.h"

#include "AppOneShot.h"
#include "SmartNFC.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

/* USER CODE END PD*/

/* Exported Variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void User_Init(void);
static void HWInitialization(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

void MX_OneShot_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN OneShot_Init_PreTreatment */

  /* USER CODE END OneShot_Init_PreTreatment */

  /* Initialize ONESHOT application */

  User_Init();

  /* USER CODE BEGIN OneShot_Init_PostTreatment */

  /* USER CODE END OneShot_Init_PostTreatment */
}

/*
 * FP-SNS-SMARTAG2 background task
 */
void MX_OneShot_Process(void)
{
  /* USER CODE BEGIN OneShot_Process_PreTreatment */

  /* USER CODE END OneShot_Process_PreTreatment */

  /* Process of the ONESHOT application */

  /* USER CODE BEGIN OneShot_Process_PostTreatment */

  /* USER CODE END OneShot_Process_PostTreatment */
}

/**
 * @brief  Initialize User process.
 *
 * @param  None
 * @retval None
 */
static void User_Init(void)
{
  HAL_Delay(1500);

  /* Initializes the Plaftorm */
  HWInitialization();

  /* SmarTag Application Start */
  OneShotAppStart();

  MEMS_Sensors_ReadData();

  ST25_RETRY(BSP_NFCTAG_ResetRFSleep_Dyn(BSP_NFCTAG_INSTANCE));
  SMARTAG2_PRINTF("WakeUp RF\r\n");
  SMARTAG2_PRINTF("End of Program\r\n");
}

/**
  * @brief  Initializes the Hardware Configuration
  * @param  None
  * @retval None
  */
static void HWInitialization(void)
{

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

#ifdef SMARTAG2_ENABLE_PRINTF
  /* Initialize Virtual COM Port */
  if(BSP_COM_Init(COM1) != BSP_ERROR_NONE) {
    STNFC_Error_Handler(STNFC_INIT_ERROR);
  }

  SMARTAG2_PRINTF("STMicroelectronics One Shot:\r\n"
         "\tVersion %d.%d.%d\r\n"
         "\tSTEVAL_SMARTAG2 board\r\n",
         SMARTAG2_VERSION_MAJOR,SMARTAG2_VERSION_MINOR,SMARTAG2_VERSION_PATCH);
#endif /* SMARTAG2_ENABLE_PRINTF */
}

/**
  * @brief  Error Handler for Reading/Writing function for NFC.
  *         User may add here some code to deal with this error.
  * @param  SNFC_ErrorCode_t ErroCode Error Code Flag: Reading/Writing/Configuration
  * @retval None
  */
void  STNFC_Error(SNFC_ErrorCode_t ErroCode, char *file, int32_t line)
{
#ifdef SMARTAG2_ENABLE_PRINTF
  switch(ErroCode) {
   case STNFC_RUNTIME_ERROR:
      SMARTAG2_PRINTF("STNFC_RUNTIME_ERROR\r\n");
      break;
    case STNFC_INIT_ERROR:
      SMARTAG2_PRINTF("STNFC_INIT_ERROR\r\n");
      break;
    case STNFC_CONFIG_ERROR:
      SMARTAG2_PRINTF("STNFC_CONFIG_ERROR\r\n");
      break;
    case STNFC_WRITING_ERROR:
      SMARTAG2_PRINTF("STNFC_WRITING_ERROR\r\n");
      break;
    case STNFC_READING_ERROR:
      SMARTAG2_PRINTF("STNFC_READING_ERROR\r\n");
      break;
  }
#endif /* SMARTAG2_ENABLE_PRINTF */
}

