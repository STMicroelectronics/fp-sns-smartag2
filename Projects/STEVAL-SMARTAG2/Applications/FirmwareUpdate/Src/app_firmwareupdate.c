/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_firmwareupdate.c
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
#include "app_firmwareupdate.h"

#include "steval_smartag2.h"
#include "st25ftm_config.h"
#include "st25ftm_process.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

/* USER CODE END PD*/

/* Exported Variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t ButtonPressed = 0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void User_Init(void);
static void FirmwareUpdateProcess(void);

static void ConfigureSystickInterrupt(void);

static void HWInitialization(void);
static void SetNFCBehaviour(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

void MX_FirmwareUpdate_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN FirmwareUpdate_Init_PreTreatment */

  /* USER CODE END FirmwareUpdate_Init_PreTreatment */

  /* Initialize FIRMWAREUPDATE application */

  User_Init();

  /* USER CODE BEGIN FirmwareUpdate_Init_PostTreatment */

  /* USER CODE END FirmwareUpdate_Init_PostTreatment */
}

/*
 * FP-SNS-SMARTAG2 background task
 */
void MX_FirmwareUpdate_Process(void)
{
  /* USER CODE BEGIN FirmwareUpdate_Process_PreTreatment */

  /* USER CODE END FirmwareUpdate_Process_PreTreatment */

  /* Process of the FIRMWAREUPDATE application */

  FirmwareUpdateProcess();

  /* USER CODE BEGIN FirmwareUpdate_Process_PostTreatment */

  /* USER CODE END FirmwareUpdate_Process_PostTreatment */
}

/**
  * @brief  Initialize User process.
  * @param  None
  * @retval None
  */
static void User_Init(void)
{
  ConfigureSystickInterrupt();
  HAL_Delay(2000);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* Initializes the Plaftorm */
  HWInitialization();

  /* Initialize the NFC Behaviour */
  SetNFCBehaviour();
}

/**
 * @brief  Configure the device as Client or Server and manage the communication
 *         between a client and a server.
 *
 * @param  None
 * @retval None
 */
static void FirmwareUpdateProcess(void)
{
  static uint32_t MailBoxEnabled=0;

  /* FTM Work Fuction*/
  if(MailBoxEnabled==1) {
    FTMManagement();
  }

  /* Handle User Button*/
  if(ButtonPressed) {
    ButtonPressed=0;
    if(MailBoxEnabled) {
      MailBoxEnabled=0;
      FTMManagementDeInit();
    } else {
      MailBoxEnabled =1;
      FTMManagementInit();
    }
  }

#ifndef SMARTAG2_ENABLE_DEBUG
  /* -> Stop mode */
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
#else /* SMARTAG2_ENABLE_DEBUG */
  /* Wait Next event */
  __WFI();
#endif /* SMARTAG2_ENABLE_DEBUG */
}

/**
  * @brief  Configure the Systick interrupt time
  * @param  None
  * @retval None
  */
static void ConfigureSystickInterrupt(void)
{
  /**Configure the Systick interrupt time
  */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  /**Configure the Systick
  */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief  Initializes the Hardware Configuration
  * @param  None
  * @retval None
  */
static void HWInitialization(void)
{
#ifdef SMARTAG2_ENABLE_PRINTF
  /* Initialize Virtual COM Port */
  if(BSP_COM_Init(COM1) == BSP_ERROR_NONE) {
    SMARTAG2_PRINTF("UART Initialized\r\n");
  } else {
    Error_Handler();
  }
#endif /* SMARTAG2_ENABLE_PRINTF */

  SMARTAG2_PRINTF("STMicroelectronics FirmwareUpdate:\r\n"
         "\tVersion %d.%d.%d\r\n"
         "\tSTEVAL_SMARTAG2 board\r\n",
         SMARTAG2_VERSION_MAJOR,
         SMARTAG2_VERSION_MINOR,
         SMARTAG2_VERSION_PATCH);

  SMARTAG2_PRINTF("\t(HAL %ld.%ld.%ld_%ld)\r\n"
        "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n\n"
#elif defined (__ARMCC_VERSION)
        " (KEIL)\r\n\n"
#elif defined (__GNUC__)
        " (STM32CubeIDE)\r\n\n"
#endif
           ,
           HAL_GetHalVersion() >>24,
          (HAL_GetHalVersion() >>16)&0xFF,
          (HAL_GetHalVersion() >> 8)&0xFF,
           HAL_GetHalVersion()      &0xFF,
         __DATE__,__TIME__);

  /* Initialize LED */
  BSP_LED_Init(LED2);
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  BSP_NFCTAG_GPO_Init();

  /* Enable Power */
//  BSP_ENV_SENSOR_PowerOn();
//  SMARTAG2_PRINTF("\r\nVDD AMB On\r\n");
//  BSP_MOTION_SENSOR_PowerOn();
//  SMARTAG2_PRINTF("VDD ACC On\r\n");
  BSP_NFCTAG_EEP_PowerOn();
  SMARTAG2_PRINTF("VDD EEP On\r\n");
  /* Rise time required by VDD_EEPROM for NFC */
  HAL_Delay(200);

  /* Init I2C interface */
  if(BSP_NFCTAG_Init(BSP_NFCTAG_INSTANCE)!=NFCTAG_OK ) {
    SMARTAG2_PRINTF("Error NFCTAG Initialized\r\n");
    Error_Handler_NFC(NFC_CONFIG_ERROR);
  } else {
    SMARTAG2_PRINTF("NFCTAG Initialized\r\n");
  }
}

/**
  * @brief  Initializes the NFC Behaviour (R/F Field Change Sensible)
  * @param  None
  * @retval None
  */
static void SetNFCBehaviour(void)
{
  /* Setting the New Password for I2C protection */
  if(BSP_NFCTAG_ChangeI2CPassword(NFC_MSB_PASSWORD,NFC_LSB_PASSWORD)!=NFCTAG_OK ) {
    SMARTAG2_PRINTF("Error NFCTAG Changing the I2C password\r\n");
    Error_Handler_NFC(NFC_CONFIG_ERROR);
  } else {
    SMARTAG2_PRINTF("NFCTAG Changed the I2C password\r\n");
  }

  /* GPO sensible to RF Field change  */
  if(BSP_NFCTAG_WriteConfigIT(NFC_MSB_PASSWORD,NFC_LSB_PASSWORD,ST25DVXXKC_GPO1_ENABLE_MASK | ST25DVXXKC_GPO1_FIELDCHANGE_MASK)!=NFCTAG_OK ) {
    SMARTAG2_PRINTF("Error NFCTAG Writing the Interrupt Configuration\r\n");
    Error_Handler_NFC(NFC_CONFIG_ERROR);
  } else {
    SMARTAG2_PRINTF("NFCTAG Written the Interrupt Configuration\r\n");
  }
}

/**
  * @brief  Error Handler for Reading/Writing function for NFC.
  *         User may add here some code to deal with this error.
  * @param  uint32_t ErroCode Error Code Flag: Reading/Writing/Configuration
  * @retval None
  */
void Error_Handler_NFC(uint32_t ErroCode)
{
  switch(ErroCode) {
    case NFC_CONFIG_ERROR:
      /* Error on Configuration triple Led blinking */
      BSP_LED_On(LED2);
      HAL_Delay(100);
      BSP_LED_Off(LED2);
      HAL_Delay(500);
      SMARTAG2_PRINTF("NFC_CONFIG_ERROR\r\n\n");

    case NFC_WRITING_ERROR:
      /* Error on NFC Writing double Led blinking */
      BSP_LED_On(LED2);
      HAL_Delay(100);
      BSP_LED_Off(LED2);
      HAL_Delay(500);
      SMARTAG2_PRINTF("NFC_WRITING_ERROR\r\n\n");

    case NFC_READING_ERROR:
      /* Error on NFC Reading single Led blinking */
      BSP_LED_On(LED2);
      HAL_Delay(100);
      BSP_LED_Off(LED2);
      HAL_Delay(500);
      SMARTAG2_PRINTF("NFC_READING_ERROR\r\n\n");
  }
}

/**
 * @brief  BSP Push Button callback
 * @param  Button Specifies the pin connected EXTI line
 * @retval None.
 */
void BSP_PB_Callback(Button_TypeDef Button)
{
  ButtonPressed=1;
}

/**
  * @brief  BSP GPO callback
  * @retval None.
  */
void BSP_GPO_Callback(void)
{
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  GPO_Activated =1;
}

/**
  * @brief This function provides accurate delay (in milliseconds) based
  *        on variable incremented.
  * @note This is a user implementation using WFI state
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay){
    __WFI();
  }
}

/* This code because the CRC is included but the c-code is not generate from STM32CubeMX
  in order to avoid the conflict with the middleware ST25FTM that uses a static handle for CRC. */
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
