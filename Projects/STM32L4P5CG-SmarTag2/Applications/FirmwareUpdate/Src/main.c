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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include "Firmware_conf.h"
#include "SmarTag2.h"
#include "SmarTag2_nfctag.h"
#include "SmarTag2_nfctag_ex.h"
#include "st25ftm_config.h"
#include "st25ftm_process.h"

#include "flashl4_if.h"

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t ButtonPressed = 0;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

static void ConfigureSystickInterrupt(void);

static void HWInitialization(void);
static void SetNFCBehaviour(void);

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

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  ConfigureSystickInterrupt();
  HAL_Delay(2000);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  
  /* Initializes the Plaftorm */
  HWInitialization();
  
  /* Initialize the NFC Behaviour */
  SetNFCBehaviour();

  /* Infinite loop */
  while (1) {
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
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK){
    Error_Handler();
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
    Error_Handler();
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
    Error_Handler();
  }
  
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISET0_GPIO_Port, ISET0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RST_SAFE_Pin|VDD_EEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HIGH_G_CS_Pin|VDD_SAF_CTRL_Pin|VDD_AMB_MCU_Pin|VDD_ACC_MCU_Pin
                          |LPD_Pin|CH_ON_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pins : INT_PRE */
  GPIO_InitStruct.Pin = INT_PRE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ISET0_Pin */
  GPIO_InitStruct.Pin = ISET0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ISET0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_RTC_Pin */
  GPIO_InitStruct.Pin = IRQ_RTC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_RTC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_TEM_Pin */
  GPIO_InitStruct.Pin = INT_TEM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /*Configure GPIO pins : ACC_INT1_Pin */
  GPIO_InitStruct.Pin = ACC_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_SAFE_Pin VDD_EEP_Pin */
  GPIO_InitStruct.Pin = RST_SAFE_Pin|VDD_EEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : HIGH_G_INT1_Pin */
  GPIO_InitStruct.Pin = HIGH_G_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HIGH_G_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO1_Pin */
  GPIO_InitStruct.Pin = GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /*Configure GPIO pins : ACC_INT2_Pin */
  GPIO_InitStruct.Pin = ACC_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ACC_INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HIGH_G_CS_Pin VDD_SAF_CTRL_Pin VDD_AMB_MCU_Pin VDD_ACC_MCU_Pin
                           LPD_Pin CH_ON_Pin */
  GPIO_InitStruct.Pin = HIGH_G_CS_Pin|VDD_SAF_CTRL_Pin|VDD_AMB_MCU_Pin|VDD_ACC_MCU_Pin
                          |LPD_Pin|CH_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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
  BSP_LED_Init();
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  BSP_GPO_Init();
  
  /* Enable Power */
  BSP_SmarTag2_AMB_PowerOn();
  SMARTAG2_PRINTF("\r\nVDD AMB On\r\n");
  BSP_SmarTag2_ACC_PowerOn();
  SMARTAG2_PRINTF("VDD ACC On\r\n");
  BSP_SmarTag2_EEP_PowerOn();
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
  if(BSP_NFCTAG_ChangeI2CPassword(SMARTAG2_MSB_PASSWORD,SMARTAG2_LSB_PASSWORD)!=NFCTAG_OK ) {
    SMARTAG2_PRINTF("Error NFCTAG Changing the I2C password\r\n");
    Error_Handler_NFC(NFC_CONFIG_ERROR);
  } else {
    SMARTAG2_PRINTF("NFCTAG Changed the I2C password\r\n");
  }
  
  /* GPO sensible to RF Field change  */
  if(BSP_NFCTAG_WriteConfigIT(SMARTAG2_MSB_PASSWORD,SMARTAG2_LSB_PASSWORD,ST25DVXXKC_GPO1_ENABLE_MASK | ST25DVXXKC_GPO1_FIELDCHANGE_MASK)!=NFCTAG_OK ) {
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
      BSP_LED_On();
      HAL_Delay(100);
      BSP_LED_Off();
      HAL_Delay(500);
      SMARTAG2_PRINTF("NFC_CONFIG_ERROR\r\n\n");

    case NFC_WRITING_ERROR:
      /* Error on NFC Writing double Led blinking */
      BSP_LED_On();
      HAL_Delay(100);
      BSP_LED_Off();
      HAL_Delay(500);
      SMARTAG2_PRINTF("NFC_WRITING_ERROR\r\n\n");

    case NFC_READING_ERROR:
      /* Error on NFC Reading single Led blinking */
      BSP_LED_On();
      HAL_Delay(100);
      BSP_LED_Off();
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
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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

