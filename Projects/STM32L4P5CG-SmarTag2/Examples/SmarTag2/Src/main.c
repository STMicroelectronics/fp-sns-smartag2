/**
  ******************************************************************************
  * @file    main.c
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.1.0
  * @date    22-February-2023
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
#include "SMARTAG2_config.h"
#include "AppSmarTag.h"
#include "SmarTag2.h"
#include "SmartNFC.h"

/* Exported Variables --------------------------------------------------------*/
RTC_HandleTypeDef hrtc;
uint8_t AccInit_LIS2DUXS12_Done=0;
uint8_t AccInit_LSM6DSO32X_Done=0;

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static volatile uint8_t MemsInterrupt = 0;
/* Interrupt from Timer */
static volatile uint32_t WakeUpTimerInterrupt =0;
/* Interrupt from User Button*/
static volatile uint32_t ButtonPressed = 0;

/* There is one RF Activity? */
static volatile uint32_t RFActivity = 0;

/* Identify if the WakeUp Timer is set  */
static uint32_t WakeUpTimerIsSet =0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void HWInitializationStep1(void);
static void WakeUpTimerCallBack(void);
static void FactoryReset(void);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
#include "SmarTag2_motion_sensors_ex.h"
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  
  /* Initializes the Plaftorm */
  HWInitializationStep1();
  
#ifndef SMARTAG2_ENABLE_DEBUG  
   /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableLowPowerRunMode();
 
  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableInternalWakeUpLine();
  /* Select MSI as system clock source after Wake Up from Stop mode */
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG (RCC_STOP_WAKEUPCLOCK_MSI);
#endif /* SMARTAG2_ENABLE_DEBUG */

  if(!WakeUpTimerIsSet) {
    SMARTAG2_PRINTF("Set WakeUp timer\r\n\r\n");
    if(HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0)!= HAL_OK) {
      STNFC_Error_Handler(STNFC_INIT_ERROR);
    }
    
    WakeUpTimerIsSet= 1;
  }

#ifndef SMARTAG2_ENABLE_DEBUG
  /* -> Stop mode */
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
#endif /* SMARTAG2_ENABLE_DEBUG */
  
  SMARTAG2_PRINTF("Wait %d sec before autoStart\r\n\r\n", SMARTAG2_AUTOSTART_SECONDS);
  
  /* Infinite loop */
  while (1)
  {
    /* Wake Up due to RTC timer */
    if(WakeUpTimerInterrupt) {
      WakeUpTimerInterrupt =0;
      WakeUpTimerCallBack();
    }
    
    /* SmarTag Process */
    SmarTagAppProcess();
    
    /* There are a RF activity */
    if(RFActivity) {
      RFActivity = 0;
      //Disable Interrupt from MEMS
      HAL_NVIC_DisableIRQ(EXTI0_IRQn);
      
      //Detect RF Activity
      SmarTagAppDetectRFActivity();
      
      //Enable Interrupt from MEMS
      HAL_NVIC_EnableIRQ(EXTI0_IRQn);
      
    }
    
    /* Handle User Button*/
    if(ButtonPressed) {
      static time_t StartTime;
      static uint32_t NumberOfTime=3;
      time_t CurrentTime;
      
      ButtonPressed=0;
      
      if(NumberOfTime==3) {
        CurrentTime = StartTime = STNFC_GetDateTime(&hrtc);
      } else {
        CurrentTime  =STNFC_GetDateTime(&hrtc);
        if((CurrentTime-StartTime)>3) {
          NumberOfTime=3;
          StartTime = CurrentTime;
        }
      }
      NumberOfTime--;
      SMARTAG2_PRINTF("\r\n\tPress Again %d Times in %dSeconds\r\n\tfor Factory Reset\r\n",NumberOfTime,3-((uint32_t) (CurrentTime-StartTime)));
      
      if(NumberOfTime==0) {
         //Disable Interrupt from MEMS
         HAL_NVIC_DisableIRQ(EXTI0_IRQn);
         
         //Deactivate Timer
         if(WakeUpTimerIsSet) {
           if(HAL_RTCEx_DeactivateWakeUpTimer(&hrtc)!= HAL_OK) {
             STNFC_Error_Handler(STNFC_INIT_ERROR);
           }
         }
           
         
        BSP_SmarTag2_EEP_PowerOn();
        SMARTAG2_PRINTF("Power on NFC (VDD EEP On)\r\n");
        /* Rise time required by VDD_EEPROM for NFC */
        HAL_Delay(200);
        BSP_LPD_Off();
        
        BSP_SmarTag2_ACC_PowerOff();
        SMARTAG2_PRINTF("VDD ACC Off\r\n");
        BSP_SmarTag2_AMB_PowerOff();
        SMARTAG2_PRINTF("VDD AMB Off\r\n");
               
        SMARTAG2_PRINTF("\r\n-----------------\r\n"); 
        SMARTAG2_PRINTF("| Factory Reset |\r\n");
        SMARTAG2_PRINTF("-----------------\r\n"); 

        /* Erase NFC Content */
        FactoryReset();
        /* Restart */
        HAL_NVIC_SystemReset();
      }
    }
    
    /* There are a Async events */
    if (MemsInterrupt) {
      MemsInterrupt = 0;
      SmarTagAppDetectMemsEvent();
    }
    

    if(!WakeUpTimerIsSet) {
      if(HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0)!= HAL_OK) {
        STNFC_Error_Handler(STNFC_RUNTIME_ERROR);
      }
      WakeUpTimerIsSet= 1;
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
 * @brief  BSP Push Button callback
 * @param  Button Specifies the pin connected EXTI line
 * @retval None.
 */
void BSP_PB_Callback(Button_TypeDef Button)
{
  ButtonPressed=1;
}


/**
 * @brief  Factory Reset NFC
 * @param  None
 * @retval None.
 */
static void FactoryReset(void) 
{
  uint32_t DataBuf32 =0xFFFFFFFF;
  uint8_t *DataBufPointer = (uint8_t *) &DataBuf32;
  int32_t Counter;
   
  SMARTAG2_PRINTF("Erasing NFC\r\n");
  for(Counter=0;Counter<STSMART_NFC_MAX_SIZE;Counter+=4) {
    SMARTAG2_PRINTF("Remaining %04d\r",STSMART_NFC_MAX_SIZE-Counter);
    BSP_LED_Toggle();
    if(BSP_NFCTAG_WriteData(BSP_NFCTAG_INSTANCE, DataBufPointer, Counter, 4)!=NFCTAG_OK){
      STNFC_Error_Handler(STNFC_WRITING_ERROR);
    }
  }
  SMARTAG2_PRINTF("\r\nNFC Content Erased\r\n");
  BSP_LED_Off();
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    while(1);
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /** Initialize RTC Only */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    STNFC_Error_Handler(STNFC_INIT_ERROR);
  }


  /** Initialize RTC and set the Time and Date */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    STNFC_Error_Handler(STNFC_INIT_ERROR);
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    STNFC_Error_Handler(STNFC_INIT_ERROR);
  }
  /** Enable the WakeUp */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 1, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0) != HAL_OK)
  {
    STNFC_Error_Handler(STNFC_INIT_ERROR);
  }

  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);

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
  HAL_GPIO_Init(INT_PRE_GPIO_Port, &GPIO_InitStruct);

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
  HAL_GPIO_Init(INT_TEM_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : ACC_INT1_Pin */
  GPIO_InitStruct.Pin = ACC_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  //GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ACC_INT1_GPIO_Port, &GPIO_InitStruct);

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
  
//  /*Configure GPIO pins : ACC_INT2_Pin */
//  GPIO_InitStruct.Pin = ACC_INT2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//  //GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(ACC_INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HIGH_G_CS_Pin VDD_SAF_CTRL_Pin VDD_AMB_MCU_Pin VDD_ACC_MCU_Pin
                           LPD_Pin CH_ON_Pin */
  GPIO_InitStruct.Pin = HIGH_G_CS_Pin|VDD_SAF_CTRL_Pin|VDD_AMB_MCU_Pin|VDD_ACC_MCU_Pin
                          |LPD_Pin|CH_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  
  HAL_NVIC_SetPriority(EXTI2_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief  Initializes the Hardware Configuration
  *         Step1: RTC/UART Init and Power Off Inertial Sensors
  * @param  None
  * @retval None
  */
static void HWInitializationStep1(void)
{
  /* Initialize all configured peripherals */
  MX_RTC_Init();
  
#ifdef SMARTAG2_ENABLE_PRINTF
  /* Initialize Virtual COM Port */
  if(BSP_COM_Init(COM1) == BSP_ERROR_NONE) {
    SMARTAG2_PRINTF("UART Initialized\r\n\r\n");
  } else {
    STNFC_Error_Handler(STNFC_INIT_ERROR);
  }
  
  SMARTAG2_PRINTF("STMicroelectronics %s:\r\n"
         "\tVersion %d.%d.%d\r\n"
         "\tSTEVAL_SMARTAG2 board"
         "\r\n",
         "FP-SNS-SMARTAG2",
         SMARTAG2_VERSION_MAJOR,SMARTAG2_VERSION_MINOR,SMARTAG2_VERSION_PATCH);

  SMARTAG2_PRINTF("\t(HAL %ld.%ld.%ld_%ld)\r\n"
        "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n\r\n"
#elif defined (__ARMCC_VERSION)
        " (KEIL)\r\n\r\n"
#elif defined (__GNUC__)
        " (STM32CubeIDE)\r\n\r\n"
#endif
           ,
           HAL_GetHalVersion() >>24,
          (HAL_GetHalVersion() >>16)&0xFF,
          (HAL_GetHalVersion() >> 8)&0xFF,
           HAL_GetHalVersion()      &0xFF,
         __DATE__,__TIME__);
 #endif /* SMARTAG2_ENABLE_PRINTF */ 
  
   BSP_SmarTag2_ACC_PowerOff();
   HAL_NVIC_DisableIRQ(EXTI0_IRQn);
   MemsInterrupt=0;
   AccInit_LIS2DUXS12_Done= 0;
   AccInit_LSM6DSO32X_Done= 0;
}

/**
  * @brief  Initializes the Hardware Configuration
  *         Steps: Initializes Hw not Initialized on Step1
  * @param  None
  * @retval None
  */
void HWInitializationStep2(void)
{
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  
  /* Initialize LED */
  BSP_LED_Init();
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  BSP_GPO_Init();
  
  /* SmarTag Application Start */
  SmarTagAppStart();
}



/**
  * @brief  Callback Function for Wake Up Timer
  * @param  None
  * @retval None
  */
static void WakeUpTimerCallBack(void)
{
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  WakeUpTimerIsSet= 0;

  SmarTagAppWakeUpTimerCallBack();
}

/**
  * @brief  BSP GPO callback
  * @retval None.
  */
void BSP_GPO_Callback(void)
{
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  RFActivity =1;
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
  SMARTAG2_PRINTF("%s@%d:",file,line);
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
  
  switch(ErroCode) {
    case STNFC_RUNTIME_ERROR:
      BSP_LED_On();
      HAL_Delay(100);
      BSP_LED_Off();
      HAL_Delay(500);
    case STNFC_INIT_ERROR:
      BSP_LED_On();
      HAL_Delay(100);
      BSP_LED_Off();
      HAL_Delay(500);
    case STNFC_CONFIG_ERROR:
      BSP_LED_On();
      HAL_Delay(100);
      BSP_LED_Off();
      HAL_Delay(500);
    case STNFC_WRITING_ERROR:
      BSP_LED_On();
      HAL_Delay(100);
      BSP_LED_Off();
      HAL_Delay(500);
    case STNFC_READING_ERROR:
      BSP_LED_On();
      HAL_Delay(100);
      BSP_LED_Off();
      HAL_Delay(500);
  }
}

/**
  * @brief  Wake Up Timer callback.
  * @param  hrtc: RTC handle
  * @retval None
  */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  UNUSED(hrtc);
  WakeUpTimerInterrupt =1;
}

/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin){
  /* Mems event detection */
  case ACC_INT1_Pin:
    MemsInterrupt = 1;
    break;
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

