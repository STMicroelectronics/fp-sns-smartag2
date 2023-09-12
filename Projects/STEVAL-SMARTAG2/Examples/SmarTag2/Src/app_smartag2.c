/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_smartag2.c
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
#include "app_smartag2.h"

#include "AppSmarTag.h"
#include "SmartNFC.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

/* USER CODE END PD*/

/* Exported Variables --------------------------------------------------------*/
uint8_t AccInit_LIS2DUXS12_Done=0;
uint8_t AccInit_LSM6DSO32X_Done=0;
volatile uint8_t MemsInterrupt = 0;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private variables ---------------------------------------------------------*/
/* Interrupt from Timer */
static volatile uint32_t WakeUpTimerInterrupt =0;
/* Interrupt from User Button*/
static volatile uint32_t ButtonPressed = 0;

/* There is one RF Activity? */
static volatile uint32_t RFActivity = 0;

/* Identify if the WakeUp Timer is set  */
static uint32_t WakeUpTimerIsSet =0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void User_Init(void);
static void User_Process(void);
static void HWInitializationStep1(void);
static void WakeUpTimerCallBack(void);
static void FactoryReset(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

void MX_SmarTag2_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN SmarTag2_Init_PreTreatment */

  /* USER CODE END SmarTag2_Init_PreTreatment */

  /* Initialize SMARTAG2 application */

  User_Init();

  /* USER CODE BEGIN SmarTag2_Init_PostTreatment */

  /* USER CODE END SmarTag2_Init_PostTreatment */
}

/*
 * FP-SNS-SMARTAG2 background task
 */
void MX_SmarTag2_Process(void)
{
  /* USER CODE BEGIN SmarTag2_Process_PreTreatment */

  /* USER CODE END SmarTag2_Process_PreTreatment */

  /* Process of the SMARTAG2 application */

  User_Process();

  /* USER CODE BEGIN SmarTag2_Process_PostTreatment */

  /* USER CODE END SmarTag2_Process_PostTreatment */
}

/**
 * @brief  Initialize User process.
 * @param  None
 * @retval None
 */
static void User_Init(void)
{
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
    if(HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0) != HAL_OK)
    {
      STNFC_Error_Handler(STNFC_INIT_ERROR);
    }

    WakeUpTimerIsSet= 1;
  }

#ifndef SMARTAG2_ENABLE_DEBUG
  /* -> Stop mode */
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
#endif /* SMARTAG2_ENABLE_DEBUG */

  SMARTAG2_PRINTF("Wait %d sec before autoStart\r\n\r\n", SMARTAG2_AUTOSTART_SECONDS);
}

/**
 * @brief  Configure the device as Client or Server and manage the communication
 *         between a client and a server.
 *
 * @param  None
 * @retval None
 */
static void User_Process(void)
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
    SMARTAG2_PRINTF("\r\n\tPress Again %ld Times in %ldSeconds\r\n\tfor Factory Reset\r\n",NumberOfTime,3-((uint32_t) (CurrentTime-StartTime)));

    if(NumberOfTime==0) {
       //Disable Interrupt from MEMS
       HAL_NVIC_DisableIRQ(EXTI0_IRQn);

       //Deactivate Timer
       if(WakeUpTimerIsSet) {
         if(HAL_RTCEx_DeactivateWakeUpTimer(&hrtc)!= HAL_OK) {
           STNFC_Error_Handler(STNFC_INIT_ERROR);
         }
       }

      BSP_NFCTAG_EEP_PowerOn();
      SMARTAG2_PRINTF("Power on NFC (VDD EEP On)\r\n");
      /* Rise time required by VDD_EEPROM for NFC */
      HAL_Delay(200);
      BSP_NFCTAG_LPD_Off();

      BSP_MOTION_SENSOR_PowerOff();
      SMARTAG2_PRINTF("VDD ACC Off\r\n");
      BSP_ENV_SENSOR_PowerOff();
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
    if(HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0) != HAL_OK)
    {
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
    SMARTAG2_PRINTF("Remaining %04ld\r",STSMART_NFC_MAX_SIZE-Counter);
    BSP_LED_Toggle(LED2);
    if(BSP_NFCTAG_WriteData(BSP_NFCTAG_INSTANCE, DataBufPointer, Counter, 4)!=NFCTAG_OK){
      STNFC_Error_Handler(STNFC_WRITING_ERROR);
    }
  }
  SMARTAG2_PRINTF("\r\nNFC Content Erased\r\n");
  BSP_LED_Off(LED2);
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
  * @brief  Initializes the Hardware Configuration
  *         Step1: RTC/UART Init and Power Off Inertial Sensors
  * @param  None
  * @retval None
  */
static void HWInitializationStep1(void)
{
  /* Initialize all configured peripherals */
  Init_RTC();

  /* Set EXTI settings for Accelerometer Interrupt */
  SetAccIntPin_exti();

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

   BSP_MOTION_SENSOR_PowerOff();
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
  BSP_LED_Init(LED2);
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  BSP_NFCTAG_GPO_Init();

  /* SmarTag Application Start */
  SmarTagAppStart();
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
  SMARTAG2_PRINTF("%s@%ld:",file,line);
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
      BSP_LED_On(LED2);
      HAL_Delay(100);
      BSP_LED_Off(LED2);
      HAL_Delay(500);
    case STNFC_INIT_ERROR:
      BSP_LED_On(LED2);
      HAL_Delay(100);
      BSP_LED_Off(LED2);
      HAL_Delay(500);
    case STNFC_CONFIG_ERROR:
      BSP_LED_On(LED2);
      HAL_Delay(100);
      BSP_LED_Off(LED2);
      HAL_Delay(500);
    case STNFC_WRITING_ERROR:
      BSP_LED_On(LED2);
      HAL_Delay(100);
      BSP_LED_Off(LED2);
      HAL_Delay(500);
    case STNFC_READING_ERROR:
      BSP_LED_On(LED2);
      HAL_Delay(100);
      BSP_LED_Off(LED2);
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

