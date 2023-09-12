/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    AppSmarTag.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.2.0
  * @date    30-May-2023
  * @brief   Header for AppSmarTag.c file.
  *          This file contains the common defines of the application.
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
#ifndef __APP_SMARTAG_H
#define __APP_SMARTAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

#include "SmarTag2_nfctag_ex.h"

extern RTC_HandleTypeDef hrtc;

/* Exported variables --------------------------------------------------------*/
extern EXTI_HandleTypeDef hexti0;
#define H_EXTI_0 hexti0
extern EXTI_HandleTypeDef hexti4;
#define H_EXTI_4 hexti4
extern EXTI_HandleTypeDef hexti13;
#define H_EXTI_13 hexti13

extern void SmarTagAppStart(void);
extern void SmarTagAppWakeUpTimerCallBack(void);
extern void SmarTagAppProcess(void);
extern void SmarTagAppDetectMemsEvent(void);
extern void SmarTagAppDetectRFActivity(void);
extern void Init_RTC(void);
extern void SetAccIntPin_exti(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_SMARTAG_H */
