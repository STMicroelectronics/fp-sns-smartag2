/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.2.0
  * @date    30-May-2023
  * @brief   Header for main.c file.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

#include "SmarTag2_conf.h"
#include "steval_smartag2.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_CRC_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define INT_PRE_Pin GPIO_PIN_13
#define INT_PRE_GPIO_Port GPIOC
#define INT_PRE_EXTI_IRQn EXTI15_10_IRQn
#define GPO_Pin GPIO_PIN_14
#define GPO_GPIO_Port GPIOC
#define GPO_EXTI_IRQn EXTI15_10_IRQn
#define ISET0_Pin GPIO_PIN_15
#define ISET0_GPIO_Port GPIOC
#define IMU_CS_Pin GPIO_PIN_0
#define IMU_CS_GPIO_Port GPIOH
#define IRQ_RTC_Pin GPIO_PIN_1
#define IRQ_RTC_GPIO_Port GPIOH
#define ACC_INT1_Pin GPIO_PIN_0
#define ACC_INT1_GPIO_Port GPIOA
#define RST_SAFE_Pin GPIO_PIN_1
#define RST_SAFE_GPIO_Port GPIOA
#define HIGH_G_INT1_Pin GPIO_PIN_2
#define HIGH_G_INT1_GPIO_Port GPIOA
#define VDD_EEP_Pin GPIO_PIN_3
#define VDD_EEP_GPIO_Port GPIOA
#define INT_TEM_Pin GPIO_PIN_4
#define INT_TEM_GPIO_Port GPIOA
#define RECT_M_Pin GPIO_PIN_5
#define RECT_M_GPIO_Port GPIOA
#define BATT_M_Pin GPIO_PIN_6
#define BATT_M_GPIO_Port GPIOA
#define ACC_INT2_Pin GPIO_PIN_0
#define ACC_INT2_GPIO_Port GPIOB
#define HIGH_G_CS_Pin GPIO_PIN_1
#define HIGH_G_CS_GPIO_Port GPIOB
#define BSP_BUTTON_EXTI_IRQn EXTI2_IRQn
#define VDD_SAF_CTRL_Pin GPIO_PIN_13
#define VDD_SAF_CTRL_GPIO_Port GPIOB
#define VDD_AMB_MCU_Pin GPIO_PIN_14
#define VDD_AMB_MCU_GPIO_Port GPIOB
#define VDD_ACC_MCU_Pin GPIO_PIN_15
#define VDD_ACC_MCU_GPIO_Port GPIOB
#define ACC_CS_Pin GPIO_PIN_15
#define ACC_CS_GPIO_Port GPIOA
#define GPIO1_Pin GPIO_PIN_5
#define GPIO1_GPIO_Port GPIOB
#define LPD_Pin GPIO_PIN_8
#define LPD_GPIO_Port GPIOB
#define CH_ON_Pin GPIO_PIN_9
#define CH_ON_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
