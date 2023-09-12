/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    AppOneShot.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.2.0
  * @date    30-May-2023
  * @brief   Header for AppOneShot.c file.
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

extern void OneShotAppStart(void);
extern void MEMS_Sensors_ReadData(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_SMARTAG_H */
