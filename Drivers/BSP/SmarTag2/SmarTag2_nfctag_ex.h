/**
  ******************************************************************************
  * @file    SmarTag2_nfctag_ex.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.0.0
  * @date    31-August-2022
  * @brief   This file contains definitions for the SmarTag2_nfctag_ex.c
  *          specific functions.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMARTAG2_NFCTAG_EX_H
#define __SMARTAG2_NFCTAG_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "st25dvxxkc.h"

/* Exported Prototypes -------------------------------------------------------*/
int32_t BSP_NFCTAG_LPD_Init( void );
int32_t BSP_NFCTAG_LPD_DeInit( void );
int32_t BSP_NFCTAG_LPD_ReadPin( void );
int32_t BSP_NFCTAG_LPD_On( void );
int32_t BSP_NFCTAG_LPD_Off( void );
int32_t BSP_NFCTAG_LPD_Toggle( void );

int32_t BSP_NFCTAG_GPO_Init( void );
int32_t BSP_NFCTAG_GPO_DeInit( void );
int32_t BSP_NFCTAG_GPO_ReadPin( void );
void BSP_GPO_IRQHandler(void);

extern int32_t BSP_NFCTAG_ChangeI2CPassword(uint32_t MsbPasswd,uint32_t LsbPasswd);
extern int32_t BSP_NFCTAG_SetICPasswordProtectionZone1(uint32_t MsbPasswd,uint32_t LsbPasswd,ST25DVxxKC_PROTECTION_CONF_E ProtectionLevel);
extern int32_t BSP_NFCTAG_EnableRFWritingPasswordProtectionZone1(uint32_t MsbPasswd,uint32_t LsbPasswd);
extern int32_t BSP_NFCTAG_DisableRFWritingPasswordProtectionZone1(uint32_t MsbPasswd,uint32_t LsbPasswd);
extern int32_t BSP_NFCTAG_CheckChangeEHMODE(uint32_t MsbPasswd,uint32_t LsbPasswd,ST25DVxxKC_EH_MODE_STATUS_E NewEHMode);
extern int32_t BSP_NFCTAG_CheckEHMODE(ST25DVxxKC_EH_MODE_STATUS_E *EHMode);
extern int32_t BSP_NFCTAG_ChangeEHMODE(uint32_t MsbPasswd,uint32_t LsbPasswd,ST25DVxxKC_EH_MODE_STATUS_E NewEHMode);
extern int32_t BSP_NFCTAG_WriteConfigIT(uint32_t MsbPasswd,uint32_t LsbPasswd,const uint16_t ITConfig);
extern int32_t BSP_NFCTAG_ChangeMBMode(uint32_t MsbPasswd,uint32_t LsbPasswd,const ST25DVxxKC_EN_STATUS_E MB_mode);
extern int32_t BSP_NFCTAG_ChangeMBWDG(uint32_t MsbPasswd,uint32_t LsbPasswd,const uint8_t WdgDelay);
extern int32_t BSP_NFCTAG_ChangeITPulse(uint32_t MsbPasswd,uint32_t LsbPasswd,const ST25DVxxKC_PULSE_DURATION_E ITtime);

#ifdef __cplusplus
}
#endif

#endif /* __SMARTAG2_NFCTAG_EX_H */


