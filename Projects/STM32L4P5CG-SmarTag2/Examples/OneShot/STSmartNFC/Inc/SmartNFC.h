/**
  ******************************************************************************
  * @file    SmartNFC.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.1.0
  * @date    22-February-2023
  * @brief   Smart NFC protocol 
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMART_NFC_H
#define __SMART_NFC_H

#include "SmarTag2_nfctag.h"
#include "SmartNFCType.h"
#include "SmarTag2_nfctag.h"
#include "SMARTAG2_config.h"

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Exported Defines ----------------------------------------------------------*/
/* 12 for UID (Length + 3 offset + 8 for UID) */
#define SMARTAG2_UID_EXTRA_LENGHT 0xC
   
/* Eval-SmarTag2 includes the st25dv64k */
#define STSMART_NFC_MAX_SIZE NFCTAG_64K_SIZE

/* Dimension of the CC file in bytes */
#define ST25DV_CC_SIZE            8

/* Address due to extended CC file + NDEF header before payload == (4 or 8) + 24 + SMARTAG2_UID_EXTRA_LENGHT*/
#define SMARTAG2_START_ADDR_OFFSET (0x18+ST25DV_CC_SIZE + SMARTAG2_UID_EXTRA_LENGHT)

/* Exported Macros -----------------------------------------------------------*/
#define MCR_STNFC_CompareWithLimits(Type,VirtualSensor,value)\
{\
  /* Limit the Value */\
  if(value>=VirtualSensor.MaxLimit.Type##Value) {\
    value=VirtualSensor.MaxLimit.Type##Value;\
  }\
  \
  if(value<=VirtualSensor.MinLimit.Type##Value) {\
    value=VirtualSensor.MinLimit.Type##Value;\
  }\
  /* Save the Value */\
  VirtualSensor.Sample.Type##Value = value;\
}

/* Exported Variables --------------------------------------------------------*/
extern char *ThresholdsUsageName[4];

#define STNFC_Error_Handler(ErrorCode) STNFC_Error(ErrorCode,__FILE__, __LINE__)

/* Exported Prototypes -------------------------------------------------------*/
extern void STNFC_Error(SNFC_ErrorCode_t ErroCode, char *file, int32_t line);
extern void InitSTSmartNFC(void);

extern void STNFC_ComputeMaxMinCompareTHsUi8t(SNFC_VirtualSensor_t *VirtualSensor,SNFC_LogDefinition_t *LogDefinition);
extern void STNFC_ComputeMaxMinCompareTHsUi16t(SNFC_VirtualSensor_t *VirtualSensor,SNFC_LogDefinition_t *LogDefinition);
extern void STNFC_ComputeMaxMinCompareTHsUi32t(SNFC_VirtualSensor_t *VirtualSensor,SNFC_LogDefinition_t *LogDefinition);


#ifdef __cplusplus
}
#endif

#endif /* __SMART_NFC_H */

