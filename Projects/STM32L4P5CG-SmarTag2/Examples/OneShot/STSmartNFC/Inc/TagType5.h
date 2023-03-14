/**
  ******************************************************************************
  * @file    TagType5.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.1.0
  * @date    22-February-2023
  * @brief   Minimal NDEF header APIs
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
#ifndef __TAGTYPE5_H
#define __TAGTYPE5_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported Functions --------------------------------------------------------*/
extern void NfcType5_NDEFInitHeader(void);
extern void NfcType5_NDEFUpdateHeaderAddTermTLV(uint32_t BytesAdded);
extern void NfcType5_ComputeNDEFPayLoadSize(uint32_t LastSamplePointer,uint32_t SampleCounter);
extern void NfcType5_SetInitialNDEFPayLoadLengthValue(uint32_t Address);
extern void NfcType5_UpdateSampleCounter(SNFC_LogDefinition_t *LogDef,uint32_t BytesAdded);

#ifdef __cplusplus
}
#endif

#endif /* __TAGTYPE5_H */

