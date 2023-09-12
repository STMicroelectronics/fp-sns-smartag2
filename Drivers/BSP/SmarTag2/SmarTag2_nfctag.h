/**
  ******************************************************************************
  * @file    SmarTag2_nfctag.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.0.0
  * @date    31-August-2022
  * @brief   This file contains definitions for the bsp SmarTag2_nfctag.c
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
#ifndef __SMARTAG2_NFCTAG_H
#define __SMARTAG2_NFCTAG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "SmarTag2_conf.h"   
#include "st25dvxxkc.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup SMARTAG2
  * @{
  */
   
/** @addtogroup SMARTAG2_NFCTAG
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/**
 * @brief  NFCTAG status enumerator definition.
 */


/* Exported constants --------------------------------------------------------*/
#define NFCTAG_64K_SIZE           ((uint32_t) 0x2000)
/* External variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported function	--------------------------------------------------------*/
/** @defgroup SMARTAG2_NFCTAG_Exported_Functions
  * @{
  */

void BSP_NFCTAG_EEP_PowerOff(void);
void BSP_NFCTAG_EEP_PowerOn(void);

int32_t BSP_NFCTAG_Init( uint32_t Instance );
void BSP_NFCTAG_DeInit( uint32_t Instance );
int32_t BSP_NFCTAG_isInitialized( uint32_t Instance );
int32_t BSP_NFCTAG_ReadID( uint32_t Instance, uint8_t * const wai_id );
int32_t BSP_NFCTAG_ConfigIT( uint32_t Instance, const uint16_t ITConfig );
int32_t BSP_NFCTAG_GetITStatus( uint32_t Instance, uint16_t * const ITConfig );
int32_t BSP_NFCTAG_ReadData( uint32_t Instance, uint8_t * const pData, const uint16_t TarAddr, const uint16_t Size );
int32_t BSP_NFCTAG_WriteData( uint32_t Instance, const uint8_t * const pData, const uint16_t TarAddr, const uint16_t Size );
int32_t BSP_NFCTAG_ReadRegister( uint32_t Instance, uint8_t * const pData, const uint16_t TarAddr, const uint16_t Size );
int32_t BSP_NFCTAG_WriteRegister( uint32_t Instance, const uint8_t * const pData, const uint16_t TarAddr, const uint16_t Size );
int32_t BSP_NFCTAG_IsDeviceReady( uint32_t Instance,const uint32_t Trials );

uint32_t BSP_NFCTAG_GetByteSize( uint32_t Instance );
int32_t BSP_NFCTAG_ReadICRev( uint32_t Instance, uint8_t * const pICRev );
int32_t BSP_NFCTAG_ReadITPulse( uint32_t Instance, ST25DVxxKC_PULSE_DURATION_E * const pITtime );
int32_t BSP_NFCTAG_WriteITPulse( uint32_t Instance, const ST25DVxxKC_PULSE_DURATION_E ITtime );
int32_t BSP_NFCTAG_ReadUID( uint32_t Instance, ST25DVxxKC_UID_t * const pUid );
int32_t BSP_NFCTAG_ReadDSFID( uint32_t Instance, uint8_t * const pDsfid );
int32_t BSP_NFCTAG_ReadDsfidRFProtection( uint32_t Instance, ST25DVxxKC_LOCK_STATUS_E * const pLockDsfid );
int32_t BSP_NFCTAG_ReadAFI( uint32_t Instance, uint8_t * const pAfi );
int32_t BSP_NFCTAG_ReadAfiRFProtection( uint32_t Instance, ST25DVxxKC_LOCK_STATUS_E * const pLockAfi );
int32_t BSP_NFCTAG_ReadI2CProtectZone( uint32_t Instance, ST25DVxxKC_I2C_PROT_ZONE_t * const pProtZone );
int32_t BSP_NFCTAG_WriteI2CProtectZonex(uint32_t Instance, const ST25DVxxKC_PROTECTION_ZONE_E Zone,  const ST25DVxxKC_PROTECTION_CONF_E ReadWriteProtection );
int32_t BSP_NFCTAG_ReadLockCCFile(uint32_t Instance, ST25DVxxKC_LOCK_CCFILE_t * const pLockCCFile );
int32_t BSP_NFCTAG_WriteLockCCFile(uint32_t Instance, const ST25DVxxKC_CCFILE_BLOCK_E NbBlockCCFile,  const ST25DVxxKC_LOCK_STATUS_E LockCCFile );
int32_t BSP_NFCTAG_ReadLockCFG(uint32_t Instance, ST25DVxxKC_LOCK_STATUS_E * const pLockCfg );
int32_t BSP_NFCTAG_WriteLockCFG(uint32_t Instance, const ST25DVxxKC_LOCK_STATUS_E LockCfg );
int32_t BSP_NFCTAG_PresentI2CPassword(uint32_t Instance, const ST25DVxxKC_PASSWD_t PassWord );
int32_t BSP_NFCTAG_WriteI2CPassword(uint32_t Instance, const ST25DVxxKC_PASSWD_t PassWord );
int32_t BSP_NFCTAG_ReadRFZxSS(uint32_t Instance, const ST25DVxxKC_PROTECTION_ZONE_E Zone,  ST25DVxxKC_RF_PROT_ZONE_t * const pRfprotZone );
int32_t BSP_NFCTAG_WriteRFZxSS(uint32_t Instance, const ST25DVxxKC_PROTECTION_ZONE_E Zone,  const ST25DVxxKC_RF_PROT_ZONE_t RfProtZone );
int32_t BSP_NFCTAG_ReadEndZonex(uint32_t Instance, const ST25DVxxKC_END_ZONE_E EndZone,  uint8_t * const pEndZ );
int32_t BSP_NFCTAG_WriteEndZonex(uint32_t Instance, const ST25DVxxKC_END_ZONE_E EndZone,  const uint8_t EndZ );
int32_t BSP_NFCTAG_InitEndZone(uint32_t Instance);
int32_t BSP_NFCTAG_CreateUserZone(uint32_t Instance, uint16_t Zone1Length,  uint16_t Zone2Length,  uint16_t Zone3Length,  uint16_t Zone4Length );
int32_t BSP_NFCTAG_ReadMemSize(uint32_t Instance, ST25DVxxKC_MEM_SIZE_t * const pSizeInfo );
int32_t BSP_NFCTAG_ReadEHMode(uint32_t Instance, ST25DVxxKC_EH_MODE_STATUS_E * const pEH_mode );
int32_t BSP_NFCTAG_WriteEHMode(uint32_t Instance, const ST25DVxxKC_EH_MODE_STATUS_E EH_mode );
int32_t BSP_NFCTAG_ReadRFMngt(uint32_t Instance, ST25DVxxKC_RF_MNGT_t * const pRF_Mngt );
int32_t BSP_NFCTAG_WriteRFMngt(uint32_t Instance, const uint8_t Rfmngt );
int32_t BSP_NFCTAG_GetRFDisable(uint32_t Instance, ST25DVxxKC_EN_STATUS_E * const pRFDisable );
int32_t BSP_NFCTAG_SetRFDisable(uint32_t Instance);
int32_t BSP_NFCTAG_ResetRFDisable(uint32_t Instance);
int32_t BSP_NFCTAG_GetRFSleep(uint32_t Instance, ST25DVxxKC_EN_STATUS_E * const pRFSleep );
int32_t BSP_NFCTAG_SetRFSleep(uint32_t Instance);
int32_t BSP_NFCTAG_ResetRFSleep(uint32_t Instance);
int32_t BSP_NFCTAG_ReadMBMode(uint32_t Instance, ST25DVxxKC_EN_STATUS_E * const pMB_mode );
int32_t BSP_NFCTAG_WriteMBMode(uint32_t Instance, const ST25DVxxKC_EN_STATUS_E MB_mode );
int32_t BSP_NFCTAG_ReadMBWDG(uint32_t Instance, uint8_t * const pWdgDelay );
int32_t BSP_NFCTAG_WriteMBWDG(uint32_t Instance, const uint8_t WdgDelay );
int32_t BSP_NFCTAG_ReadMailboxData(uint32_t Instance, uint8_t * const pData,  const uint16_t TarAddr,  const uint16_t NbByte );
int32_t BSP_NFCTAG_WriteMailboxData(uint32_t Instance, const uint8_t * const pData,  const uint16_t NbByte );
int32_t BSP_NFCTAG_ReadMailboxRegister(uint32_t Instance, uint8_t * const pData,  const uint16_t TarAddr,  const uint16_t NbByte );
int32_t BSP_NFCTAG_WriteMailboxRegister(uint32_t Instance, const uint8_t * const pData,  const uint16_t TarAddr,  const uint16_t NbByte );
int32_t BSP_NFCTAG_ReadI2CSecuritySession_Dyn(uint32_t Instance, ST25DVxxKC_I2CSSO_STATUS_E * const pSession );
int32_t BSP_NFCTAG_ReadITSTStatus_Dyn(uint32_t Instance, uint8_t * const pITStatus );
int32_t BSP_NFCTAG_ReadGPO_Dyn(uint32_t Instance, uint8_t *GPOConfig );
int32_t BSP_NFCTAG_GetGPO_en_Dyn(uint32_t Instance, ST25DVxxKC_EN_STATUS_E * const pGPO_en );
int32_t BSP_NFCTAG_SetGPO_en_Dyn(uint32_t Instance);
int32_t BSP_NFCTAG_ResetGPO_en_Dyn(uint32_t Instance);
int32_t BSP_NFCTAG_ReadEHCtrl_Dyn(uint32_t Instance, ST25DVxxKC_EH_CTRL_t * const pEH_CTRL );
int32_t BSP_NFCTAG_GetEHENMode_Dyn(uint32_t Instance, ST25DVxxKC_EN_STATUS_E * const pEH_Val );
int32_t BSP_NFCTAG_SetEHENMode_Dyn(uint32_t Instance);
int32_t BSP_NFCTAG_ResetEHENMode_Dyn(uint32_t Instance);
int32_t BSP_NFCTAG_GetEHON_Dyn(uint32_t Instance, ST25DVxxKC_EN_STATUS_E * const pEHON );
int32_t BSP_NFCTAG_GetRFField_Dyn(uint32_t Instance, ST25DVxxKC_FIELD_STATUS_E * const pRF_Field );
int32_t BSP_NFCTAG_GetVCC_Dyn(uint32_t Instance, ST25DVxxKC_VCC_STATUS_E * const pVCC );
int32_t BSP_NFCTAG_ReadRFMngt_Dyn(uint32_t Instance, ST25DVxxKC_RF_MNGT_t * const pRF_Mngt );
int32_t BSP_NFCTAG_WriteRFMngt_Dyn(uint32_t Instance, const uint8_t RF_Mngt );
int32_t BSP_NFCTAG_GetRFDisable_Dyn(uint32_t Instance, ST25DVxxKC_EN_STATUS_E * const pRFDisable );
int32_t BSP_NFCTAG_SetRFDisable_Dyn(uint32_t Instance);
int32_t BSP_NFCTAG_ResetRFDisable_Dyn(uint32_t Instance);
int32_t BSP_NFCTAG_GetRFSleep_Dyn(uint32_t Instance, ST25DVxxKC_EN_STATUS_E * const pRFSleep );
int32_t BSP_NFCTAG_SetRFSleep_Dyn(uint32_t Instance);
int32_t BSP_NFCTAG_ResetRFSleep_Dyn(uint32_t Instance);
int32_t BSP_NFCTAG_ReadMBCtrl_Dyn(uint32_t Instance, ST25DVxxKC_MB_CTRL_DYN_STATUS_t * const pCtrlStatus );
int32_t BSP_NFCTAG_GetMBEN_Dyn(uint32_t Instance, ST25DVxxKC_EN_STATUS_E * const pMBEN );
int32_t BSP_NFCTAG_SetMBEN_Dyn(uint32_t Instance);
int32_t BSP_NFCTAG_ResetMBEN_Dyn(uint32_t Instance);
int32_t BSP_NFCTAG_ReadMBLength_Dyn(uint32_t Instance, uint8_t * const pMBLength );
int32_t BSP_NFCTAG_GetTick(void);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */ 

#endif /* __SMARTAG2_NFCTAG_H */

