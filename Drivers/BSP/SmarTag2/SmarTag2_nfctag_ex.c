/**
  ******************************************************************************
  * @file    SmarTag2_nfctag_ex.c
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.0.0
  * @date    31-August-2022
  * @brief   This file provides set of driver functions for using the NFC sensor
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
  
/* Includes ------------------------------------------------------------------*/
#include "SmarTag2_nfctag.h"
#include "SmarTag2_nfctag_ex.h"

/** @addtogroup BSP
 * @{
 */

/** @addtogroup SMARTAG2
 * @{
 */

/** @defgroup SMARTAG2_NFCTAG_EX
 * @{
 */
 
__weak void BSP_GPO_Callback(void);

EXTI_HandleTypeDef GPO_EXTI={.Line=BSP_GPO_EXTI_LINE};

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
 
 /* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/ 
/* Global variables ----------------------------------------------------------*/

/** @defgroup SMARTAG2_NFCTAG_EX_Private_Variables
 * @{
 */
//static NFCTAG_DrvTypeDef *Nfctag_Drv = NULL;
//static ST25DVxxKC_Object_t NfcTagObj;

/**
 * @}
 */
 
/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/

/** @defgroup SMARTAG2_NFCTAG_EX_Public_Functions
 * @{
 */

/**
  * @brief  This function initialize the GPIO to manage the NFCTAG LPD pin
  * @param  None
  * @return Status
  */
int32_t BSP_NFCTAG_LPD_Init( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin   = BSP_LPD_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init( BSP_LPD_GPIO_Port, &GPIO_InitStruct );

  HAL_GPIO_WritePin( BSP_LPD_GPIO_Port, BSP_LPD_Pin, GPIO_PIN_RESET );

  return BSP_ERROR_NONE;
}

/**
  * @brief  DeInit LPD.
  * @param  None.
  * @return Status
  * @note LPD DeInit does not disable the GPIO clock nor disable the Mfx
  */
int32_t BSP_NFCTAG_LPD_DeInit( void )
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* DeInit the GPIO_GPO pin */
  gpio_init_structure.Pin = BSP_LPD_Pin;
  HAL_GPIO_DeInit( BSP_LPD_GPIO_Port, gpio_init_structure.Pin);

  return BSP_ERROR_NONE;
}

/**
  * @brief  This function get the LPD state
  * @param  None
  * @retval GPIO pin status
  */
int32_t BSP_NFCTAG_LPD_ReadPin( void )
{
  return (int32_t)HAL_GPIO_ReadPin( BSP_LPD_GPIO_Port, BSP_LPD_Pin );
}

/**
  * @brief  This function sets the LPD GPIO
  * @param  None
  * @return Status
  */
int32_t BSP_NFCTAG_LPD_On( void )
{
  HAL_GPIO_WritePin( BSP_LPD_GPIO_Port, BSP_LPD_Pin, GPIO_PIN_SET );

  return BSP_ERROR_NONE;

}

/**
  * @brief  This function resets the LPD GPIO
  * @param  None
  * @return Status
  */
int32_t BSP_NFCTAG_LPD_Off( void )
{
  HAL_GPIO_WritePin( BSP_LPD_GPIO_Port, BSP_LPD_Pin, GPIO_PIN_RESET );

  return BSP_ERROR_NONE;

}

/**
  * @brief  This function toggles the LPD GPIO
  * @param  None
  * @return Status
  */
int32_t BSP_NFCTAG_LPD_Toggle( void )
{
  HAL_GPIO_TogglePin( BSP_LPD_GPIO_Port, BSP_LPD_Pin );

  return BSP_ERROR_NONE;

}

/**
  * @brief  This function initialize the GPIO to manage the NFCTAG GPO pin
  * @param  None
  * @return Status
  */
int32_t BSP_NFCTAG_GPO_Init( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin   = BSP_GPO_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init( BSP_GPO_GPIO_Port, &GPIO_InitStruct );

  (void)HAL_EXTI_GetHandle(&GPO_EXTI, BSP_GPO_EXTI_LINE);
  (void)HAL_EXTI_RegisterCallback(&GPO_EXTI,  HAL_EXTI_COMMON_CB_ID, BSP_GPO_Callback);

  /* Enable interruption */
  HAL_NVIC_SetPriority( BSP_GPO_EXTI_IRQn, BSP_NFCTAG_GPO_PRIORITY, 0 );
  HAL_NVIC_EnableIRQ( BSP_GPO_EXTI_IRQn );

  return BSP_ERROR_NONE;
}

/**
  * @brief  DeInit GPO.
  * @param  None.
  * @return Status
  * @note GPO DeInit does not disable the GPIO clock nor disable the Mfx
  */
int32_t BSP_NFCTAG_GPO_DeInit( void )
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* DeInit the GPIO_GPO pin */
  gpio_init_structure.Pin = BSP_GPO_Pin;
  HAL_GPIO_DeInit( BSP_GPO_GPIO_Port, gpio_init_structure.Pin);

  return BSP_ERROR_NONE;

}

/**
  * @brief  This function get the GPO value through GPIO
  * @param  None
  * @retval GPIO pin status
  */
int32_t BSP_NFCTAG_GPO_ReadPin( void )
{
  return (int32_t)HAL_GPIO_ReadPin( BSP_GPO_GPIO_Port, BSP_GPO_Pin );
}

/**
  * @brief  BSP GPO callback
  * @retval None.
  */
__weak void BSP_GPO_Callback(void)
{
  /* Prevent unused argument(s) compilation warning */

  /* This function should be implemented by the user application.
     It is called into this driver when an event on Button is triggered. */
}

void BSP_GPO_IRQHandler(void)
{
  HAL_EXTI_IRQHandler(&GPO_EXTI);
}

/**
  * @brief  Change the I2C Password protection
  * @param  uint32_t MsbPasswd MSB 32-bit password
  * @param  uint32_t LsbPasswd LSB 32-bit password
  * @retval int32_t enum status
  */
int32_t BSP_NFCTAG_ChangeI2CPassword(uint32_t MsbPasswd,uint32_t LsbPasswd)
{
  int32_t ret = NFCTAG_OK;
  ST25DVxxKC_I2CSSO_STATUS_E i2csso;
  ST25DVxxKC_PASSWD_t Passwd;
  BSP_NFCTAG_ReadI2CSecuritySession_Dyn( BSP_NFCTAG_INSTANCE, &i2csso );
  if( i2csso == ST25DVXXKC_SESSION_CLOSED ) {
    /* if I2C session is closed, present default password to open session */
    Passwd.MsbPasswd = 0;
    Passwd.LsbPasswd = 0;
    ret = BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );
    if(ret==NFCTAG_OK) {
      /* Ok we could Change the default Password */
      Passwd.MsbPasswd = MsbPasswd;
      Passwd.LsbPasswd = LsbPasswd;
      ret = BSP_NFCTAG_WriteI2CPassword(BSP_NFCTAG_INSTANCE, Passwd);
      if(ret==NFCTAG_OK) {
        /* Present a wrong password for closing the session we have alredy setted the new one here */
        Passwd.MsbPasswd = ~MsbPasswd;
        Passwd.LsbPasswd = ~LsbPasswd;
        BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );
      }
    }
  }
  return ret;
}

/**
  * @brief  Set the I2C protection level creating a Single secure zone
  * @param  uint32_t MsbPasswd MSB 25-bit password
  * @param  uint32_t LsbPasswd LSB 25-bit password
  * ST25DVxxKC_PROTECTION_CONF_E ProtectionLevel
  * @retval int32_t enum status
  */
int32_t BSP_NFCTAG_SetICPasswordProtectionZone1(uint32_t MsbPasswd,uint32_t LsbPasswd,ST25DVxxKC_PROTECTION_CONF_E ProtectionLevel)
{
  int32_t ret = NFCTAG_OK;
  ST25DVxxKC_I2C_PROT_ZONE_t pProtZone;
  /* Read the Protection levels */
  ret = BSP_NFCTAG_ReadI2CProtectZone(BSP_NFCTAG_INSTANCE, &pProtZone);
  if(ret==NFCTAG_OK) {
    /* Check if the Protect Zone 1 is already on Read and Write Protection */
    if(pProtZone.ProtectZone1!=ProtectionLevel) {
      ST25DVxxKC_I2CSSO_STATUS_E I2CSS;

      /* Read the Session status */
      BSP_NFCTAG_ReadI2CSecuritySession_Dyn( BSP_NFCTAG_INSTANCE, &I2CSS );
      if( I2CSS == ST25DVXXKC_SESSION_CLOSED ) {
        ST25DVxxKC_PASSWD_t Passwd;
        /* if I2C session is closed, present password to open session */
        Passwd.MsbPasswd = MsbPasswd;
        Passwd.LsbPasswd = LsbPasswd;
        ret = BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );

        if(ret==NFCTAG_OK) {
          ST25DVxxKC_MEM_SIZE_t st25dvmemsize;
          uint16_t st25dvbmsize;

          BSP_NFCTAG_ReadMemSize( BSP_NFCTAG_INSTANCE, &st25dvmemsize );
          /* st25dvmemsize is composed of Mem_Size (number of blocks) and BlockSize (size of each blocks in bytes) */
          st25dvbmsize = (st25dvmemsize.Mem_Size + 1U) * (((uint16_t)st25dvmemsize.BlockSize) + 1U);

          /* We create one Memory size == to the whole memory size */
          ret = (int32_t)BSP_NFCTAG_CreateUserZone( BSP_NFCTAG_INSTANCE, st25dvbmsize, 0, 0, 0 );

          if(ret==NFCTAG_OK) {
            /* Set Protection leve for zone 1 for i2c  */
            ret = BSP_NFCTAG_WriteI2CProtectZonex( BSP_NFCTAG_INSTANCE, ST25DVXXKC_PROT_ZONE1, ProtectionLevel );
          }
          /* Present a wrong Password for closing the session */
          Passwd.MsbPasswd = ~MsbPasswd;
          Passwd.LsbPasswd = ~LsbPasswd;
          BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );
        }
      }
    }
  }
  return ret;
}

/**
  * @brief  Enable the R/F writing protection for Zone 1 using RF_PWD_1 password
  * @param  uint32_t MsbPasswd MSB 25-bit password
  * @param  uint32_t LsbPasswd LSB 25-bit password
  * @retval int32_t enum status
  */
int32_t BSP_NFCTAG_EnableRFWritingPasswordProtectionZone1(uint32_t MsbPasswd,uint32_t LsbPasswd)
{
  int32_t ret = NFCTAG_OK;
  ST25DVxxKC_RF_PROT_ZONE_t pRfprotZone;
  /* Read Level of R/F protection */
  ret = BSP_NFCTAG_ReadRFZxSS(BSP_NFCTAG_INSTANCE, ST25DVXXKC_PROT_ZONE1, &pRfprotZone );
  if(ret==NFCTAG_OK) {
    /* Change the R/F protection level if it's necessary */
    if((pRfprotZone.PasswdCtrl != ST25DVXXKC_PROT_PASSWD1 ) && (pRfprotZone.RWprotection != ST25DVXXKC_WRITE_PROT)) {
      ST25DVxxKC_PASSWD_t Passwd;
      /* Present password to open session  */
      Passwd.MsbPasswd = MsbPasswd;
      Passwd.LsbPasswd = LsbPasswd;
      ret = BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );

      pRfprotZone.PasswdCtrl = ST25DVXXKC_PROT_PASSWD1;
      pRfprotZone.RWprotection = ST25DVXXKC_WRITE_PROT;
      ret = BSP_NFCTAG_WriteRFZxSS(BSP_NFCTAG_INSTANCE, ST25DVXXKC_PROT_ZONE1, pRfprotZone);

      /* present wrong password for closing the session */
      Passwd.MsbPasswd = ~MsbPasswd;
      Passwd.LsbPasswd = ~LsbPasswd;
      BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );
    }
  }
  return ret;
}

/**
  * @brief  Disable the R/F writing protection for Zone 1 using RF_PWD_1 password
  * @param  uint32_t MsbPasswd MSB 25-bit password
  * @param  uint32_t LsbPasswd LSB 25-bit password
  * @retval int32_t enum status
  */
int32_t BSP_NFCTAG_DisableRFWritingPasswordProtectionZone1(uint32_t MsbPasswd,uint32_t LsbPasswd)
{
  int32_t ret = NFCTAG_OK;
  ST25DVxxKC_RF_PROT_ZONE_t pRfprotZone;
  /* Read Level of R/F protection */
  ret = BSP_NFCTAG_ReadRFZxSS(BSP_NFCTAG_INSTANCE, ST25DVXXKC_PROT_ZONE1, &pRfprotZone );
  if(ret==NFCTAG_OK) {
    /* Change the R/F protection level if it's necessary */
    if((pRfprotZone.PasswdCtrl != ST25DVXXKC_NOT_PROTECTED ) && (pRfprotZone.RWprotection != ST25DVXXKC_NO_PROT)) {
      ST25DVxxKC_PASSWD_t Passwd;
      /* Present password to open session  */
      Passwd.MsbPasswd = MsbPasswd;
      Passwd.LsbPasswd = LsbPasswd;
      ret = BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );

      pRfprotZone.PasswdCtrl = ST25DVXXKC_NOT_PROTECTED;
      pRfprotZone.RWprotection = ST25DVXXKC_NO_PROT;
      ret = BSP_NFCTAG_WriteRFZxSS(BSP_NFCTAG_INSTANCE, ST25DVXXKC_PROT_ZONE1, pRfprotZone);

      /* present wrong password for closing the session */
      Passwd.MsbPasswd = ~MsbPasswd;
      Passwd.LsbPasswd = ~LsbPasswd;
      BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );
    }
  }
  return ret;
}

/**
  * @brief  Check and Eventually Change the Energy harvesting mode 
  * @param  uint32_t MsbPasswd MSB 32-bit password
  * @param  uint32_t LsbPasswd LSB 32-bit password
  * @param  ST25DVxxKC_EH_MODE_STATUS NewEHMode Energy harvesting mode
  * @retval int32_t enum status
  */
int32_t BSP_NFCTAG_CheckChangeEHMODE(uint32_t MsbPasswd,uint32_t LsbPasswd,ST25DVxxKC_EH_MODE_STATUS_E NewEHMode)
{
  int32_t ret = NFCTAG_OK;
  ST25DVxxKC_EH_MODE_STATUS_E EHmode;
  ret = BSP_NFCTAG_ReadEHMode(BSP_NFCTAG_INSTANCE, &EHmode );
  if((EHmode != NewEHMode) && (ret==NFCTAG_OK)){
    ST25DVxxKC_PASSWD_t Passwd;
    /* Present password to open session  */
    Passwd.MsbPasswd = MsbPasswd;
    Passwd.LsbPasswd = LsbPasswd;
    ret = BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );
    BSP_NFCTAG_WriteEHMode(BSP_NFCTAG_INSTANCE, NewEHMode);
    /* present wrong password for closing the session */
    Passwd.MsbPasswd = ~MsbPasswd;
    Passwd.LsbPasswd = ~LsbPasswd;
    BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );
  }
  return ret;
}

/**
  * @brief  Check the Energy harvesting mode 
  * @param  ST25DVxxKC_EH_MODE_STATUS EHMode Energy harvesting mode
  * @retval int32_t enum status
  */
int32_t BSP_NFCTAG_CheckEHMODE(ST25DVxxKC_EH_MODE_STATUS_E *EHMode)
{
  int32_t ret = NFCTAG_OK;
  ret = BSP_NFCTAG_ReadEHMode(BSP_NFCTAG_INSTANCE, EHMode );
  return ret;
}

/**
  * @brief  Change the Energy harvesting mode 
  * @param  uint32_t MsbPasswd MSB 32-bit password
  * @param  uint32_t LsbPasswd LSB 32-bit password
  * @param  ST25DVxxKC_EH_MODE_STATUS NewEHMode Energy harvesting mode
  * @retval int32_t enum status
  */
int32_t BSP_NFCTAG_ChangeEHMODE(uint32_t MsbPasswd,uint32_t LsbPasswd,ST25DVxxKC_EH_MODE_STATUS_E NewEHMode)
{
  int32_t ret = NFCTAG_OK;
  ST25DVxxKC_PASSWD_t Passwd;
  /* Present password to open session  */
  Passwd.MsbPasswd = MsbPasswd;
  Passwd.LsbPasswd = LsbPasswd;
  ret = BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );
  
  if(ret==NFCTAG_OK) {
    ret = BSP_NFCTAG_WriteEHMode(BSP_NFCTAG_INSTANCE, NewEHMode);
  }
  
  /* present wrong password for closing the session */
  Passwd.MsbPasswd = ~MsbPasswd;
  Passwd.LsbPasswd = ~LsbPasswd;
  BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );
  return ret;
}

/**
  * @brief  Write GPO configuration:
  *      GPO managed by user             = ST25DVXXKC_GPO1_ENABLE_MASK | ST25DVXXKC_GPO1_RFUSERSTATE_MASK
  *      GPO sensible to RF activity     = ST25DVXXKC_GPO1_ENABLE_MASK | ST25DVXXKC_GPO1_RFACTIVITY_MASK
  *      GPO sensible to RF Field change = ST25DVXXKC_GPO1_ENABLE_MASK | ST25DVXXKC_GPO1_FIELDCHANGE_MASK
  *
  * @param  uint32_t MsbPasswd MSB 32-bit password
  * @param  uint32_t LsbPasswd LSB 32-bit password
  * @param  uint16_t ITConfig Provides the GPO configuration to apply
  * @retval int32_t enum status
  */
int32_t BSP_NFCTAG_WriteConfigIT(uint32_t MsbPasswd,uint32_t LsbPasswd,const uint16_t ITConfig)
{
  int32_t ret;
  ST25DVxxKC_PASSWD_t Passwd;
  /* Present password to open session  */
  Passwd.MsbPasswd = MsbPasswd;
  Passwd.LsbPasswd = LsbPasswd;
  ret = BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );

  /* Change the GPO configuration value */
  ret = BSP_NFCTAG_ConfigIT( BSP_NFCTAG_INSTANCE, ITConfig );

  /* present wrong password for closing the session */
  Passwd.MsbPasswd = ~MsbPasswd;
  Passwd.LsbPasswd = ~LsbPasswd;
  BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );

  return ret;
}

/**
  * @brief  Change MailboxConfiguration
  * @param  uint32_t MsbPasswd MSB 32-bit password
  * @param  uint32_t LsbPasswd LSB 32-bit password
  * @param  ST25DVxxKC_EN_STATUS_E MB_mode Enable Disable the Mailbox
  * @retval int32_t enum status
  */
int32_t BSP_NFCTAG_ChangeMBMode(uint32_t MsbPasswd,uint32_t LsbPasswd,const ST25DVxxKC_EN_STATUS_E MB_mode)
{
  int32_t ret;
  ST25DVxxKC_PASSWD_t Passwd;
  /* Present password to open session  */
  Passwd.MsbPasswd = MsbPasswd;
  Passwd.LsbPasswd = LsbPasswd;
  ret = BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );

  /* Change the Mailbox configuration value */
  ret = BSP_NFCTAG_WriteMBMode( BSP_NFCTAG_INSTANCE, MB_mode );

  /* present wrong password for closing the session */
  Passwd.MsbPasswd = ~MsbPasswd;
  Passwd.LsbPasswd = ~LsbPasswd;
  BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );

  return ret;
}

/**
  * @brief  Change the Mailbox watchdog coefficient delay
  * @param  uint32_t MsbPasswd MSB 32-bit password
  * @param  uint32_t LsbPasswd LSB 32-bit password
  * @param  WdgDelay Watchdog duration coefficient to be written (Watch dog duration = MB_WDG*30 ms +/- 6%).
  * @return int32_t enum status.
  */
int32_t BSP_NFCTAG_ChangeMBWDG(uint32_t MsbPasswd,uint32_t LsbPasswd,const uint8_t WdgDelay)
{
  int32_t ret;
  ST25DVxxKC_PASSWD_t Passwd;
  /* Present password to open session  */
  Passwd.MsbPasswd = MsbPasswd;
  Passwd.LsbPasswd = LsbPasswd;
  ret = BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );

  /* Change the Mailbox configuration value */
  ret = BSP_NFCTAG_WriteMBWDG( BSP_NFCTAG_INSTANCE, WdgDelay );

  /* present wrong password for closing the session */
  Passwd.MsbPasswd = ~MsbPasswd;
  Passwd.LsbPasswd = ~LsbPasswd;
  BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );

  return ret;
}

/**
  * @brief  Configures the ITtime duration for the GPO pulse.
  * @param  uint32_t MsbPasswd MSB 32-bit password
  * @param  uint32_t LsbPasswd LSB 32-bit password
  * @param  ITtime Coefficient for the Pulse duration to be written (Pulse duration = 302,06 us - ITtime * 512 / fc)
  * @retval int32_t enum status.
  */
int32_t BSP_NFCTAG_ChangeITPulse(uint32_t MsbPasswd,uint32_t LsbPasswd,const ST25DVxxKC_PULSE_DURATION_E ITtime)
{
  int32_t ret;
  ST25DVxxKC_PASSWD_t Passwd;
  /* Present password to open session  */
  Passwd.MsbPasswd = MsbPasswd;
  Passwd.LsbPasswd = LsbPasswd;
  ret = BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );

  /* Change the Mailbox configuration value */
  ret = BSP_NFCTAG_WriteITPulse( BSP_NFCTAG_INSTANCE, ITtime );

  /* present wrong password for closing the session */
  Passwd.MsbPasswd = ~MsbPasswd;
  Passwd.LsbPasswd = ~LsbPasswd;
  BSP_NFCTAG_PresentI2CPassword( BSP_NFCTAG_INSTANCE, Passwd );

  return ret;
}
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
 
