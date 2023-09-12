/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    st25ftm_config.c
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.2.0
  * @date    30-May-2023
  * @brief   FTM Configuration
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

/* Includes ----------------------------------------------------------------- */
#include "main.h"

/* Private includes --------------------------------------------------------- */
#include "SmarTag2_nfctag.h"
#include "SmarTag2_nfctag_ex.h"
#include "st25ftm_common.h"
#include "st25ftm_config.h"

#include <string.h>
#include <stdlib.h>

__weak void Error_Handler_NFC(uint32_t ErroCode);

ST25FTM_MessageOwner_t mailboxStatus = ST25FTM_MESSAGE_EMPTY;
static uint8_t FieldOnEvt;
static uint8_t FieldOffEvt;
static CRC_HandleTypeDef hcrc;

volatile uint8_t GPO_Activated=0;

static void ManageGPO( void );

#define ST25_RETRY_NB     ((uint8_t) 15)
#define ST25_RETRY_DELAY  ((uint8_t) 40)

/**
  * @brief Iterate ST25DV command depending on the command return status.
  * @param cmd A ST25DV function returning a NFCTAG_StatusTypeDef status.
  */
#define ST25_RETRY(cmd) do {                                                  \
                          int st25_retry = ST25_RETRY_NB;                     \
                          int st25_status = NFCTAG_ERROR;    \
                          while(st25_status != NFCTAG_OK)                     \
                          {                                                   \
                            st25_status = cmd;                                \
                            if(st25_status != NFCTAG_OK)                      \
                              HAL_Delay(ST25_RETRY_DELAY);                    \
                            if(st25_retry-- <= 0)                             \
                            {                                                 \
                              st25_retry = ST25_RETRY_NB;                     \
                            }                                                 \
                          }                                                   \
                      } while(0)

void ST25FTM_DeviceInit(void)
{
   /* Init GPO Activation Flag */
  GPO_Activated = 0;

  /* Change Pulse Duration */
  if(BSP_NFCTAG_ChangeITPulse(NFC_MSB_PASSWORD,NFC_LSB_PASSWORD,ST25DVXXKC_302_US)!=NFCTAG_OK ) {
    ENABLE_DEBUG_PRINTF("Error NFCTAG Setting the Duration of Interrupt Pulse\r\n");
    Error_Handler_NFC(NFC_CONFIG_ERROR);
  } else {
    ENABLE_DEBUG_PRINTF("NFCTAG Set the Duration of Interrupt Pulse\r\n");
  }

  /* GPO sensible to RF Field change & Read/Written Mailbox */
  if(BSP_NFCTAG_WriteConfigIT(NFC_MSB_PASSWORD,NFC_LSB_PASSWORD,ST25DVXXKC_GPO1_ENABLE_MASK | ST25DVXXKC_GPO1_FIELDCHANGE_MASK| ST25DVXXKC_GPO1_RFPUTMSG_MASK | ST25DVXXKC_GPO1_RFGETMSG_MASK)!=NFCTAG_OK ) {
    ENABLE_DEBUG_PRINTF("Error NFCTAG Writing the Interrupt Configuration\r\n");
    Error_Handler_NFC(NFC_CONFIG_ERROR);
  } else {
    ENABLE_DEBUG_PRINTF("NFCTAG Written the Interrupt Configuration\r\n");
  }

  /* Initialize Mailbox */
  {
    ST25DVxxKC_EN_STATUS_E MB_mode;
    if(BSP_NFCTAG_ReadMBMode(BSP_NFCTAG_INSTANCE, &MB_mode)!=NFCTAG_OK ) {
      ENABLE_DEBUG_PRINTF("Error NFCTAG Reading Mailbox Status\r\n");
      Error_Handler_NFC(NFC_CONFIG_ERROR);
    } else {
      if(MB_mode==ST25DVXXKC_DISABLE) {
        if(BSP_NFCTAG_ChangeMBMode(NFC_MSB_PASSWORD,NFC_LSB_PASSWORD,ST25DVXXKC_ENABLE)!=NFCTAG_OK ) {
          ENABLE_DEBUG_PRINTF("Error NFCTAG Enabling Mailbox\r\n");
          Error_Handler_NFC(NFC_CONFIG_ERROR);
        } else {
          /* Enable Mailbox in dynamic register */
          if(BSP_NFCTAG_SetMBEN_Dyn(BSP_NFCTAG_INSTANCE)!=NFCTAG_OK ) {
            ENABLE_DEBUG_PRINTF("Error NFCTAG Enabling Mailbox dynamic register\r\n");
            Error_Handler_NFC(NFC_CONFIG_ERROR);
          } else {
            ENABLE_DEBUG_PRINTF("NFCTAG Mailbox Enabled\r\n");
          }
        }
      } else {
        /* if already activated Clear MB content and flag */
        if(BSP_NFCTAG_ResetMBEN_Dyn(BSP_NFCTAG_INSTANCE)!=NFCTAG_OK ) {
          ENABLE_DEBUG_PRINTF("Error NFCTAG Resetting Mailbox dynamic register\r\n");
          Error_Handler_NFC(NFC_CONFIG_ERROR);
        } else {
          if(BSP_NFCTAG_SetMBEN_Dyn(BSP_NFCTAG_INSTANCE)!=NFCTAG_OK ) {
            ENABLE_DEBUG_PRINTF("Error NFCTAG Enabling Mailbox dynamic register\r\n");
            Error_Handler_NFC(NFC_CONFIG_ERROR);
          } else {
             ENABLE_DEBUG_PRINTF("NFCTAG Mailbox Enabled\r\n");
          }
        }
      }
      /* Disable MB watchdog feature */
      if(BSP_NFCTAG_ChangeMBWDG(NFC_MSB_PASSWORD,NFC_LSB_PASSWORD,0)!=NFCTAG_OK ) {
        ENABLE_DEBUG_PRINTF("Error NFCTAG Disabling Mailbox watchdog\r\n");
        Error_Handler_NFC(NFC_CONFIG_ERROR);
      } else {
         ENABLE_DEBUG_PRINTF("NFCTAG Mailbox watchdog Disabled\r\n");
      }
    }
  }
}

/**
  * @brief  De Initialize the Mailbox
  * @param  None
  * @retval None
  */
void DeInitMailbox(void) {
  if(BSP_NFCTAG_ResetMBEN_Dyn(BSP_NFCTAG_INSTANCE)!=NFCTAG_OK ) {
    ENABLE_DEBUG_PRINTF("Error NFCTAG Resetting Mailbox dynamic register\r\n");
    Error_Handler_NFC(NFC_CONFIG_ERROR);
  } else {
    ENABLE_DEBUG_PRINTF("NFCTAG Mailbox Disabled\r\n");
  }
}

/**
  * @brief  Writes message in Mailbox.
  * @param  msg Pointer to the data to write.
  * @param  msg_len Number of bytes to write.
  * @return NFCTAG_StatusTypeDef status.
  */
ST25FTM_MessageStatus_t ST25FTM_WriteMessage(uint8_t* msg, uint32_t msg_len)
{
  int ret = NFCTAG_OK;
  ST25DVxxKC_MB_CTRL_DYN_STATUS_t data = {0};

  /* Check if Mailbox is available */
  ret = BSP_NFCTAG_ReadMBCtrl_Dyn(BSP_NFCTAG_INSTANCE, &data );
  if( ret != NFCTAG_OK )
  {
    return ST25FTM_MSG_ERROR;
  }

  /* If available, write data */
  if( (data.HostPutMsg == 0) && (data.RfPutMsg == 0) )
  {
    ret = BSP_NFCTAG_WriteMailboxData(BSP_NFCTAG_INSTANCE, msg, msg_len );
  }
  else
  {
    return ST25FTM_MSG_BUSY;
  }

  if(ret == NFCTAG_OK)
  {
    mailboxStatus = ST25FTM_MESSAGE_ME;
    return ST25FTM_MSG_OK;
  } else {
    return ST25FTM_MSG_ERROR;
  }
}

/**
  * @brief  Reads entire Mailbox Message from the tag.
  * @param  msg Pointer to the read data to store.
  * @param  msg_len Number of bytes to read.
  * @return NFCTAG_StatusTypeDef status.
  */
ST25FTM_MessageStatus_t ST25FTM_ReadMessage(uint8_t *msg, uint32_t* msg_len)
{
  int ret = NFCTAG_OK;
  uint16_t mblength = 0;

  /* Read length of message */
  ret = BSP_NFCTAG_ReadMBLength_Dyn(BSP_NFCTAG_INSTANCE,  (uint8_t *)&mblength );
  if( ret != NFCTAG_OK )
  {
    return ST25FTM_MSG_ERROR;
  }
  *msg_len = mblength + 1;

  /* Read all data in Mailbox */
  ret = BSP_NFCTAG_ReadMailboxData(BSP_NFCTAG_INSTANCE, msg, 0, *msg_len );
  if(ret == NFCTAG_OK)
  {
    mailboxStatus = ST25FTM_MESSAGE_EMPTY;
    /* Trick to automatically detect the max frame length of the reader
       To have this auto detection working, the reader must send a long command
       before receiveing a long response.
    */
    ST25FTM_Ctrl_Byte_t ctrl;
    ctrl.byte = msg[0];
    if((!ST25FTM_CTRL_HAS_PKT_LEN(ctrl)) && !(msg[0] & ST25FTM_STATUS_BYTE))
    {
      ST25FTM_SetRxFrameMaxLength(*msg_len);
    }
    return ST25FTM_MSG_OK;
  }
  return ST25FTM_MSG_ERROR;
}

ST25FTM_MessageOwner_t ST25FTM_GetMessageOwner(void)
{
  /* Check if GPO IT has raised */
  ManageGPO();
  return mailboxStatus;
}

void ST25FTM_UpdateFieldStatus(void)
{
  ManageGPO();
  // first case: Both field transition occured, need to get the RF state from the register
  // second case: no RF transition, but as RF is supposed to be OFF, it doesn't harm to check the register
  if( ((FieldOffEvt == 1) && (FieldOnEvt == 1)) ||
      (((FieldOffEvt == 0) && (FieldOnEvt == 0)) && (gFtmState.rfField == ST25DVXXKC_FIELD_OFF)))
  {
    // can't decide, need to read the register to get actual state
    ST25DVxxKC_FIELD_STATUS_E field;
    int32_t status = BSP_NFCTAG_GetRFField_Dyn(BSP_NFCTAG_INSTANCE,&field);
    if((field == ST25DVXXKC_FIELD_ON) || (status == NFCTAG_NACK))
    {
      if(FieldOnEvt || FieldOffEvt)
        ST25FTM_LOG("FtmInfo Field Off->On\r\n");
      gFtmState.rfField = ST25DVXXKC_FIELD_ON;
    } else {
      gFtmState.rfField = ST25DVXXKC_FIELD_OFF;
      if(FieldOnEvt || FieldOffEvt)
        ST25FTM_LOG("FtmInfo Field On->Off\r\n");
    }
    FieldOnEvt = 0;
    FieldOffEvt = 0;

  }
  // Field transition to OFF
  if( (FieldOffEvt == 1) &&  (FieldOnEvt == 0) )
  {
    FieldOffEvt = 0;
    gFtmState.rfField = ST25DVXXKC_FIELD_OFF;
    ST25FTM_LOG("FtmInfo Field Off\r\n");
    return;
  }
  // Field transition to ON
  if( (FieldOffEvt == 0) && (FieldOnEvt == 1) )
  {
    FieldOnEvt = 0;
    gFtmState.rfField = ST25DVXXKC_FIELD_ON;
    ST25FTM_LOG("FtmInfo Field On\r\n");
  }
}

/**
  * @brief  Reads the GPO interrupt source.
  * @details This function reads the interrupt status register from the ST25DV to report which interrupt(s) occured.
  * @param None.
  * @return None.
  */
static void ManageGPO( void )
{
  uint8_t itstatus;

  if(GPO_Activated == 1)
  {
    GPO_Activated = 0;

     ST25_RETRY(BSP_NFCTAG_ReadITSTStatus_Dyn(BSP_NFCTAG_INSTANCE, &itstatus));

    if((itstatus & ST25DVXXKC_ITSTS_DYN_FIELDFALLING_MASK) == ST25DVXXKC_ITSTS_DYN_FIELDFALLING_MASK)
    {
      FieldOffEvt = 1;
    }

    if((itstatus & ST25DVXXKC_ITSTS_DYN_FIELDRISING_MASK) == ST25DVXXKC_ITSTS_DYN_FIELDRISING_MASK)
    {
      FieldOnEvt = 1;
    }

    if((itstatus & ST25DVXXKC_ITSTS_DYN_RFPUTMSG_MASK) == ST25DVXXKC_ITSTS_DYN_RFPUTMSG_MASK)
    {
      mailboxStatus = ST25FTM_MESSAGE_PEER;
    }

    if((itstatus & ST25DVXXKC_ITSTS_DYN_RFGETMSG_MASK) == ST25DVXXKC_ITSTS_DYN_RFGETMSG_MASK)
    {
      mailboxStatus = ST25FTM_MESSAGE_EMPTY;
    }
  }
}

/* CRC services */
void ST25FTM_CRC_Initialize(void)
{
  __HAL_RCC_CRC_CLK_ENABLE();
  hcrc.Instance = CRC;
  __HAL_CRC_RESET_HANDLE_STATE(&hcrc);

  hcrc.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.GeneratingPolynomial    = 0x04C11DB7;
  hcrc.Init.CRCLength               = CRC_POLYLENGTH_32B;
  hcrc.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat              = CRC_INPUTDATA_FORMAT_WORDS;
  (void)HAL_CRC_Init( &hcrc );
}

ST25FTM_Crc_t ST25FTM_GetCrc(uint8_t *data, uint32_t length, ST25FTM_crc_control_t crc_control)
{
  uint32_t crc;
  static uint8_t last_word[4];
  static uint8_t last_extra_bytes = 0;

  ST25FTM_LOG("GetCrc %x %d\r\n",data,length);
  if((crc_control == ST25FTM_CRC_START) || (crc_control == ST25FTM_CRC_ONESHOT))
  {
    /* Compute number of words */
    uint32_t nbWords = length / 4U;
    /* The uint32_t* cast is ok as buffer length is managed above */
    crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)data,nbWords);
     /* Save remaining extra bytes */
    last_extra_bytes = (uint8_t)(length%4U);
    memcpy(last_word,&data[nbWords*4U],last_extra_bytes);
  } else {
    uint32_t nbWords;
    /* In accumulate mode max length is 256 bytes (mailbox length)
       + last 3 extra bytes from a previous call */
    uint8_t temp_buff[260];
    /* starts with extra bytes from previous call, if any */
    memcpy(temp_buff, last_word, last_extra_bytes);
    /* Append the provided data */
    memcpy(&temp_buff[last_extra_bytes],data,length);
    /* Compute full length & number of words */
    length += last_extra_bytes;
    nbWords = length / 4U;
    crc = HAL_CRC_Accumulate(&hcrc, (uint32_t *)temp_buff,nbWords);
    /* Save remaining extra bytes */
    last_extra_bytes = (uint8_t)(length%4U);
    memcpy(last_word,&temp_buff[nbWords*4U],last_extra_bytes);
  }

  if((crc_control == ST25FTM_CRC_END) || (crc_control == ST25FTM_CRC_ONESHOT))
  {
    if(last_extra_bytes > 0)
    {
    /* Complete the CRC computation with the remaining bytes padded with 0 */
    memset(&last_word[last_extra_bytes],0,sizeof(last_word) - last_extra_bytes);
    crc = HAL_CRC_Accumulate(&hcrc, (uint32_t *)last_word,1);
    last_extra_bytes = 0;
    }
  }

  return crc;

}

/**
  * @brief  Error Handler for Reading/Writing function for NFC.
  *         User may add here some code to deal with this error.
  * @param  uint32_t ErroCode Error Code Flag: Reading/Writing/Configuration
  * @retval None
  */
__weak void Error_Handler_NFC(uint32_t ErroCode)
{
  switch(ErroCode) {
    case NFC_CONFIG_ERROR:
      /* Error on Configuration triple Led blinking */
      ENABLE_DEBUG_PRINTF("NFC_CONFIG_ERROR\r\n\n");

    case NFC_WRITING_ERROR:
      /* Error on NFC Writing double Led blinking */
      ENABLE_DEBUG_PRINTF("NFC_WRITING_ERROR\r\n\n");

    case NFC_READING_ERROR:
      /* Error on NFC Reading single Led blinking */
      ENABLE_DEBUG_PRINTF("NFC_READING_ERROR\r\n\n");
  }
}
