/**
  ******************************************************************************
  * @file    st25ftm_common.c
  * @author  MMY Application Team
  * @version 1.0.0
  * @date    29-July-2022
  * @brief   Fast Transfer Memory protocol common file to utilities 
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

#include <string.h>
#include "st25ftm_common.h"
#include "st25ftm_config.h"

ST25FTM_InternalState_t gFtmState;

void logHexBuf(uint8_t* buf, uint32_t len)
{
#if (ST25FTM_ENABLE_LOG != 0)
  if(len > 32)
  {
    char data[40];
    memset(data,0,sizeof(data));
    memcpy(data, ST25FTM_HEX2STR((uint8_t*)(buf + len - 16),16),32);
    ST25FTM_LOG("%s...%s\r\n",ST25FTM_HEX2STR(buf,16),data);
  } else {
    ST25FTM_LOG("%s\r\n",ST25FTM_HEX2STR(buf,len));
  }
#else
  UNUSED(buf);
  UNUSED(len);
#endif
}
 
ST25FTM_Acknowledge_Status_t ST25FTM_GetAcknowledgeStatus(void)
{
  uint8_t msg[ST25FTM_BUFFER_LENGTH];
  uint32_t msg_len = 0U;
  ST25FTM_Acknowledge_Status_t status;
  if(ST25FTM_GetMessageOwner() == ST25FTM_MESSAGE_PEER)
  {
    if(ST25FTM_ReadMessage(msg, &msg_len) != ST25FTM_MSG_OK)
    {
      status = ST25FTM_ACK_BUSY;
    } else {
      if(msg[0] == ((uint8_t)ST25FTM_SEGMENT_OK | (uint8_t)ST25FTM_STATUS_BYTE))
      {
        status = ST25FTM_SEGMENT_OK;
      } else if (msg[0] == ((uint8_t)ST25FTM_CRC_ERROR | (uint8_t)ST25FTM_STATUS_BYTE))
      {
        status = ST25FTM_CRC_ERROR;
      } else {
        /* Unexpected value, this is not a ACK */
        status = ST25FTM_ACK_ERROR;
      }
    }
  } else {
    status = ST25FTM_ACK_BUSY;
  }
  return status;
}

void ST25FTM_State_Init(void)
{
  gFtmState.state = ST25FTM_IDLE;
  gFtmState.lastState = ST25FTM_IDLE;
  gFtmState.rfField = ST25FTM_FIELD_OFF;
  gFtmState.totalDataLength = 0;
  gFtmState.retryLength = 0;
  gFtmState.lastTick = ST25FTM_TICK();

  ST25FTM_TxStateInit();
  ST25FTM_RxStateInit();
}


uint32_t ST25FTM_CompareTime(uint32_t a, uint32_t b)
{
  uint32_t result;
  if(a > b)
  {
    result = a - b;
  } else {
    result = b - a;
  }
  return result;
}
