/**
  ******************************************************************************
  * @file    st25ftm_protocol.c
  * @author  MMY Application Team
  * @version 1.0.0
  * @date    29-July-2022
  * @brief   Fast Transfer Memory protocol APIs
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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "st25ftm_protocol.h"
#include "st25ftm_common.h"
#include "st25ftm_config.h"

#if (ST25FTM_ENABLE_LOG != 0)
/*! String version of the FTM Reception state machine for display */
const char * ST25FTM_RxState_Str[] = {

  "FTM_READ_IDLE",
  "FTM_READ_CMD",
  "FTM_READ_PKT",
  "FTM_READ_WRITE_ACK",
  "FTM_READ_WRITE_NACK",
  "FTM_READ_WRITE_ERR",
  "FTM_READ_WAIT_ACK_READ",
  "FTM_READ_BUFFER_FULL",
  "FTM_READ_DONE",
  "FTM_READ_ERROR"
};

/*! String version of the FTM main state machine for display */
const char * ST25FTM_State_Str[] = {
  "FTM_IDLE",
  "FTM_WRITE",
  "FTM_READ"
};

/*! String version of the FTM transmission state machine for display */
const char * ST25FTM_TxState_Str[] = {
  "FTM_WRITE_IDLE",
  "FTM_WRITE_CMD",
  "FTM_WRITE_SEGMENT",
  "FTM_WRITE_PKT",
  "FTM_WRITE_WAIT_READ",
  "FTM_WRITE_READ_ACK",
  "FTM_WRITE_DONE",
  "FTM_WRITE_ERROR",
};
#endif

/*! Initialize the FTM state machines and the NFC device */
void ST25FTM_Init(void)
{
  ST25FTM_State_Init();

  (void)ST25FTM_DeviceInit();

  ST25FTM_UpdateFieldStatus();
}

/*! Register the maximum frame length while transmitting
 *  @param len Maximum frame length in bytes
 */
void ST25FTM_SetTxFrameMaxLength(uint32_t len)
{
  gFtmState.tx.frameMaxLength = len;
}

/*! Get the maximum frame length while transmitting
 * @return The maxmum number of bytes per transmitted frame
 */
uint32_t ST25FTM_GetTxFrameMaxLength(void)
{
  return gFtmState.tx.frameMaxLength;
}

/*! Register the maximum frame length while receiving
 *  @param len Maximum frame length in bytes
 */
void ST25FTM_SetRxFrameMaxLength(uint32_t len)
{
  gFtmState.rx.frameMaxLength = len;
}

/*! Get the maximum frame length while receiving
 * @return The maxmum number of bytes per received frame
 */
uint32_t ST25FTM_GetRxFrameMaxLength(void)
{
  return gFtmState.rx.frameMaxLength;
}


/*! Initialize a transmission
  * @param  data Pointer to the data buffer to be transmitted
  * @param length Number of bytes to be transmitted
  * @param ack Enables handchecks during the transfer
  * @param data_cb Optional callback function, called to request data to send (to be set to NULL if not used)
  */
void ST25FTM_SendCommand(uint8_t* data, uint32_t length, ST25FTM_Send_Ack_t ack, ftm_data_cb data_cb)
{
  gFtmState.tx.cmdPtr = data;
  gFtmState.tx.cmdLen = length;
  gFtmState.tx.state = ST25FTM_WRITE_IDLE;
  gFtmState.state = ST25FTM_WRITE;
  gFtmState.tx.sendAck = ack;
  gFtmState.tx.getdata_cb = data_cb;
}


/*! Initialize a reception
  * @param  data Pointer to the data buffer used for the reception
  * @param length Pointer to a word defining the maximum number of bytes that can be received.
                  This parameter is also used to return the number of bytes actually read
  * @param ack Enables handchecks during the transfer
  * @param data_cb Optional callback function, called to write received datad (to be set to NULL if not used)
  */
void ST25FTM_ReceiveCommand(uint8_t* data, uint32_t *length, ftm_data_cb data_cb)
{
  gFtmState.rx.cmdPtr = data;
  gFtmState.rx.cmdLen = length;
  gFtmState.rx.maxCmdLen = *length;
  gFtmState.rx.state = ST25FTM_READ_IDLE;
  gFtmState.state = ST25FTM_READ;
  gFtmState.rx.recvdata_cb = data_cb;
}

/*! Run the FTM state machine */
void ST25FTM_Runner(void)
{
  static ST25FTM_Field_State_t lastRfField = ST25FTM_FIELD_OFF;
  if(gFtmState.state != gFtmState.lastState)
  {
    ST25FTM_LOG("State = %s\r\n",ST25FTM_State_Str[gFtmState.state]);
    gFtmState.lastState = gFtmState.state;
  }

  ST25FTM_UpdateFieldStatus();
  /* Do nothing if field is off */
  if(gFtmState.rfField == ST25FTM_FIELD_OFF)
  {
    lastRfField = ST25FTM_FIELD_OFF;
  } else {
    /* a field off occured while transmitting, restart at the beg of the segment
       We don't know if last packet has been read or not => restart segment */
    if((lastRfField == ST25FTM_FIELD_OFF) && (gFtmState.rfField == ST25FTM_FIELD_ON))
    {
      if((gFtmState.state == ST25FTM_WRITE) && (gFtmState.tx.state >= ST25FTM_WRITE_SEGMENT))
      {
        ST25FTM_LOG("FIELD OFF while transmiting, restart segment\r\n");
        ST25FTM_LOG("  segmentRemainingData=%d\r\n",gFtmState.tx.segmentRemainingData);
        ST25FTM_LOG("  pktIndex=%d\r\n",gFtmState.tx.pktIndex);
        ST25FTM_LOG("  segmentPtr=%X\r\n",gFtmState.tx.segmentPtr);
        ST25FTM_LOG("  segmentIndex=%d\r\n",gFtmState.tx.segmentIndex);
        ST25FTM_LOG("  state=%s\r\n",ST25FTM_TxState_Str[gFtmState.tx.state]);
        ST25FTM_TxResetSegment();
        ST25FTM_LOG("After reset:\r\n");
        ST25FTM_LOG("  segmentRemainingData=%d\r\n",gFtmState.tx.segmentRemainingData);
        ST25FTM_LOG("  pktIndex=%d\r\n",gFtmState.tx.pktIndex);
        ST25FTM_LOG("  segmentPtr=%X\r\n",gFtmState.tx.segmentPtr);
        ST25FTM_LOG("  segmentIndex=%d\r\n",gFtmState.tx.segmentIndex);
        ST25FTM_LOG("  state=%s\r\n",ST25FTM_TxState_Str[gFtmState.tx.state]);
        gFtmState.lastTick = ST25FTM_TICK();

      }
    }

    if(gFtmState.state == ST25FTM_WRITE)
    {
      if(gFtmState.tx.state != gFtmState.tx.lastState)
      {
        ST25FTM_LOG("TxState = %s\r\n",ST25FTM_TxState_Str[gFtmState.tx.state]);
        gFtmState.tx.lastState = gFtmState.tx.state;
        gFtmState.lastTick = ST25FTM_TICK();
      }
      if((ST25FTM_CompareTime(ST25FTM_TICK(),gFtmState.lastTick) > ST25FTM_WAIT_TIMEOUT)
         && (gFtmState.tx.state == ST25FTM_WRITE_READ_ACK))
      {
        /* a timeout occured while waiting for the RF to read packet or write a ack
           reset segment transmission */
        ST25FTM_LOG("Timeout while transmitting, restart segment\r\n");
        ST25FTM_TxResetSegment();
        gFtmState.lastTick = ST25FTM_TICK();
      }
      ST25FTM_Transmit();
    } else if (gFtmState.state == ST25FTM_READ)
    {
      if(gFtmState.rx.state != gFtmState.rx.lastState)
      {
        ST25FTM_LOG("RxState = %s\r\n",ST25FTM_RxState_Str[gFtmState.rx.state]);
        gFtmState.rx.lastState = gFtmState.rx.state;
        gFtmState.lastTick = ST25FTM_TICK();
      }
      ST25FTM_Receive();
    } else {
      /* do nothing */
    }
    lastRfField = gFtmState.rfField;
  }
}

/*! Get the current FTM state. Resets the state machine in case an error occured during the transmission.
  * @retval ST25FTM_IDLE State machine is Idle, the transfer is over.
  * @retval ST25FTM_READ Reception is on-going.
  * @retval ST25FTM_WRITE Transmission is on-going.
 */
ST25FTM_State_t ST25FTM_Status(void)
{
  ST25FTM_State_t state = gFtmState.state;
  if(gFtmState.state == ST25FTM_READ)
  {
    if((gFtmState.rx.state == ST25FTM_READ_DONE) || (gFtmState.rx.state == ST25FTM_READ_ERROR))
    {
    /* let the application reset the state machine */
    }
  }
  if(gFtmState.state == ST25FTM_WRITE)
  {
    if((gFtmState.tx.state == ST25FTM_WRITE_DONE) || (gFtmState.tx.state == ST25FTM_WRITE_ERROR))
    {
      ST25FTM_Reset();
    }
  }
  return state;
}

/*! Detect that a new reception has started.
  * @retval 1 when a new recpetion has started since last call
  * @retval 0 otherwise
*/
uint8_t ST25FTM_IsNewFrame(void)
{
  uint8_t status;
  if(gFtmState.rx.isNewFrame != 0U)
  {
    ST25FTM_LOG("*** Rx New Frame ***\r\n");
    gFtmState.rx.isNewFrame = 0U;
    status = 1U;
  } else {
    status = 0U;
  }
  return status;
}

/*! Compute the current transfer progress.
  * @return THe current transfer progress (percentage).
  */
uint32_t ST25FTM_GetTransferProgress(void)
{
  uint32_t progress;
  if(gFtmState.state == ST25FTM_WRITE)
  {
    progress = ((gFtmState.tx.cmdLen - gFtmState.tx.remainingData - gFtmState.tx.segmentRemainingData) * 100U) / gFtmState.tx.cmdLen;
  } else if (gFtmState.state == ST25FTM_READ)
  {
    progress = (gFtmState.rx.totalValidReceivedLength * 100U) / *gFtmState.rx.cmdLen;
  } else {
    progress = 0;
  }
  return progress;
}

/*! Get the number of byte received.
    It can be used by the application to process the received data before transfer completion.
  * @return The current number of valid bytes received.
  */
uint32_t ST25FTM_GetAvailableDataLength(void)
{
  return gFtmState.rx.validReceivedLength;
 }

/*! Read received bytes during the transmission, freeing space to continue the reception.
    It can be used by the application to process the received data before transfer completion.
  * @param dst The buffer to copy the received data.
  * @param length Number of bytes to copy.
  * @retval 0 if the data has been copied.
  * @retval 1 otherwise.
  */
uint8_t ST25FTM_ReadBuffer(uint8_t *dst,  uint32_t length)
{
  uint8_t status;
  if(length <= gFtmState.rx.validReceivedLength)
  {
    gFtmState.rx.receivedLength -= length;
    gFtmState.rx.validReceivedLength -= length;
    (void)memcpy(dst,gFtmState.rx.cmdPtr,length);
    (void)memmove(gFtmState.rx.cmdPtr,&gFtmState.rx.cmdPtr[length],gFtmState.rx.receivedLength);
    gFtmState.rx.dataPtr -= length;
    gFtmState.rx.segmentPtr -= length;
    gFtmState.rx.readBufferOffset += length;
    status = 0;
  } else {
    status = 1;
  }
  return status;
}

/*! Get the current offset in the command of the next byte that will be read with ST25FTM_ReadBuffer.
    It can be used by the application to keep track of the incoming data before the transfer completes.
  * @return The offset of the next byte to read.
  */
uint32_t ST25FTM_GetReadBufferOffset(void)
{
  return gFtmState.rx.readBufferOffset;
}

/*! Get the current field state (only relevant for dynamic tag device).
  * @retval 1 if RF field is present.
  * @retval 0 otherwise.
*/
ST25FTM_Field_State_t ST25FTM_GetFieldState(void)
{
  return gFtmState.rfField;
}


/*! Get the total length of the transfer (including protocol metadata).
  * @return The total number of bytes transfered.
*/
uint32_t ST25FTM_GetTotalLength(void)
{
  return gFtmState.totalDataLength;
}

/*! Get the number of bytes that have been resent.
  * @return The number of bytes that have been resent during this transfer.
*/
uint32_t ST25FTM_GetRetryLength(void)
{
  return gFtmState.retryLength;
}

/*! Check if the reception has been completed.
  * @retval 1 is the reception has completed.
  * @retval 0 otherwise.
*/
uint8_t ST25FTM_IsReceptionComplete(void)
{
  uint8_t isRxCompleted;
  if ((gFtmState.state == ST25FTM_READ) && (gFtmState.rx.state == ST25FTM_READ_DONE))
  {
    isRxCompleted = 1;
  } else {
    isRxCompleted = 0;
  }
  return isRxCompleted;
}

/*! Check if the transmission has been completed.
  * @retval 1 is the transmission has completed.
  * @retval 0 otherwise.
*/
uint8_t ST25FTM_IsTransmissionComplete(void)
{
  uint8_t isTxCompleted;
  if ((gFtmState.state == ST25FTM_WRITE) && (gFtmState.tx.state == ST25FTM_WRITE_DONE))
  {
    isTxCompleted = 1;
  } else {
    isTxCompleted = 0;
  }
  return isTxCompleted;
}

/*! Check if the ST25FTM state machine is idle.
  * @retval 1 The state machine is Idle.
  * @retval 0 otherwise.
*/
uint8_t ST25FTM_IsIdle(void)
{
  uint8_t isIdle;
  if (gFtmState.state == ST25FTM_IDLE)
  {
    isIdle = 1;
  } else {
    isIdle = 0;
  }
  return isIdle;
}

/*! Check if an error occured.
  * @retval 1 An error occured.
  * @retval 0 otherwise.
*/
uint8_t ST25FTM_CheckError(void)
{
  uint8_t isError;
  if((gFtmState.rx.state == ST25FTM_READ_ERROR)  || (gFtmState.tx.state == ST25FTM_WRITE_ERROR))
  {
    isError = 1;
  } else {
    isError = 0;
  }
  return isError;
}

/*! Reset the ST25FTM state machine.
  */
void ST25FTM_Reset(void)
{
  gFtmState.tx.cmdPtr = NULL;
  gFtmState.tx.cmdLen = 0;
  gFtmState.tx.state = ST25FTM_WRITE_IDLE;
  gFtmState.rx.state = ST25FTM_READ_IDLE;
  gFtmState.state = ST25FTM_IDLE;
}


/*! Set the lenth of a segment (in bytes, max value is ST25FTM_SEGMENT_LEN) */
void ST25FTM_SetTxSegmentMaxLength(uint32_t length)
{
  if(length <= ST25FTM_SEGMENT_LEN)
  {
    gFtmState.tx.segmentMaxLength = length;
  }
}

/*! Reset the lenth of a segment to its max value ST25FTM_SEGMENT_LEN */
void ST25FTM_ResetTxSegmentMaxLength(void)
{
    gFtmState.tx.segmentMaxLength = ST25FTM_SEGMENT_LEN;
}

/*! Get the lenth of a segment (in bytes) */
uint32_t ST25FTM_GetTxSegmentMaxLength(void)
{
  return gFtmState.tx.segmentMaxLength;
}
