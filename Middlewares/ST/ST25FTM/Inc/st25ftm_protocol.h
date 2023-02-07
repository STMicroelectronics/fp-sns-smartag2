/**
  ******************************************************************************
  * @file    st25ftm_protocol.h
  * @author  MMY Application Team
  * @version 1.0.0
  * @date    29-July-2022
  * @brief   Fast Transfer Memory protocol APIs prototypes
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

#ifndef ST25FTM_PROTOCOL_H
#define ST25FTM_PROTOCOL_H
#include <stdint.h>

/*! ST25FTM Transmission state machine states */
typedef enum {
  ST25FTM_WRITE_IDLE,       /*!< The state machine is idle */
  ST25FTM_WRITE_CMD,        /*!< Start a transfer */
  ST25FTM_WRITE_SEGMENT,    /*!< Start a segment */
  ST25FTM_WRITE_PKT,        /*!< Write a message */
  ST25FTM_WRITE_WAIT_READ,  /*!< Wait a message to be read */
  ST25FTM_WRITE_READ_ACK,   /*!< Read the acknowledge from the peeer device */
  ST25FTM_WRITE_DONE,       /*!< End of the transfer */
  ST25FTM_WRITE_ERROR,      /*!< An unrecoverable error occured */
} ST25FTM_TxState_t;

/*! ST25FTM Reception state machine states */
typedef enum {
  ST25FTM_READ_IDLE,            /*!< The state machine is idle */
  ST25FTM_READ_CMD,             /*!< Start the reception */
  ST25FTM_READ_PKT,             /*!< Read a message */
  ST25FTM_READ_WRITE_ACK,       /*!< Write an acknowledge */
  ST25FTM_READ_WRITE_NACK,      /*!< Write a non-ack  */
  ST25FTM_READ_WRITE_ERR,       /*!< Write an error while checking data integrity */
  ST25FTM_READ_WAIT_ACK_READ,   /*!< Wait the ack to be read */
  ST25FTM_READ_BUFFER_FULL,     /*!< Reserved for futur use */
  ST25FTM_READ_DONE,            /*!< Reception has completed */
  ST25FTM_READ_ERROR            /*!< An unrecoverable error occured */
} ST25FTM_RxState_t;

/*! ST25FTM main state machine states */
typedef enum {
  ST25FTM_IDLE = 0, /*!< The state machine is idle */
  ST25FTM_WRITE,    /*!< A transmission is on-going */
  ST25FTM_READ      /*!< A reception is on-going */
} ST25FTM_State_t;

/*! RF field state (for dynamic tag) */
typedef enum {
  ST25FTM_FIELD_OFF,    /*!< RF field is OFF */
  ST25FTM_FIELD_ON      /*!< RF field is ON */
} ST25FTM_Field_State_t;

/*! Handshake selection */
typedef enum {
  ST25FTM_SEND_WITHOUT_ACK=0,       /*!< The transfer does not require acknowledges*/
  ST25FTM_SEND_WITH_ACK=1,          /*!< The transfer requests acknowledges from peer device */
} ST25FTM_Send_Ack_t;

/*! Prototype for callbacks to get data to send or to write received data */
typedef void (*ftm_data_cb)(uint8_t* buf, uint8_t *src, uint32_t length);

void ST25FTM_Init(void);
void ST25FTM_SendCommand(uint8_t* data, uint32_t length, ST25FTM_Send_Ack_t ack, ftm_data_cb data_cb);
void ST25FTM_ReceiveCommand(uint8_t* data, uint32_t *length, ftm_data_cb data_cb);
void ST25FTM_Runner(void);

ST25FTM_State_t ST25FTM_Status(void);
void ST25FTM_SetTxFrameMaxLength(uint32_t len);
uint32_t ST25FTM_GetTxFrameMaxLength(void);
void ST25FTM_SetRxFrameMaxLength(uint32_t len);
uint32_t ST25FTM_GetRxFrameMaxLength(void);
uint8_t ST25FTM_IsNewFrame(void);
ST25FTM_Field_State_t ST25FTM_GetFieldState(void);
uint32_t ST25FTM_GetTransferProgress(void);
uint32_t ST25FTM_GetAvailableDataLength(void);
uint8_t ST25FTM_ReadBuffer(uint8_t *dst,  uint32_t length);
uint32_t ST25FTM_GetReadBufferOffset(void);
uint32_t ST25FTM_GetTotalLength(void);
uint32_t ST25FTM_GetRetryLength(void);
uint8_t ST25FTM_IsReceptionComplete(void);
uint8_t ST25FTM_IsTransmissionComplete(void);
uint8_t ST25FTM_CheckError(void);
uint8_t ST25FTM_IsIdle(void);
void ST25FTM_Reset(void);
void ST25FTM_SetTxSegmentMaxLength(uint32_t length);
uint32_t ST25FTM_GetTxSegmentMaxLength(void);
void ST25FTM_ResetTxSegmentMaxLength(void);

#endif
