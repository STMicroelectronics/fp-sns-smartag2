/**
  ******************************************************************************
  * @file    st25ftm_common.h
  * @author  MMY Application Team
  * @version 1.0.0
  * @date    29-July-2022
  * @brief   Fast Transfer Memory protocol common header file to utilities 
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

#ifndef ST25FTM_COMMON_H
#define ST25FTM_COMMON_H

#include <stdint.h>
#include "st25ftm_protocol.h"
#include "st25ftm_config.h"

#define ST25FTM_WAIT_TIMEOUT (1000U)

// only used for TX
#define ST25FTM_MAX_DATA_IN_SINGLE_PACKET() ((gFtmState.tx.frameMaxLength) - sizeof(ST25FTM_Ctrl_Byte_t) )

#define ST25FTM_CTRL_HAS_PKT_LEN(msg)   ((msg).b.pktLen != 0U)

#define ST25FTM_CTRL_HAS_TOTAL_LEN(msg) (((msg).b.position) == (uint8_t)(ST25FTM_FIRST_PACKET))


#define ST25FTM_CTRL_IS_SINGLE_PACKET(ctrl) ((ST25FTM_Packet_Position_t)(ctrl) == (ST25FTM_SINGLE_PACKET))
#define ST25FTM_CTRL_IS_LAST_PACKET(ctrl) ((ST25FTM_Packet_Position_t)(ctrl) == (ST25FTM_LAST_PACKET))
#define ST25FTM_CTRL_IS_COMMAND_COMPLETE(msg) (ST25FTM_CTRL_IS_SINGLE_PACKET(((ST25FTM_Ctrl_Byte_t*)msg)->b.position) ||\
                                              ST25FTM_CTRL_IS_LAST_PACKET(((FTM_Ctrl_Byte_t*)msg)->b.position))



#define ST25FTM_CTRL_IS_ACK_SINGLE(ackCtrl) ((ackCtrl) == (uint8_t)(ST25FTM_ACK_SINGLE_PKT))
#define ST25FTM_CTRL_IS_SEGMENT_END(ackCtrl) ((ackCtrl) == (uint8_t)(ST25FTM_SEGMENT_END))
#define ST25FTM_CTRL_HAS_CRC(msg)       (ST25FTM_CTRL_IS_ACK_SINGLE((msg).b.ackCtrl) || \
                                      ST25FTM_CTRL_IS_SEGMENT_END((msg).b.ackCtrl))


#define ST25FTM_GET_PKT_LEN(msg)        (((ST25FTM_Header_t*)msg)->with_len.length)
#define ST25FTM_GET_TOTAL_LEN_WITH_LEN(msg)     (((ST25FTM_Header_t*)msg)->with_len.totalLength)
#define ST25FTM_GET_TOTAL_LEN_WITHOUT_LEN(msg)  (((ST25FTM_Header_t*)msg)->without_len.totalLength)

#define ST25FTM_CHANGE_ENDIANESS(x) ( (x) = \
                                  (((x)>>24U)&0xFFU) | \
                                  (((x)>>8U)&0xFF00U)| \
                                  (((x)<<8U)&0xFF0000U)| \
                                  (((x)<<24U)&0xFF000000U))

typedef enum {
  ST25FTM_SINGLE_PACKET = 0U,
  ST25FTM_FIRST_PACKET = 1U,
  ST25FTM_MIDDLE_PACKET = 2U,
  ST25FTM_LAST_PACKET = 3U
} ST25FTM_Packet_Position_t;

typedef enum {
  ST25FTM_NO_ACK_PACKET = 0U,
  ST25FTM_SEGMENT_START = 1U,
  ST25FTM_SEGMENT_END = 2U,
  ST25FTM_ACK_SINGLE_PKT = 3U
} ST25FTM_Packet_Acknowledge_t;

typedef enum {
  ST25FTM_SEGMENT_OK = 0,
  ST25FTM_CRC_ERROR = 1,
  ST25FTM_ACK_RFU = 2,
  ST25FTM_ABORT_TRANSFER = 3,
  ST25FTM_ACK_BUSY=-1,
  ST25FTM_ACK_ERROR=-2
} ST25FTM_Acknowledge_Status_t;

typedef enum {
  ST25FTM_CTRL_BYTE = 0x0,
  ST25FTM_STATUS_BYTE = 0x80
} ST25FTM_Packet_Identifier_t;

typedef union {
 struct {
  uint8_t inSegment:1;
  uint8_t segId:1;
  uint8_t position:2;
  uint8_t ackCtrl:2;
  uint8_t pktLen:1;
  uint8_t type:1;
  } b;
  uint8_t byte;
} ST25FTM_Ctrl_Byte_t;

/* This struct is used to decode the received ST25FTM headers, it must be packed */
typedef  ST25FTM_PACKED(union)  {
	ST25FTM_PACKED(struct)  {
  uint8_t ctrl;
  ST25FTM_Packet_Length_t length;
  uint32_t totalLength;
  }  with_len;
  ST25FTM_PACKED(struct)  {
  uint8_t ctrl;
  uint32_t totalLength;
  }  without_len;
}  ST25FTM_Header_t;


typedef struct {
  ST25FTM_Ctrl_Byte_t ctrl;
  uint32_t length;
  uint32_t totalLength;
  ST25FTM_Crc_t crc;
  uint8_t* data;
  uint8_t has_crc;
} ST25FTM_Packet_t;

typedef union {
  struct {
    uint8_t status:4;
    uint8_t rfu:3;
    uint8_t type:1;
  } bit;
  uint8_t byte;
} ST25FTM_Status_Byte_t;


typedef struct {
  ST25FTM_TxState_t   state;
  ST25FTM_TxState_t   lastState;
  uint8_t*        cmdPtr;
  uint32_t        cmdLen;
  uint32_t        remainingData;
  uint32_t        frameMaxLength;
  uint32_t        nbError;
  uint8_t         lastError;
  ST25FTM_Send_Ack_t  sendAck;
  uint8_t*        dataPtr;
  uint8_t*        segmentPtr;
  uint8_t*        segmentStart;
  uint32_t        segmentLength;
  uint32_t        segmentRemainingData;
  uint32_t        segmentMaxLength;
  uint8_t         retransmit;
  uint32_t        pktIndex;
  uint32_t        segmentIndex;
  uint8_t         packetBuf[ST25FTM_BUFFER_LENGTH];
  uint32_t        packetLength;
  uint32_t        segmentNumber;
  ftm_data_cb     getdata_cb;
} ST25Ftm_InternalTxState_t;

typedef struct {
  ST25FTM_RxState_t state;
  ST25FTM_RxState_t lastState;
  uint8_t       isNewFrame;
  uint8_t*      cmdPtr;
  uint32_t*     cmdLen;
  uint32_t      maxCmdLen;
  uint32_t      nbError;
  uint8_t       lastError;
  uint8_t       unrecoverableError;
  uint32_t      frameMaxLength;
  uint32_t      receivedLength;
  uint32_t      validLength;
  uint32_t      validReceivedLength;
  uint32_t      totalValidReceivedLength;
  uint8_t*      segmentPtr;
  uint32_t      segmentLength;
  uint8_t*      dataPtr;
  uint8_t       lastAck;
  uint8_t       rewriteOnFieldOff;
  uint8_t       ignoreRetransSegment;
  ST25FTM_Packet_Position_t pktPosition;
  uint32_t      segmentNumber;
  ftm_data_cb   recvdata_cb;
  uint32_t      readBufferOffset;
} ST25Ftm_InternalRxState_t;

typedef enum {
  ST25FTM_STATE_MACHINE_CONTINUE,
  ST25FTM_STATE_MACHINE_RELEASE
} ST25FTM_StateMachineCtrl_t;

typedef struct {
  ST25FTM_State_t           state;
  ST25FTM_State_t           lastState;
  ST25FTM_INTERNALSTATE_FIELD_STATUS     rfField;
  uint32_t              totalDataLength;
  uint32_t              retryLength;
  ST25Ftm_InternalTxState_t tx;
  ST25Ftm_InternalRxState_t rx;
  uint32_t              lastTick;
} ST25FTM_InternalState_t;

extern ST25FTM_InternalState_t gFtmState;


/* API for ftm_tx/rx */
void ST25FTM_CRC_Initialize(void);
ST25FTM_Crc_t ST25FTM_GetCrc(uint8_t *data, uint32_t length, ST25FTM_crc_control_t control);
void logHexBuf(uint8_t* buf, uint32_t len);
ST25FTM_Acknowledge_Status_t ST25FTM_GetAcknowledgeStatus(void);
uint32_t ST25FTM_CompareTime(uint32_t a, uint32_t b);


/* From ftm_tx/rx */
void ST25FTM_State_Init(void);
void ST25FTM_TxStateInit(void);
void ST25FTM_RxStateInit(void);
void ST25FTM_Transmit(void);
void ST25FTM_Receive(void);
void ST25FTM_TxResetSegment(void);


#endif /* ST25FTM_COMMON_H */

