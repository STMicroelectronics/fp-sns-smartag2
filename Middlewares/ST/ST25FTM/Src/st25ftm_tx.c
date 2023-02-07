/**
  ******************************************************************************
  * @file    st25ftm_tx.c
  * @author  MMY Application Team
  * @version 1.0.0
  * @date    29-July-2022
  * @brief   Fast Transfer Memory manage function for data transmit
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

#include "st25ftm_protocol.h"
#include "st25ftm_common.h"
#include "st25ftm_config.h"
#include <string.h>

static void ST25FTM_GetData(uint8_t* buf, uint8_t* src, uint32_t length)
{
  if(gFtmState.tx.getdata_cb == NULL)
  {
	/* Default case: simply copy source data into packed buffer */
    (void)memcpy(buf,src, length);
  } else {
	/* Let's the application manage the input data */
    gFtmState.tx.getdata_cb(buf, src, length);
  }
}

static uint32_t  ST25FTM_Pack(ST25FTM_Packet_t *pkt, uint8_t *msg)
{
  uint32_t index = 0;
  uint32_t data_index = 0;
  uint32_t data_payload = 0;

  msg[index] = pkt->ctrl.byte;
  index++;
  if(pkt->ctrl.b.pktLen != 0U)
  {
    msg[index] = (uint8_t)pkt->length;
    index++;
  }

  if(ST25FTM_CTRL_HAS_TOTAL_LEN(pkt->ctrl))
  {
    uint32_t txTotalLen = pkt->totalLength;
    ST25FTM_CHANGE_ENDIANESS(txTotalLen);
    (void)memcpy(&msg[index],&txTotalLen,sizeof(txTotalLen));
    index += sizeof(pkt->totalLength);
  }

  /* Save data index for Crc computation */
  data_index = index;
  data_payload = pkt->length - ( pkt->has_crc ? 4 : 0);
  ST25FTM_GetData(&msg[index],pkt->data, data_payload);
  index += data_payload;

  /* compute CRC */
  if(gFtmState.tx.sendAck == ST25FTM_SEND_WITH_ACK)
  {
    pkt->crc = 0;
    if(pkt->ctrl.b.ackCtrl == ST25FTM_SEGMENT_START)
    {
      ST25FTM_GetCrc(&msg[data_index],data_payload,ST25FTM_CRC_START);
    } else if (pkt->ctrl.b.ackCtrl == ST25FTM_SEGMENT_END)
    {
      /* pkt.length contains the CRC length, remove it */
      pkt->crc = ST25FTM_GetCrc(&msg[data_index],data_payload,ST25FTM_CRC_END);
    } else if (pkt->ctrl.b.ackCtrl == ST25FTM_ACK_SINGLE_PKT) {
      /* Single frame segment */
      pkt->crc = ST25FTM_GetCrc(&msg[data_index],data_payload,ST25FTM_CRC_ONESHOT);
    } else {
      /* middle of a segment */
      ST25FTM_GetCrc(&msg[data_index],data_payload,ST25FTM_CRC_ACCUMULATE);
    }
    if(pkt->has_crc)
    {
      ST25FTM_CHANGE_ENDIANESS(pkt->crc);
     (void)memcpy(&msg[index],&pkt->crc, 4);
      index += 4;
    }
  }

  return index;
}

void ST25FTM_TxStateInit(void)
{
  gFtmState.tx.state = ST25FTM_WRITE_IDLE;
  gFtmState.tx.lastState = ST25FTM_WRITE_IDLE;
  gFtmState.tx.frameMaxLength = 0xFF;
  gFtmState.tx.cmdPtr = NULL;
  gFtmState.tx.cmdLen = 0;
  gFtmState.tx.remainingData = 0;
  gFtmState.tx.nbError = 0;
  gFtmState.tx.sendAck = ST25FTM_SEND_WITH_ACK;
  gFtmState.tx.dataPtr = NULL;
  gFtmState.tx.segmentPtr = NULL;
  gFtmState.tx.segmentStart = NULL;
  gFtmState.tx.segmentLength = 0;
  gFtmState.tx.segmentRemainingData = 0;
  gFtmState.tx.segmentMaxLength = ST25FTM_SEGMENT_LEN;
  gFtmState.tx.retransmit = 0;
  gFtmState.tx.pktIndex = 0;
  gFtmState.tx.segmentIndex = 0;
  gFtmState.tx.packetLength = 0;
  gFtmState.tx.segmentNumber = 0;
  gFtmState.tx.getdata_cb = NULL;
  (void)memset(gFtmState.tx.packetBuf, 0, sizeof(gFtmState.tx.packetBuf));
}


/************** Ftm Tx States ***************/
static ST25FTM_StateMachineCtrl_t ST25FTM_StateTxIdle(void)
{
  ST25FTM_LOG("Tx Length %d\r\n",  gFtmState.tx.cmdLen);
  gFtmState.tx.remainingData =   gFtmState.tx.cmdLen;
  gFtmState.tx.state = ST25FTM_WRITE_CMD;
  gFtmState.tx.dataPtr = gFtmState.tx.cmdPtr;
  gFtmState.tx.segmentLength = 0;
  gFtmState.tx.segmentRemainingData = 0;
  gFtmState.tx.retransmit = 0;
  gFtmState.tx.pktIndex = 0;
  gFtmState.tx.segmentIndex = 0;
  gFtmState.totalDataLength = 0;
  gFtmState.retryLength = 0;
  gFtmState.tx.segmentNumber = 0;
  ST25FTM_CRC_Initialize();
  return ST25FTM_STATE_MACHINE_CONTINUE;
}

static ST25FTM_StateMachineCtrl_t ST25FTM_StateTxCommand(void)
{
  if (gFtmState.tx.remainingData > 0U)
  {
    uint32_t data_processed;

    ST25FTM_LOG("Starting Segment %d\r\n", gFtmState.tx.segmentNumber);

    /* prepare next segment */
    if (gFtmState.tx.sendAck == ST25FTM_SEND_WITH_ACK) {
      data_processed = (gFtmState.tx.remainingData > (gFtmState.tx.segmentMaxLength - 4U)) ?
                         (gFtmState.tx.segmentMaxLength - 4U) :
                          gFtmState.tx.remainingData;
      gFtmState.tx.segmentLength = data_processed + 4;
      gFtmState.tx.segmentPtr = gFtmState.tx.dataPtr;
    } else {
      data_processed = gFtmState.tx.remainingData;
      gFtmState.tx.segmentLength = data_processed;
      gFtmState.tx.segmentPtr = gFtmState.tx.dataPtr;
    }
    gFtmState.tx.segmentStart = gFtmState.tx.segmentPtr;
    gFtmState.tx.remainingData -= data_processed;
    gFtmState.tx.dataPtr += data_processed;
    gFtmState.tx.segmentRemainingData = gFtmState.tx.segmentLength;
    gFtmState.tx.state = ST25FTM_WRITE_SEGMENT;
  } else {
    gFtmState.tx.state = ST25FTM_WRITE_DONE;
    return ST25FTM_STATE_MACHINE_RELEASE;
  }
  return ST25FTM_STATE_MACHINE_CONTINUE;
}

static ST25FTM_StateMachineCtrl_t ST25FTM_StateTxSegment(void)
{
  ST25FTM_Packet_t pkt = {0};
  pkt.length = gFtmState.tx.frameMaxLength - sizeof(ST25FTM_Ctrl_Byte_t);
  pkt.has_crc = 0;

  pkt.data = gFtmState.tx.segmentPtr;
  pkt.ctrl.b.segId = (uint8_t)(gFtmState.tx.segmentNumber % 2U);
  pkt.ctrl.b.ackCtrl = 0;
  pkt.ctrl.b.inSegment = gFtmState.tx.sendAck == ST25FTM_SEND_WITH_ACK;

  ST25FTM_LOG("SegmentLen %d\r\n",gFtmState.tx.segmentLength);
  /* Segment has to be sent over several packets */
  if(gFtmState.tx.segmentRemainingData > ST25FTM_MAX_DATA_IN_SINGLE_PACKET())
  {

    if(gFtmState.tx.pktIndex == 0U)
    {
      /* First Packet
         don't mention packet length if the whole buffer is used */
      pkt.ctrl.b.pktLen = 0;
      pkt.totalLength = gFtmState.tx.cmdLen;
      pkt.ctrl.b.position = (uint8_t)(ST25FTM_FIRST_PACKET);
      pkt.length -= sizeof(pkt.totalLength);
    } else {
      /* Middle Packet
         don't mention packet length if the whole buffer is used */
      pkt.ctrl.b.pktLen = 0U;
      pkt.ctrl.b.position = (uint8_t)(ST25FTM_MIDDLE_PACKET);
    }
    if(gFtmState.tx.sendAck == ST25FTM_SEND_WITH_ACK)
    {
      if(gFtmState.tx.segmentIndex == 0U)
      {
        pkt.ctrl.b.ackCtrl |= (uint8_t)(ST25FTM_SEGMENT_START);
      }
      if((gFtmState.tx.segmentRemainingData - pkt.length) < sizeof(pkt.crc))
      {
        /* This is to make sure that CRC is not split between 2 packets */
        pkt.ctrl.b.pktLen = 1U;
        pkt.length = gFtmState.tx.segmentRemainingData - sizeof(pkt.crc);
      }
    }
  } else {
    /* Single or last Packet command */
    pkt.length = gFtmState.tx.segmentRemainingData;
    if(gFtmState.tx.segmentRemainingData == ST25FTM_MAX_DATA_IN_SINGLE_PACKET())
    {
      /* exact fit */
      pkt.ctrl.b.pktLen = 0U;
    } else {
      /* smaller than data buffer */
      pkt.ctrl.b.pktLen = 1U;
    }
    if(gFtmState.tx.remainingData == 0U)
    {
      if(gFtmState.tx.pktIndex == 0U)
      {
        pkt.ctrl.b.position = (uint8_t)(ST25FTM_SINGLE_PACKET);
      } else {
        pkt.ctrl.b.position = (uint8_t)(ST25FTM_LAST_PACKET);
      }
    } else if (gFtmState.tx.pktIndex != 0U) {
      pkt.ctrl.b.position = (uint8_t)(ST25FTM_MIDDLE_PACKET);
    } else {
      /* do nothing */
    }
    if(gFtmState.tx.sendAck == ST25FTM_SEND_WITHOUT_ACK)
    {
      pkt.ctrl.b.ackCtrl = (uint8_t)(ST25FTM_NO_ACK_PACKET);
    } else {
      if(gFtmState.tx.sendAck == ST25FTM_SEND_WITH_ACK)
      {
        pkt.has_crc = 1;
      }
      if(gFtmState.tx.segmentLength <= ST25FTM_MAX_DATA_IN_SINGLE_PACKET())
      {
        pkt.ctrl.b.ackCtrl = (uint8_t)(ST25FTM_ACK_SINGLE_PKT);
      } else {
        pkt.ctrl.b.ackCtrl = (uint8_t)(ST25FTM_SEGMENT_END);
      }
    }
  }
  

  ST25FTM_LOG("segmentPtr = %x\r\n",gFtmState.tx.segmentPtr);
  /* Note: segmentPtr will overflow when CRC is added for last segment packet
           not an issue since it will be reset for next segment */
  gFtmState.tx.segmentPtr += pkt.length;
  gFtmState.tx.segmentRemainingData -= pkt.length;
  gFtmState.tx.pktIndex++;
  gFtmState.tx.segmentIndex++;

  gFtmState.tx.packetLength = 0;
  (void)memset(gFtmState.tx.packetBuf,0,sizeof(gFtmState.tx.packetBuf));
  gFtmState.tx.packetLength = ST25FTM_Pack(&pkt,gFtmState.tx.packetBuf);

  ST25FTM_LOG("PktId %d\r\n",gFtmState.tx.pktIndex);
  ST25FTM_LOG("PktLen total=%d payload=%d\r\n",gFtmState.tx.packetLength,pkt.length);
  ST25FTM_LOG("Wr ");
  logHexBuf(gFtmState.tx.packetBuf,gFtmState.tx.packetLength);

  gFtmState.tx.state = ST25FTM_WRITE_PKT;
  return ST25FTM_STATE_MACHINE_CONTINUE;
}

static ST25FTM_StateMachineCtrl_t ST25FTM_StateTxPacket(void)
{
  ST25FTM_StateMachineCtrl_t control = ST25FTM_STATE_MACHINE_RELEASE;
  ST25FTM_MessageStatus_t status = ST25FTM_WriteMessage(gFtmState.tx.packetBuf,gFtmState.tx.packetLength);
  if(status == ST25FTM_MSG_OK) {
    gFtmState.totalDataLength += gFtmState.tx.packetLength;
    gFtmState.tx.state = ST25FTM_WRITE_WAIT_READ;
  } else if (status == ST25FTM_MSG_BUSY) {
    /* If there is a message in the mailbox, the status is MAILBOX_BUSY
       this is not expected, a timeout may have occured
       it may be a new command or a NACK to request retransmit
       continue with reading the MB to know what to do */
    ST25FTM_LOG("Write error, mailbox busy\r\n");
    gFtmState.tx.nbError++;
    gFtmState.tx.state = ST25FTM_WRITE_WAIT_READ;
    control = ST25FTM_STATE_MACHINE_CONTINUE;
  } else {
    /* If a RF operation is on-going, the I2C is NACKED
       retry later! */
    gFtmState.tx.state = ST25FTM_WRITE_PKT;
  }
  return control;
}

static ST25FTM_StateMachineCtrl_t ST25FTM_StateTxWaitRead(void)
{
  ST25FTM_StateMachineCtrl_t control = ST25FTM_STATE_MACHINE_RELEASE;
  ST25FTM_MessageOwner_t msgOwner = ST25FTM_GetMessageOwner();
  ST25FTM_Ctrl_Byte_t ctrl_byte;
  ctrl_byte.byte = gFtmState.tx.packetBuf[0];
  if(ST25FTM_CTRL_HAS_CRC(ctrl_byte))
  {
    if((msgOwner == ST25FTM_MESSAGE_EMPTY) || (msgOwner == ST25FTM_MESSAGE_PEER))
    {
      gFtmState.tx.state = ST25FTM_WRITE_READ_ACK;
    }
  } else {
    if(msgOwner == ST25FTM_MESSAGE_EMPTY)
    {
      if(gFtmState.tx.segmentRemainingData > 0U)
      {
        gFtmState.tx.state = ST25FTM_WRITE_SEGMENT;
        control = ST25FTM_STATE_MACHINE_CONTINUE;
      } else if (gFtmState.tx.remainingData > 0U) {
        gFtmState.tx.state = ST25FTM_WRITE_CMD;
        control = ST25FTM_STATE_MACHINE_CONTINUE;
      } else {
        gFtmState.tx.state = ST25FTM_WRITE_DONE;
      }
    } else if (msgOwner == ST25FTM_MESSAGE_PEER) {
      /* this is not expected
         continue with reading the MB to know what to do
         it may be a new command or a NACK to request retransmit */
      gFtmState.tx.nbError++;
      gFtmState.tx.state = ST25FTM_WRITE_READ_ACK;
      ST25FTM_LOG("Write error, mailbox busy 2\r\n");
      control = ST25FTM_STATE_MACHINE_CONTINUE;
    } else {
      /* this is still our message, do nothing */
    }
  }
  return control;
}

static ST25FTM_StateMachineCtrl_t ST25FTM_StateTxReadAck(void)
{
  ST25FTM_StateMachineCtrl_t control = ST25FTM_STATE_MACHINE_RELEASE;
  ST25FTM_Acknowledge_Status_t ack_status;
  
  ack_status = ST25FTM_GetAcknowledgeStatus();
  ST25FTM_LOG("Rx Ack=%d\r\n",ack_status);
  if(ack_status == ST25FTM_SEGMENT_OK)
  {
    ST25FTM_LOG("Ending Segment %d\r\n", gFtmState.tx.segmentNumber);
    gFtmState.tx.retransmit = 0U;
    gFtmState.tx.segmentIndex = 0U;
    gFtmState.tx.segmentNumber++;
    if(gFtmState.tx.remainingData == 0U)
    {
      gFtmState.tx.state = ST25FTM_WRITE_DONE;
    } else { 
      /* there are other packets to send */
      gFtmState.tx.state = ST25FTM_WRITE_CMD;
      control = ST25FTM_STATE_MACHINE_CONTINUE;
    }
  } else if (ack_status == ST25FTM_ACK_BUSY) { 
    /* do nothing */
    gFtmState.tx.state = ST25FTM_WRITE_READ_ACK;
  } else if (ack_status == ST25FTM_CRC_ERROR) {
    ST25FTM_TxResetSegment();
    control = ST25FTM_STATE_MACHINE_CONTINUE;
  } else {
    /* this is not a ACK message, it must be a new command */
    gFtmState.tx.state = ST25FTM_WRITE_ERROR;
    ST25FTM_LOG("Write error, mailbox busy 3\r\n");
  }
  return control;
}

void ST25FTM_Transmit(void)
{
  ST25FTM_StateMachineCtrl_t control = ST25FTM_STATE_MACHINE_CONTINUE;
  while(control == ST25FTM_STATE_MACHINE_CONTINUE)
  {
    switch (gFtmState.tx.state)
    {
    case ST25FTM_WRITE_IDLE:
      control = ST25FTM_StateTxIdle();
    break;
    case ST25FTM_WRITE_CMD:
      control = ST25FTM_StateTxCommand();
    break;
    case ST25FTM_WRITE_SEGMENT:
      control = ST25FTM_StateTxSegment();
    break;
    case ST25FTM_WRITE_PKT:
      control = ST25FTM_StateTxPacket();
    break;
    case ST25FTM_WRITE_WAIT_READ:
      control = ST25FTM_StateTxWaitRead();
    break;
    case ST25FTM_WRITE_READ_ACK:
      control = ST25FTM_StateTxReadAck();
    break;
    default:
      ST25FTM_Reset();
      control = ST25FTM_STATE_MACHINE_RELEASE;
    break;
    }
  }
}

void ST25FTM_TxResetSegment()
{
  gFtmState.retryLength += gFtmState.tx.segmentLength - gFtmState.tx.segmentRemainingData;
  /* rewind to retransmit */
  gFtmState.tx.segmentRemainingData = gFtmState.tx.segmentLength;
  gFtmState.tx.pktIndex -= gFtmState.tx.segmentIndex;
  gFtmState.tx.segmentPtr = gFtmState.tx.segmentStart;
  gFtmState.tx.retransmit = 1;
  gFtmState.tx.segmentIndex = 0;
  gFtmState.tx.state = ST25FTM_WRITE_SEGMENT;
}
