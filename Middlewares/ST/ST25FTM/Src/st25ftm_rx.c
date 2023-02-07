/**
  ******************************************************************************
  * @file    st25ftm_rx.c
  * @author  MMY Application Team
  * @version 1.0.0
  * @date    29-July-2022
  * @brief   Fast Transfer Memory manage function for data receive
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

#include "st25ftm_common.h"
#include "st25ftm_config.h"
#include <string.h>

static void ST25FTM_SetData(uint8_t* buf, uint8_t* src, uint32_t length)
{
  if(gFtmState.rx.recvdata_cb == NULL)
  {
	/* Default case: simply copy packet data into destination buffer */
	  (void)memcpy(buf,src,length);
  } else {
	/* Let's the application manage the received data */
    gFtmState.rx.recvdata_cb(buf,src,length);
  }
}


void ST25FTM_RxStateInit(void)
{
  gFtmState.rx.state = ST25FTM_READ_IDLE;
  gFtmState.rx.lastState = ST25FTM_READ_IDLE;
  gFtmState.rx.frameMaxLength = 0xFF;
  gFtmState.rx.isNewFrame = 0;
  gFtmState.rx.cmdPtr = NULL;
  gFtmState.rx.cmdLen = NULL;
  gFtmState.rx.maxCmdLen = 0;
  gFtmState.rx.nbError = 0;
  gFtmState.rx.unrecoverableError=0;
  gFtmState.rx.receivedLength = 0;
  gFtmState.rx.validReceivedLength = 0;
  gFtmState.rx.totalValidReceivedLength = 0;
  gFtmState.rx.segmentPtr = NULL;
  gFtmState.rx.dataPtr = NULL;
  gFtmState.rx.validLength = 0;
  gFtmState.rx.lastAck = 0;
  gFtmState.rx.rewriteOnFieldOff = 0;
  gFtmState.rx.ignoreRetransSegment = 0;
  gFtmState.rx.segmentNumber = 0;

}

static ST25FTM_Packet_t ST25FTM_Unpack(uint8_t *msg)
{
  ST25FTM_Packet_t pkt = {0};
  uint32_t hdr_len = sizeof(pkt.ctrl);
  pkt.ctrl.byte = msg[0];

  if(ST25FTM_CTRL_HAS_PKT_LEN(pkt.ctrl))
  {
    pkt.length = ST25FTM_GET_PKT_LEN(msg);
    hdr_len += sizeof(ST25FTM_Packet_Length_t);

    if(ST25FTM_CTRL_HAS_TOTAL_LEN(pkt.ctrl))
    {
      pkt.totalLength = ST25FTM_GET_TOTAL_LEN_WITH_LEN(msg);
      hdr_len +=sizeof(pkt.totalLength);
    }

  } else {
    if(ST25FTM_CTRL_HAS_TOTAL_LEN(pkt.ctrl))
    {
      pkt.totalLength = msg[1];
      pkt.totalLength = (pkt.totalLength << 8U) + msg[2];
      pkt.totalLength = (pkt.totalLength << 8U) + msg[3];
      pkt.totalLength = (pkt.totalLength << 8U) + msg[4];
      hdr_len +=sizeof(pkt.totalLength);
    }
    pkt.length = gFtmState.rx.frameMaxLength - hdr_len;
  }
  if(ST25FTM_CTRL_HAS_CRC(pkt.ctrl))
  {
    /* The pkt.length count the crc bytes, remove them */
    pkt.length -= 4;
  }

  /* compute the begining of the payload */
  pkt.data = msg;
  pkt.data += hdr_len;
  return pkt;
}

static void ST25FTM_RewindSegment(void)
{
  gFtmState.rx.dataPtr -= gFtmState.rx.segmentLength;
  gFtmState.rx.receivedLength -= gFtmState.rx.segmentLength;
  gFtmState.rx.segmentLength = 0;
  if(gFtmState.rx.dataPtr < gFtmState.rx.cmdPtr)
  {
    gFtmState.rx.lastError = 11;
    ST25FTM_LOG("FtmRxError11: data pointer out of band\r\n");
  }
}

static ST25FTM_StateMachineCtrl_t ST25FTM_StateRxIdle(void)
{
  gFtmState.rx.segmentPtr = gFtmState.rx.cmdPtr;
  gFtmState.rx.dataPtr = gFtmState.rx.cmdPtr;
  gFtmState.rx.segmentLength = 0;
  gFtmState.rx.receivedLength = 0;
  gFtmState.rx.validReceivedLength = 0;
  gFtmState.rx.totalValidReceivedLength = 0;
  ST25FTM_CRC_Initialize();
  gFtmState.rx.state = ST25FTM_READ_CMD;
  gFtmState.totalDataLength=0;
  gFtmState.retryLength = 0;
  gFtmState.rx.state = ST25FTM_READ_CMD;
  gFtmState.rx.segmentNumber = 0;
  gFtmState.rx.readBufferOffset = 0;
  return ST25FTM_STATE_MACHINE_CONTINUE;
}


static ST25FTM_StateMachineCtrl_t ST25FTM_StateRxCommand(void)
{
  ST25FTM_StateMachineCtrl_t control = ST25FTM_STATE_MACHINE_RELEASE;

  if(gFtmState.rx.receivedLength >= (gFtmState.rx.maxCmdLen))
  {
    /* ERROR: receive more data than we can handle */
    gFtmState.rx.nbError++;
    gFtmState.rx.unrecoverableError=1;
    gFtmState.rx.state = ST25FTM_READ_CMD;
    gFtmState.rx.lastError = 0;
    ST25FTM_LOG("FtmRxError0 too much data received\r\n");
    ST25FTM_LOG("gFtmState.rx.receivedLength=%d\r\n",gFtmState.rx.receivedLength);
    ST25FTM_LOG("gFtmState.rx.maxCmdLen=%d\r\n",gFtmState.rx.maxCmdLen);
    ST25FTM_RewindSegment();
  } else if(ST25FTM_GetMessageOwner() == ST25FTM_MESSAGE_PEER) {
    gFtmState.rx.rewriteOnFieldOff = 0;
    gFtmState.rx.state = ST25FTM_READ_PKT;

    control = ST25FTM_STATE_MACHINE_CONTINUE;
  } else {
    /* no error, do nothing */
  }
  return control;
}

static ST25FTM_StateMachineCtrl_t ST25FTM_StateRxPacket(void)
{
  ST25FTM_StateMachineCtrl_t control = ST25FTM_STATE_MACHINE_RELEASE;
  uint8_t msg[ST25FTM_BUFFER_LENGTH];
  uint32_t msg_len = 0;
  ST25FTM_Packet_t pkt;

  (void)memset(msg,0,sizeof(msg));
  if(ST25FTM_ReadMessage(msg, &msg_len) != ST25FTM_MSG_OK)
  {
    /* Cannot read MB, retry later */
    gFtmState.rx.state = ST25FTM_READ_PKT;
    control = ST25FTM_STATE_MACHINE_RELEASE;
  } else {

    ST25FTM_LOG("Rx ");
    logHexBuf(msg,msg_len);
    if(msg_len == 0U)
    {
      gFtmState.rx.lastError = 5;
      ST25FTM_LOG("FtmRxError5 len = 0\r\n");
      gFtmState.rx.nbError++;
      gFtmState.rx.state = ST25FTM_READ_ERROR;
      control =  ST25FTM_STATE_MACHINE_RELEASE;
    } else {

      pkt = ST25FTM_Unpack(msg);
      
      gFtmState.rx.pktPosition = (ST25FTM_Packet_Position_t)pkt.ctrl.b.position; 
      if((pkt.ctrl.b.position == (uint8_t)ST25FTM_SINGLE_PACKET) || (pkt.ctrl.b.position == (uint8_t)ST25FTM_FIRST_PACKET))
      {
        gFtmState.rx.segmentNumber = 0;
        if (pkt.ctrl.b.position == (uint8_t)ST25FTM_SINGLE_PACKET)
        {
          /* pkt length represents the sent data including encryption (but not crc) */
          *gFtmState.rx.cmdLen = pkt.length;
        } else {
          /* this represents the payload (unencrypted) length */
          *gFtmState.rx.cmdLen = pkt.totalLength;
        }

        if(*gFtmState.rx.cmdLen >= (gFtmState.rx.maxCmdLen))
        {
            gFtmState.rx.lastError = 17;
            gFtmState.rx.nbError++;
            gFtmState.rx.unrecoverableError=1;
            ST25FTM_LOG("FtmRxError17 Transfer is bigger than reception buffer\r\n");
        }

        if(gFtmState.totalDataLength > 0U)
        {
          gFtmState.retryLength += gFtmState.totalDataLength;
          ST25FTM_LOG("FtmRxWarning0 Command restarted, length=%d\r\n",gFtmState.retryLength);
        }
        gFtmState.totalDataLength = msg_len;
        gFtmState.rx.isNewFrame = 1;
        if(gFtmState.rx.receivedLength != 0U)
        {
          /* the transmitter started a new command without completing the last one */
          gFtmState.rx.dataPtr = gFtmState.rx.cmdPtr;
          gFtmState.rx.segmentPtr = gFtmState.rx.cmdPtr;
          gFtmState.rx.segmentLength = 0U;
          gFtmState.rx.receivedLength = 0U;
          gFtmState.rx.validReceivedLength = 0U;
          gFtmState.rx.totalValidReceivedLength = 0U;
          gFtmState.rx.readBufferOffset = 0U;
        }
      } else {
        if (gFtmState.totalDataLength == 0U)
        {
          /* we missed first packet: continue the reception and ask for retransmission */
          gFtmState.rx.nbError++;
          gFtmState.rx.lastError = 13;
          ST25FTM_LOG("FtmRxError13 First packet missed\r\n");
        } else {
          /* no error, so do nothing */
        }
        gFtmState.totalDataLength += msg_len;

      }

      if(((pkt.ctrl.b.ackCtrl & (uint8_t)ST25FTM_SEGMENT_START) != 0U)
         || (pkt.ctrl.b.ackCtrl == (uint8_t)ST25FTM_ACK_SINGLE_PKT))
      {
        ST25FTM_LOG("Starting Segment %d\r\n", gFtmState.rx.segmentNumber);
        /* detect retransmission */
        gFtmState.rx.ignoreRetransSegment = 0U;
        if(pkt.ctrl.b.segId != (gFtmState.rx.segmentNumber % 2U))
        {
          ST25FTM_LOG("Retransmission %d\r\n", pkt.ctrl.b.segId );
          /* segment is retransmitted, so don't take it into account */
          gFtmState.rx.ignoreRetransSegment = 1;
        }

        if(gFtmState.rx.segmentLength > 0U)
        {
          gFtmState.retryLength += gFtmState.rx.segmentLength;
          ST25FTM_LOG("FtmRxWarning2 Segment restarted, retryLength=%d\r\n",gFtmState.retryLength);
          ST25FTM_LOG("segmentLength=%d\r\n",gFtmState.rx.segmentLength);
          ST25FTM_LOG("receivedLength=%d\r\n",gFtmState.rx.receivedLength);
          ST25FTM_LOG("totalValidReceivedLength=%d\r\n",gFtmState.rx.totalValidReceivedLength);
        }

        /* rewind if necessary (i.e. when segment_data > 0)
           means that segment has been restarted */
        ST25FTM_RewindSegment();
      }
      if((gFtmState.rx.receivedLength + pkt.length) > gFtmState.rx.maxCmdLen)
      {
        /* ERROR: receive more data than we can handle
           this may happen if we miss several start of segment, restart segment */
        gFtmState.rx.nbError++;
        gFtmState.rx.state = ST25FTM_READ_CMD;
        gFtmState.rx.lastError = 14;
        ST25FTM_LOG("FtmRxError14 too much data received\r\n");
        ST25FTM_LOG("gFtmState.rx.receivedLength=%d\r\n",gFtmState.rx.receivedLength);
        ST25FTM_LOG("gFtmState.rx.maxCmdLen=%d\r\n",gFtmState.rx.maxCmdLen);
        ST25FTM_RewindSegment();
        control = ST25FTM_STATE_MACHINE_RELEASE;
      } else {

        //(void)memcpy(gFtmState.rx.dataPtr,pkt.data,pkt.length);
        /* don't register the data, if it has already been successfully transmitted */
        if(gFtmState.rx.ignoreRetransSegment == 0)
        {
        	ST25FTM_SetData(gFtmState.rx.dataPtr,pkt.data,pkt.length);
        }
        gFtmState.rx.dataPtr += pkt.length;
        gFtmState.rx.segmentLength += pkt.length;
        gFtmState.rx.receivedLength += pkt.length;

        if(pkt.ctrl.b.ackCtrl == (uint8_t)(ST25FTM_SEGMENT_START))
        {
      	  ST25FTM_GetCrc(pkt.data,pkt.length,ST25FTM_CRC_START);
          gFtmState.rx.state = ST25FTM_READ_CMD;
        } else if ((pkt.ctrl.b.ackCtrl == (uint8_t)ST25FTM_SEGMENT_END) || (pkt.ctrl.b.ackCtrl == (uint8_t)ST25FTM_ACK_SINGLE_PKT))
        {
          ST25FTM_Crc_t computed_crc = 0;
          if(pkt.ctrl.b.ackCtrl == (uint8_t)ST25FTM_ACK_SINGLE_PKT)
          {
            computed_crc = ST25FTM_GetCrc(pkt.data,pkt.length,ST25FTM_CRC_ONESHOT);
          } else if (pkt.ctrl.b.ackCtrl == (uint8_t)ST25FTM_SEGMENT_END) {
            computed_crc = ST25FTM_GetCrc(pkt.data,pkt.length,ST25FTM_CRC_END);
          }
          gFtmState.rx.validLength = gFtmState.rx.segmentLength;
          uint8_t* crc_p = pkt.data + pkt.length;
          uint32_t segment_crc = crc_p[0];
          segment_crc = (segment_crc << 8) + crc_p[1];
          segment_crc = (segment_crc << 8) + crc_p[2];
          segment_crc = (segment_crc << 8) + crc_p[3];
          if(segment_crc == computed_crc)
          {
            gFtmState.rx.state = ST25FTM_READ_WRITE_ACK;
          } else {
            gFtmState.rx.state = ST25FTM_READ_WRITE_NACK;
          }
          control = ST25FTM_STATE_MACHINE_CONTINUE;
        } else {
          /* not beginning or end ofa segment */
          if(pkt.ctrl.b.inSegment)
          {
            /* Accumulate CRC */
            ST25FTM_GetCrc(pkt.data,pkt.length,ST25FTM_CRC_ACCUMULATE);
          } else {
            /* No segment is used, consider the data as valid */
            gFtmState.rx.totalValidReceivedLength += pkt.length;
            gFtmState.rx.validReceivedLength += pkt.length;
          }
          if((ST25FTM_CTRL_IS_SINGLE_PACKET(pkt.ctrl.b.position)) || (ST25FTM_CTRL_IS_LAST_PACKET(pkt.ctrl.b.position)))
          {
            if(gFtmState.rx.totalValidReceivedLength == *gFtmState.rx.cmdLen)
            {
              gFtmState.rx.state = ST25FTM_READ_DONE;
            } else {
              /* inconsistent data length */
              gFtmState.rx.nbError++;
              gFtmState.rx.state = ST25FTM_READ_ERROR;         
              gFtmState.rx.lastError = 2;
              ST25FTM_LOG("FtmRxError2 Inconsistent length\r\n");
            }
          } else {
            /* this is a start/middle frame, continue reception */
            gFtmState.rx.state = ST25FTM_READ_CMD;
          }
        }
      }
    }
  }
  return control;
}

static ST25FTM_StateMachineCtrl_t ST25FTM_StateRxWriteAck(void)
{
  ST25FTM_StateMachineCtrl_t control = ST25FTM_STATE_MACHINE_RELEASE;
  ST25FTM_MessageStatus_t status;
  uint8_t msg[ST25FTM_BUFFER_LENGTH];
  int32_t msg_len = 1;

  (void)memset(msg,0,sizeof(msg));
  if(gFtmState.rx.unrecoverableError)
  {
	ST25FTM_LOG("FtmRx TxStop\r\n");
	gFtmState.rx.lastAck = 1;
	msg[0] = ((uint8_t)ST25FTM_STATUS_BYTE | (uint8_t)ST25FTM_ABORT_TRANSFER);
	gFtmState.rx.unrecoverableError = 0;
  } else if(gFtmState.rx.state == ST25FTM_READ_WRITE_ACK)
  {
    ST25FTM_LOG("FtmRx TxAck\r\n");
    gFtmState.rx.lastAck = 1;
    msg[0] = ((uint8_t)ST25FTM_STATUS_BYTE | (uint8_t)ST25FTM_SEGMENT_OK);
  } else if (gFtmState.rx.state == ST25FTM_READ_WRITE_NACK)
  {
    ST25FTM_LOG("FtmRx TxNack\r\n");
    gFtmState.rx.lastAck = 0;
    msg[0] = ((uint8_t)ST25FTM_STATUS_BYTE | (uint8_t)ST25FTM_CRC_ERROR);
  } else {
    /* Undefined Ack response */
    gFtmState.rx.lastError = 8;
    ST25FTM_LOG("FtmRxError8 Unknown ack state %d\r\n",gFtmState.rx.state);
  }
  status = ST25FTM_WriteMessage(msg,msg_len);
  if(status == ST25FTM_MSG_OK)
  {
    gFtmState.rx.state = ST25FTM_READ_WAIT_ACK_READ;
    control = ST25FTM_STATE_MACHINE_CONTINUE;
  } else if (status == ST25FTM_MSG_BUSY) {
    /* If Mailbox is busy there is a message in the mailbox
    a timeout may have occured */
    gFtmState.rx.nbError++;
    gFtmState.rx.lastError = 7;
    ST25FTM_LOG("FtmRxError7 Mailbox not empty\r\n");
    gFtmState.rx.state = ST25FTM_READ_CMD;
  }  else {
    /* If a RF operation is on-going, the I2C is NACKED: retry later! */
  }
  return control;
}


static ST25FTM_StateMachineCtrl_t ST25FTM_StateRxWaitAckRead(void)
{
  ST25FTM_StateMachineCtrl_t control = ST25FTM_STATE_MACHINE_RELEASE;
  ST25FTM_MessageOwner_t msgOwner = ST25FTM_GetMessageOwner();
  if((msgOwner == ST25FTM_MESSAGE_EMPTY) || (msgOwner == ST25FTM_MESSAGE_PEER))
  {
    ST25FTM_LOG("lastAck=%d\r\n",gFtmState.rx.lastAck);
    if(gFtmState.rx.lastAck != 0U)
    {
     /* only consider the data valid once the ack has been read */
      ST25FTM_LOG("ignoreRetrans=%d\r\n",gFtmState.rx.ignoreRetransSegment);
      if(gFtmState.rx.ignoreRetransSegment == 0U)
      {
        ST25FTM_LOG("Ending Segment %d\r\n", gFtmState.rx.segmentNumber);
        gFtmState.rx.segmentPtr = gFtmState.rx.dataPtr;
        gFtmState.rx.validReceivedLength += gFtmState.rx.validLength;
        gFtmState.rx.totalValidReceivedLength += gFtmState.rx.validLength;
        gFtmState.rx.segmentNumber++;
      } else {
        ST25FTM_LOG("Dropping retrans %d\r\n", gFtmState.rx.segmentNumber);
        gFtmState.rx.dataPtr = gFtmState.rx.segmentPtr;
        gFtmState.rx.receivedLength -= gFtmState.rx.segmentLength;
      }
      gFtmState.rx.segmentLength = 0U;

      ST25FTM_LOG("receivedLen=%d\r\n",gFtmState.rx.receivedLength);
      ST25FTM_LOG("validLen=%d\r\n",gFtmState.rx.validLength);
      ST25FTM_LOG("totalvalid=%d\r\n",gFtmState.rx.totalValidReceivedLength);
    }
    if((ST25FTM_CTRL_IS_SINGLE_PACKET(gFtmState.rx.pktPosition) ||
        ST25FTM_CTRL_IS_LAST_PACKET(gFtmState.rx.pktPosition)) &&
        (gFtmState.rx.lastAck != 0U))
    {
      if(ST25FTM_CTRL_IS_LAST_PACKET(gFtmState.rx.pktPosition) &&
          (gFtmState.rx.totalValidReceivedLength != *gFtmState.rx.cmdLen))
      {
        /* inconsistent data length -> error */
        gFtmState.rx.lastError = 3U;
        ST25FTM_LOG("FtmRxError3: Inconsistent length\r\n");
        gFtmState.rx.nbError++;
        gFtmState.rx.state = ST25FTM_READ_ERROR;
        control =  ST25FTM_STATE_MACHINE_RELEASE;
      } else {
        /* no need to check received length in single packet */
        if(ST25FTM_CTRL_IS_SINGLE_PACKET(gFtmState.rx.pktPosition))
        {
          *gFtmState.rx.cmdLen = gFtmState.rx.totalValidReceivedLength;
        }
        ST25FTM_LOG("FtmRx Ack has been read\r\n");
        gFtmState.rx.state = ST25FTM_READ_DONE;
      }
    } else {
      /* continue reception if this is not the last packet or this was a NACK */
      ST25FTM_LOG("FtmRx Continue reception\r\n");
      gFtmState.rx.state = ST25FTM_READ_CMD;
    }
  }
  return control;
}


void ST25FTM_Receive(void)
{
  ST25FTM_StateMachineCtrl_t control = ST25FTM_STATE_MACHINE_CONTINUE;
  while(control == ST25FTM_STATE_MACHINE_CONTINUE)
  {
    switch (gFtmState.rx.state)
    {
      case ST25FTM_READ_IDLE:
        control = ST25FTM_StateRxIdle();
      break;
      case ST25FTM_READ_CMD:
        control = ST25FTM_StateRxCommand();
      break;
      case ST25FTM_READ_PKT:
        control = ST25FTM_StateRxPacket();
      break;
      case ST25FTM_READ_WRITE_ACK:
      case ST25FTM_READ_WRITE_NACK:
      case ST25FTM_READ_WRITE_ERR:
        control = ST25FTM_StateRxWriteAck();
      break;
      case ST25FTM_READ_WAIT_ACK_READ:
        control = ST25FTM_StateRxWaitAckRead();
      break;
      default:
        control = ST25FTM_STATE_MACHINE_RELEASE;
      break;
    }
  }
}
