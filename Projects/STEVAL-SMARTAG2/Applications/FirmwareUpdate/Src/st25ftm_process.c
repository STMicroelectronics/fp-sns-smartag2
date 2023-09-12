/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    st25ftm_process.c
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.2.0
  * @date    30-May-2023
  * @brief   Fast transfer mode management
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

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "st25ftm_protocol.h"
#include "fw_command.h"
#include "st25ftm_config.h"
#include "st25ftm_process.h"

#include <string.h>

/* Private define ------------------------------------------------------------*/
/*! Start any response to a command with the command code with MSB set to 1 */
#define FTM_RESP_CMD (0x80)

/*! FW upgrade is only allowed 1 min after presenting the password  */
#define FTM_FWUPGDPWDTIMEOUT   60000

/* Private Types -------------------------------------------------------------*/
/*! Response code to the commands (2nd response byte) */
typedef enum {
  FTM_VALID        = ((uint8_t)0x81),  /*!> Command has been successfully processed */
  FTM_ERROR        = ((uint8_t)0x82),  /*!> Failure when processing the command */
  FTM_INTERNAL_ERR = ((uint8_t)0x83),  /*!> Unused value */
  FTM_UNKNOWN      = ((uint8_t)0x84),  /*!> Unknown command */
  FTM_NOT_ALLOWED  = ((uint8_t)0x85)   /*!> The command is currently not allowed */
} ftm_status_t;

/*! Supported command codes */
typedef enum {
 FTM_READ_ID        = 0x0,  /*!> Get FW and board version */
 FTM_SEND_PICTURE   = 0x1,  /*!> Send a picture to the FW */
 FTM_READ_PICTURE   = 0x2,  /*!> Read a picture from the FW */
 FTM_STOPWATCH      = 0x3,  /*!> Send timestamp */
 FTM_FW_UPGRADE     = 0x4,  /*!> Send a new FW */
 FTM_SEND_DATA      = 0x5,  /*!> Send data to the FW */
 FTM_READ_DATA      = 0x6,  /*!> Read data from the FW */
 FTM_SEND_PASSWORD  = 0x7,  /*!> Send password, required before the FW upgrade command */
 FTM_READ_PICTURE_NO_ERROR_RECOVERY = 0x08,  /*!> Read data from the FW, not using Error recovery */
 FTM_READ_DATA_NO_ERROR_RECOVERY = 0x09,  /*!> Read a picture from the FW, not using Error recovery */
 FTM_ECHO           = 0x0F,  /*!> Send and read data to/from teh FW, respomse data are the same than command. Error recovery is decided by 1st command byte */
 FTM_NO_COMMAND     = 0xFF,  /*!> Default value when no command is on-going */
} ftm_cmd_t;

/* Private variables ---------------------------------------------------------*/
/*! Set when FW upgrade is allowed */
static bool allowfwupg = false;
/*! Hold last time a password has been successfully presented */
static uint32_t fwpasswordtimeout = 0;
/*! Used to measure command duration */
static uint32_t time_elapse_ms = 0;
/*! Save current command code */
static ftm_cmd_t current_command = FTM_NO_COMMAND;
/*! Buffer used by the ST25FTM lib when receiving data */
static uint8_t cmdBuffer[6000];
/*! Length of current command */
static uint32_t cmdLength;
/*! to know if an erase is required */
static uint32_t flash_erased = 0;

/* Private function prototypes -----------------------------------------------*/
static void FTM_ProcessFirstPacket(uint8_t cmdCode, uint32_t dataLen);
static void FTM_FlushToFlash(uint8_t isLast);
static uint32_t FTM_ProcessCommand(uint8_t* data, uint32_t dataLen);
static ftm_status_t FTM_ReadId(uint8_t* response, uint32_t *length);
static ftm_status_t FTM_SendPassword(uint8_t* rxBuf, uint32_t rxLength);

/**
  * @brief  Fast transfer mode management initialization
  * @param  None
  * @retval None
  */
void FTMManagementInit( void )
{
  SMARTAG2_PRINTF("Starting FTM\r\n");
  allowfwupg = false;
  fwpasswordtimeout = 0;
  time_elapse_ms = 0;

  /* Erase Flash for Data save */
  if( COMMAND_EraseFlashFirstTime() )
  {
    SMARTAG2_PRINTF("FTM Failure!\r\nFlash cannot be erased!\r\n");
    return;
  }
  flash_erased = 1;

  /* Enable FTM */
  ST25FTM_Init();
  /* Set maximum tranmsit length (bytes) = Android NFC buffer length */
  ST25FTM_SetTxFrameMaxLength(253);
  current_command = FTM_NO_COMMAND;
  /* Set maximum reception length (in flash) */
  cmdLength = FIRMWARE_FLASH_LAST_PAGE_ADDRESS - FIRMWARE_ADDRESS;
  /* Prepare reception of a command */
  ST25FTM_ReceiveCommand(cmdBuffer,&cmdLength,NULL);

  SMARTAG2_PRINTF("Started FTM\r\n");
}

/**
  * @brief  Fast transfer mode management de-initialization
  * @param  None
  * @retval None
  */
void FTMManagementDeInit(void)
{
  ST25FTM_Reset();
  current_command = FTM_NO_COMMAND;
  cmdLength = FIRMWARE_FLASH_LAST_PAGE_ADDRESS - FIRMWARE_ADDRESS;
  ST25FTM_ReceiveCommand(cmdBuffer,&cmdLength,NULL);
  DeInitMailbox();
}

/**
  * @brief  Fast transfer mode management process
  * @param  None
  * @retval None
  */
void FTMManagement( void )
{
  ST25FTM_Runner();
  if(ST25FTM_IsNewFrame())
  {
    /* A new command has started */
    time_elapse_ms = HAL_GetTick();
    /* Save command code */
    current_command = (ftm_cmd_t)cmdBuffer[0];
    FTM_ProcessFirstPacket(current_command,cmdLength);
  }

  /* In case of big data transfer, flush the buffer to flash when data is validated */
  FTM_FlushToFlash(ST25FTM_IsReceptionComplete());

  if(ST25FTM_IsReceptionComplete())
  {
    /* Check command content and prepare response */
    FTM_ProcessCommand(cmdBuffer,cmdLength);
  }
  if(ST25FTM_IsTransmissionComplete() || ST25FTM_IsIdle())
  {      /* The response has been completed */
      if((current_command == FTM_SEND_DATA)
         || (current_command == FTM_SEND_PICTURE)
         || (current_command == FTM_FW_UPGRADE))
      {
        /* Display command time for longer commands */
        time_elapse_ms = (int)HAL_GetTick() - (int)time_elapse_ms;
        SMARTAG2_PRINTF("File transfer done!\n\nDuration: %ld ms\r\n\n", time_elapse_ms );
        if((current_command == FTM_SEND_PICTURE) || (current_command == FTM_SEND_DATA))
        {
          if(current_command == FTM_SEND_PICTURE)
          {
            SMARTAG2_PRINTF("Not Implemented FTM_SEND_PICTURE\r\n");
          }
          SMARTAG2_PRINTF( "Erasing flash: Please wait...\r\n");
          COMMAND_EraseFlash( USER_DATA_ADDRESS );
          flash_erased = 1;
        } else if (current_command == FTM_FW_UPGRADE)
        {
          allowfwupg = false;
          /* Jump to received Firmware */
          COMMAND_Jump();
        }
      }

    /* Prepare reception of next command */
    current_command = FTM_NO_COMMAND;
    cmdLength = FIRMWARE_FLASH_LAST_PAGE_ADDRESS - FIRMWARE_ADDRESS;
    ST25FTM_ReceiveCommand(cmdBuffer,&cmdLength,NULL);
  }
  if(ST25FTM_CheckError())
  {
    /* an Error occured */
    ST25FTM_Reset();
    current_command = FTM_NO_COMMAND;
    cmdLength = FIRMWARE_FLASH_LAST_PAGE_ADDRESS - FIRMWARE_ADDRESS;
    ST25FTM_ReceiveCommand(cmdBuffer,&cmdLength,NULL);
  }

  /* Check FW upgrade password timeout */
  if( (allowfwupg == true) && (current_command != FTM_FW_UPGRADE) )
  {
    if( (HAL_GetTick( ) - fwpasswordtimeout) > FTM_FWUPGDPWDTIMEOUT )
    {
      fwpasswordtimeout = 0;
      allowfwupg = false;
    }
  }
}

/*! Updates the screen depending on the on-going command
 * @param cmdCode Current command code
 * @param dataLen Length of the command (bytes)
 */
static void FTM_ProcessFirstPacket(uint8_t cmdCode, uint32_t dataLen)
{
  if((cmdCode == FTM_FW_UPGRADE) || (cmdCode == FTM_SEND_DATA) || (cmdCode == FTM_SEND_PICTURE ))
  {
    /* In case of FW upgrade, let's check if password has been presented */
    if( (cmdCode == FTM_FW_UPGRADE) && (allowfwupg == false) )
    {
      SMARTAG2_PRINTF("Fw Upgrade Demo: Need password for firmware upgrade!\r\n");
      ST25FTM_Reset();
    }

    if(((cmdCode == FTM_FW_UPGRADE) && (dataLen > FIRMWARE_FLASH_SIZE) )
        || ((cmdCode == FTM_SEND_PICTURE ) && (dataLen > USER_DATA_FLASH_SIZE) ))
    {
      SMARTAG2_PRINTF( "FTM Demo: Transfer size is bigger than receive buffer!\r\n");
      ST25FTM_Reset();
    }

    /* Erase flash if the previous command was not completed */
    if(!flash_erased)
    {
        SMARTAG2_PRINTF( "Erasing flash: Please wait...\r\n" );
        COMMAND_EraseFlash( USER_DATA_ADDRESS );
        flash_erased = 1;
    }
  }
}

/*! Process command data and prepare the response
 * @param data Command data
 * @param dataLen Length of the command (bytes)
 * @return number of bytes in the response
 */
static uint32_t FTM_ProcessCommand(uint8_t* data, uint32_t dataLen)
{
  /* by default, responds using the command buffer */
  uint8_t* response_p = data;
  uint32_t rspLen = 0;
  /* By default use error recovery for the response */
  ST25FTM_Send_Ack_t send_ack = ST25FTM_SEND_WITH_ACK;
  uint8_t cmdCode = current_command;
  ftm_status_t status = FTM_ERROR;
  /* For many commands the default repsonse is fine: command code with MSB set + VALID byte */
  uint8_t basic_response = 1;

  dataLen--;
  /* In case last command was an ECHO - segment length is shortened for ECHO commands for testing purpose */
  ST25FTM_ResetTxSegmentMaxLength();

  switch (current_command)
  {
    case FTM_READ_ID:
      status = FTM_ReadId(response_p,&rspLen);
      send_ack = ST25FTM_SEND_WITHOUT_ACK;
    break;
    case FTM_SEND_PASSWORD:
      status = FTM_SendPassword(&data[1],dataLen);
    break;
    case FTM_FW_UPGRADE:
      status = FTM_VALID;
    break;
    case FTM_SEND_PICTURE:
      SMARTAG2_PRINTF("FTM_SEND_PICTURE not Implemented\r\n");
      status = FTM_ERROR;
      current_command = FTM_NO_COMMAND;
      /* Set maximum reception length (in flash) */
      cmdLength = FIRMWARE_FLASH_LAST_PAGE_ADDRESS - FIRMWARE_ADDRESS;
      /* Prepare reception of a command */
      ST25FTM_ReceiveCommand(cmdBuffer,&cmdLength,NULL);
    break;
    case FTM_SEND_DATA:
      SMARTAG2_PRINTF("FTM_SEND_DATA not Implemented\r\n");
      status = FTM_ERROR;
      current_command = FTM_NO_COMMAND;
      /* Set maximum reception length (in flash) */
      cmdLength = FIRMWARE_FLASH_LAST_PAGE_ADDRESS - FIRMWARE_ADDRESS;
      /* Prepare reception of a command */
      ST25FTM_ReceiveCommand(cmdBuffer,&cmdLength,NULL);
    break;
    case FTM_STOPWATCH:
      SMARTAG2_PRINTF("FTM_STOPWATCH not Implemented\r\n");
      status = FTM_ERROR;
      current_command = FTM_NO_COMMAND;
      /* Set maximum reception length (in flash) */
      cmdLength = FIRMWARE_FLASH_LAST_PAGE_ADDRESS - FIRMWARE_ADDRESS;
      /* Prepare reception of a command */
      ST25FTM_ReceiveCommand(cmdBuffer,&cmdLength,NULL);
    break;
    case FTM_READ_PICTURE_NO_ERROR_RECOVERY:
    case FTM_READ_PICTURE:
      SMARTAG2_PRINTF("FTM_READ_PICTURE not Implemented\r\n");
      status = FTM_ERROR;
      current_command = FTM_NO_COMMAND;
      /* Set maximum reception length (in flash) */
      cmdLength = FIRMWARE_FLASH_LAST_PAGE_ADDRESS - FIRMWARE_ADDRESS;
      /* Prepare reception of a command */
      ST25FTM_ReceiveCommand(cmdBuffer,&cmdLength,NULL);
    break;
    case FTM_READ_DATA:
    case FTM_READ_DATA_NO_ERROR_RECOVERY:
      SMARTAG2_PRINTF("FTM_READ_DATA not Implemented\r\n");
      status = FTM_ERROR;
      current_command = FTM_NO_COMMAND;
      /* Set maximum reception length (in flash) */
      cmdLength = FIRMWARE_FLASH_LAST_PAGE_ADDRESS - FIRMWARE_ADDRESS;
      /* Prepare reception of a command */
      ST25FTM_ReceiveCommand(cmdBuffer,&cmdLength,NULL);
    break;
    case FTM_ECHO:
      basic_response = 0;
        /* Select Ack/NoAck response depending on the on the first byte of data */
      if(data[1] == 0) {
        send_ack = ST25FTM_SEND_WITHOUT_ACK;
      }
      rspLen = dataLen + 1;
      status = FTM_VALID;
      ST25FTM_SetTxSegmentMaxLength(600);
      ST25FTM_SendCommand(response_p, rspLen, send_ack, NULL);
    break;
    default:
      SMARTAG2_PRINTF("Unknown command %X\r\n", cmdCode );
    break;
  }

  if(basic_response)
  {
    /* generic response bytes: command code with MSB bit set and status OK byte */
    response_p[0] = (FTM_RESP_CMD | cmdCode);
    response_p[1] = (status);
    rspLen += 2;
    ST25FTM_SendCommand(response_p, rspLen, send_ack, NULL);
  }
  return rspLen;
}

/*! Process the ReadID command, returning FW and board version
 * @param response  pointer to the received buffer, also used for the response
 * @param length    pointer to the length of the command in bytes, also used for the response
 * @retval FTM_VALID Command processing success
 */
static ftm_status_t FTM_ReadId(uint8_t* response, uint32_t *length)
{
  response[2] = 0x02; //SMARTAG2 FTM Code
  response[3] = SMARTAG2_VERSION_MAJOR;
  response[4] = SMARTAG2_VERSION_MINOR;
  response[5] = SMARTAG2_VERSION_PATCH;
  *length = 4;
  return FTM_VALID;
}

/*! Process the Send Password command, checking provided password
 * @param rxBuf     pointer to the received buffer, also used for the response
 * @param rxLength  pointer to the length of the command in bytes, also used for the response
 * @retval FTM_VALID Correct password has been provided
 * @retval FTM_ERROR Wrong password
 */
static ftm_status_t FTM_SendPassword(uint8_t* rxBuf, uint32_t rxLength)
{
  const uint8_t fwpassword[] = {18,52,86,120};
  if( (sizeof(fwpassword) == rxLength)
      && !memcmp(fwpassword,rxBuf,rxLength))
  {
    allowfwupg = true;
    fwpasswordtimeout = HAL_GetTick( );
    SMARTAG2_PRINTF("Init Fw Flash...\r\n" );
    /* Erase Flash for Data save */
    COMMAND_EraseFlash( FIRMWARE_ADDRESS );

    SMARTAG2_PRINTF("Password OK!\r\n" );
    return FTM_VALID;
  }
  else
  {
    allowfwupg = false;
    SMARTAG2_PRINTF( "Wrong Password\r\n" );
    SMARTAG2_PRINTF( "Fw upgrade denied!!!\r\n" );
    return FTM_ERROR;
  }
}

/*! Copy valid data to the embedded flash for FW upgrade, SendData & SendPicture commands
 * @param islast Indicates if this is the last chunk of data (in which case whole remaining data is copied)
 */
void FTM_FlushToFlash(uint8_t isLast)
{
  uint8_t page_buffer[512];
  uint32_t read_length = sizeof(page_buffer);
  uint32_t write_length = read_length;
  uint32_t flash_index;
  if((current_command == FTM_FW_UPGRADE)
       || (current_command == FTM_SEND_PICTURE)
       || (current_command == FTM_SEND_DATA))
  {

    /* Get the offset of read data */
    flash_index = ST25FTM_GetReadBufferOffset();
    if(flash_index == 0)
    {
      /* If this is the first frame, removes command bytes */
      uint8_t cmd;
      ST25FTM_ReadBuffer(&cmd,1);
      flash_index = ST25FTM_GetReadBufferOffset();
    }

    while(ST25FTM_ReadBuffer(page_buffer,read_length) == 0)
    {
      /* Flush as many data as possible  */
      Command_WriteBufferToFlash( USER_DATA_ADDRESS, flash_index-1, page_buffer, write_length );
      flash_index = ST25FTM_GetReadBufferOffset();
      flash_erased = 0;
    }
    /* write the last incomplete page if the command ended */
    if(isLast)
    {
      read_length = ST25FTM_GetAvailableDataLength();
      write_length = read_length + ( (read_length % 8) ? 8 - (read_length % 8) : 0 );
      memset(page_buffer, 0xFF, write_length);
      ST25FTM_ReadBuffer(page_buffer,read_length);
      Command_WriteBufferToFlash( USER_DATA_ADDRESS, flash_index-1, page_buffer, write_length );
      flash_erased = 0;
    }
  }
}

