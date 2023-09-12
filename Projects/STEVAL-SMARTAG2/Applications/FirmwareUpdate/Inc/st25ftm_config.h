/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    st25ftm_config.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.2.0
  * @date    30-May-2023
  * @brief   FTM Configuration APIs
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

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __ST25FTM_CONFIG_H__
#define __ST25FTM_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "SmarTag2_nfctag.h"
#include "SmarTag2_nfctag_ex.h"

#ifndef NFC_READING_ERROR
#define NFC_READING_ERROR 0
#endif

#ifndef NFC_WRITING_ERROR
#define NFC_WRITING_ERROR 1
#endif

#ifndef NFC_CONFIG_ERROR
#define NFC_CONFIG_ERROR  2
#endif

#define NFC_MSB_PASSWORD 0x90ABCDEF
#define NFC_LSB_PASSWORD 0x12345678

/* Uncomment only one of the following 2 defines in function of the ST25 component Type */
#define ST25FTM_INTERNALSTATE_FIELD_STATUS  ST25DVxxKC_FIELD_STATUS_E
//#define ST25FTM_INTERNALSTATE_FIELD_STATUS ST25DV_FIELD_STATUS

/* Exported types ----------------------------------------------------------- */
/*! Fast Transfer Mode buffer access status */
typedef enum {
  ST25FTM_MSG_OK =0,        /*!< Message read/write ok */
  ST25FTM_MSG_ERROR,        /*!< The peer device doesn't respond */
  ST25FTM_MSG_BUSY          /*!< The buffer is not empty while writing */
} ST25FTM_MessageStatus_t;

/*! Fast Transfer Mode current message owner */
typedef enum {
  ST25FTM_MESSAGE_EMPTY = 0,        /*!< There is no message */
  ST25FTM_MESSAGE_ME = 1,           /*!< Current message has been written by this device */
  ST25FTM_MESSAGE_PEER = 2,         /*!< Current message has been written by the peer device */
  ST25FTM_MESSAGE_OWNER_ERROR = 3   /*!< An error occured while getting the message owner */
} ST25FTM_MessageOwner_t;

typedef enum {
  ST25FTM_CRC_START,
  ST25FTM_CRC_END,
  ST25FTM_CRC_ACCUMULATE,
  ST25FTM_CRC_ONESHOT
} ST25FTM_crc_control_t;

typedef uint32_t ST25FTM_Crc_t;

/*! Define format of the packet length field, when present */
typedef uint8_t ST25FTM_Packet_Length_t;

/* Exported constants ------------------------------------------------------- */

/* Exported defines --------------------------------------------------------- */
/*! Length of the buffer used to store a single message data */
#define ST25FTM_BUFFER_LENGTH (256)

/*! Length of the buffer used to store unvalidated data */
#define ST25FTM_SEGMENT_LEN (5795)

/*! Define the platform function to get the ms tick */
#define ST25FTM_TICK() HAL_GetTick() /* call here a function returning the system tick value */

/*! Enables debug traces for the ST25FTM library */
#define ST25FTM_ENABLE_LOG 0

/* Uncomment the following define for enabling debug traces for only user application */
#define ST25FTM_ENABLE_DEBUG

/* Exporte variables -------------------------------------------------------- */
extern volatile uint8_t GPO_Activated;

/* Exported functions ------------------------------------------------------- */
extern void Error_Handler_NFC(uint32_t ErroCode);
extern void DeInitMailbox(void);

/* Interface API */
/* Functions to implement for the platform */
/*! Check what device wrote the current message in the FTM buffer
  * @retval ST25FTM_MESSAGE_EMPTY       The buffer is empty.
  * @retval ST25FTM_MESSAGE_ME          Message has been written by this device.
  * @retval ST25FTM_MESSAGE_PEER        Message has been written by the peer device.
  * @retval ST25FTM_MESSAGE_OWNER_ERROR Message owner cannot be retrieved.
 */
extern ST25FTM_MessageOwner_t ST25FTM_GetMessageOwner(void);

/*! Read the content of the FTM buffer.
  * @param msg      A buffer used to store read data
                    Buffer length must be greater than ST25FTM_BUFFER_LENGTH.
  * @param msg_len  A pointer used to return the number of bytes read.
  * @retval ST25FTM_MSG_OK      Message successfully read.
  * @retval ST25FTM_MSG_ERROR   Unable to read the message.
*/
extern ST25FTM_MessageStatus_t ST25FTM_ReadMessage(uint8_t *msg, uint32_t* msg_len);

/*! Write the FTM buffer.
  * @param msg      The buffer containing the data to written.
  * @param msg_len  Number of bytes to write.
  * @retval ST25FTM_MSG_OK      Message successfully written.
  * @retval ST25FTM_MSG_ERROR   Unable to write the message (eg: tag has been removed).
  * @retval ST25FTM_MSG_BUSY    FTM buffer contains a meesage that has not been read yet.
*/
extern ST25FTM_MessageStatus_t ST25FTM_WriteMessage(uint8_t* msg, uint32_t msg_len);

/*! Initialize the NFC device (dynamic tag or reader) for the FTM.
*/
extern void ST25FTM_DeviceInit(void);

/*! Check if the RF field is present (for dynamic tag only)
*/
extern void ST25FTM_UpdateFieldStatus(void);

/*! Initialize the CRC computation */
extern void ST25FTM_CRC_Initialize(void);

/*! Compute a CRC32.
  * @param data     Buffer containing the data on which the CRC must be computed.
  * @param length   Number of bytes of data in the buffer.
  * @param control  Define how to compute the crc:
  *                 - starting from the initial value or from previous crc
  *                 - Adding remaining bytes (less than word) with padding
 */
extern ST25FTM_Crc_t ST25FTM_GetCrc(uint8_t *data, uint32_t length, ST25FTM_crc_control_t control);

/* Exported macros ---------------------------------------------------------- */
#if defined ( __GNUC__ ) && !defined (__CC_ARM)
/* GNU Compiler: packed attribute must be placed after the type keyword */
#define ST25FTM_PACKED(type) type __attribute__((packed,aligned(1)))
#else
/* ARM Compiler: packed attribute must be placed before the type keyword */
#define ST25FTM_PACKED(type) __packed type
#endif

#if (ST25FTM_ENABLE_LOG != 0)
#include "logger.h"
#define ST25FTM_LOG(...)  printf(__VA_ARGS__)
#define ST25FTM_HEX2STR(buf,len) hex2Str(buf,len)
#else
#define ST25FTM_LOG(...)
#define ST25FTM_HEX2STR(buf,len)
#endif

#ifdef ST25FTM_ENABLE_DEBUG
  #include <stdio.h>
  #define ENABLE_DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else /* ST25FTM_ENABLE_DEBUG */
  #define ENABLE_DEBUG_PRINTF(...)
#endif /* ST25FTM_ENABLE_DEBUG */

#ifdef __cplusplus
}
#endif

#endif /* __ST25FTM_CONFIG_H__*/

