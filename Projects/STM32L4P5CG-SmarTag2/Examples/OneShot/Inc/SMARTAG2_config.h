/**
  ******************************************************************************
  * @file    SMARTAG2_config.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.1.0
  * @date    22-February-2023
  * @brief   Configuration file for SmarTag application
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMARTAG2_CONFIG_H
#define __SMARTAG2_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Uncomment the following define for disabling the LowPowerMode for Debug */
//#define SMARTAG2_ENABLE_DEBUG

/* Uncomment the following define for enabling the VCOM printf */
#define SMARTAG2_ENABLE_PRINTF

/* AutoStart after 2 Seconds */
#define SMARTAG2_AUTOSTART_SECONDS 2
   
/* Default Sample value time in seconds */
#define DATA_DEFAULT_SAMPLE_INT 60
   
#ifdef SMARTAG2_ENABLE_PRINTF
  //Uncomment the following defines for increasing the verbosity level
  //#define  SMARTAG2_VERBOSE_PRINTF
#endif /* SMARTAG2_ENABLE_PRINTF */



/* Default Epoch Start time if there is not a Valid Configuration:
 *     GMT: Monday 4 July 2023 07:28:53
 */   
#define SMARTAG2_DEFAULT_EPOCH_START_TIME 1656919733


/******************  Don't change the following section ***********************/
   
#define ST25_RETRY_NB     ((uint8_t) 15)
#define ST25_RETRY_DELAY  ((uint8_t) 40)

/**
  * @brief Iterate ST25DV command depending on the command return status.
  * @param cmd A ST25DV function returning a NFCTAG_StatusTypeDef status.
  */
#define ST25_RETRY(cmd) {              \
    int st25_retry = ST25_RETRY_NB;    \
    int st25_status = NFCTAG_ERROR;    \
    while((st25_status != NFCTAG_OK) & (st25_retry>0)) { \
      st25_status = cmd;               \
      if(st25_status != NFCTAG_OK) {   \
        HAL_Delay(ST25_RETRY_DELAY);   \
      }                                \
      st25_retry--;                    \
    }                                  \
}
                        
/** SmarTag logging mode functionality ****************************************/

/* Package Version only numbers 0->9 */
#define SMARTAG2_VERSION_MAJOR  1
#define SMARTAG2_VERSION_MINOR  0
#define SMARTAG2_VERSION_PATCH  2

/* NFC Protocol Version and Revision */
#define SMARTAG2_RECORD_VERSION 2
#define SMARTAG2_RECORD_REVISION 1

/* Board Id and Firmware Id */
#define SMARTAG2_BOARD_ID 1
#define SMARTAG2_FIRMWARE_ID 3
   
/* Virtual Sensor Configuration */
#define SMARTAG2_VIRTUAL_SENSORS_NUM 4
#define STTS22H_VS_ID         0
#define LPS22DF_VS_ID         1
#define VD6283_LUX_VS_ID      2
#define VD6283_CCT_VS_ID      3

#ifdef SMARTAG2_ENABLE_PRINTF
  #include <stdio.h>
  #define SMARTAG2_PRINTF(...) printf(__VA_ARGS__)
#else /* SMARTAG2_ENABLE_PRINTF */
  #define SMARTAG2_PRINTF(...)
#endif /* SMARTAG2_ENABLE_PRINTF */

#ifdef __cplusplus
}
#endif

#endif /* __SMARTAG2_CONFIG_H */

