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
   
/* Uncomment the following define for avoiding
 * to save again one sample == to the lastest saved */
//#define SMARTAG2_DONT_SAVE_EQUAL_SAMPLES

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
#define SMARTAG2_LOGMODE_INACTIVE 0
#define SMARTAG2_LOGMODE_ACTIVE   1

/* Package Version only numbers 0->9 */
#define SMARTAG2_VERSION_MAJOR  1
#define SMARTAG2_VERSION_MINOR  1
#define SMARTAG2_VERSION_PATCH  0

/* NFC Protocol Version and Revision */
#define SMARTAG2_RECORD_VERSION 2
#define SMARTAG2_RECORD_REVISION 1

/* Board Id and Firmware Id */
#define SMARTAG2_BOARD_ID 1
#define SMARTAG2_FIRMWARE_ID 4

/* NFC field */
#define FIELD_UNDEF   0
#define FIELD_FALLING 1
#define FIELD_RISING  2

/* Events type */
#define NO_EVENT    0x00
#define SYNC_EVENT  0x01
#define ASYNC_EVENT 0x02
   
/* Virtual Sensor Configuration */
#define SMARTAG2_VIRTUAL_SENSORS_NUM 10
#define STTS22H_VS_ID         0
#define LPS22DF_VS_ID         1
#define VD6283_LUX_VS_ID      2
#define VD6283_CCT_VS_ID      3
#define LSM6DSOX32_VS_ID      4
#define LSM6DSOX32_6D_VS_ID   5
#define LSM6DSOX32_MLC_VS_ID  6
#define LIS2DUXS12_MLC_VS_ID  7
#define LIS2DUXS12_VS_ID      8
#define LIS2DUXS12_6D_VS_ID   9


/* Accelerometer 6D orientation */
#define ORIENTATION_UNDEF  0x00
#define ORIENTATION_RIGHT  0x01
#define ORIENTATION_TOP    0x02
#define ORIENTATION_LEFT   0x03
#define ORIENTATION_BOTTOM 0x04
#define ORIENTATION_UP     0x05
#define ORIENTATION_DOWN   0x06

//Asset Tracking LIS2DUXS12 MLC output */
#define AT_STATIONARY_UPRIGHT     0x00
#define AT_STATIONARY_NOT_UPRIGHT 0x04
#define AT_IN_MOTION              0x08
#define AT_SHAKEN                 0x0C
   
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

