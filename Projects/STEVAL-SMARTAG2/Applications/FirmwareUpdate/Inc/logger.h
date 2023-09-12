/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    logger.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.2.0
  * @date    30-May-2023
  * @brief   Helper to convert hex data into formated string.
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

/*
 *      PROJECT:
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file
 *
 *  \author
 *
 *  \brief Serial output log declaration file
 *
 */

/*!
 *
 * This driver provides a printf-like way to output log messages
 * via the UART interface. It makes use of the uart driver.
 *
 */

#ifndef LOGGER_H
#define LOGGER_H
#include <stdint.h>
#include <stddef.h>
/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

/*
******************************************************************************
* DEFINES
******************************************************************************
*/
#define LOGGER_ON   1
#define LOGGER_OFF  0

/*!
 *****************************************************************************
 *  \brief  helper to convert hex data into formated string
 *
 *  \param[in] data : pointer to buffer to be dumped.
 *
 *  \param[in] dataLen : buffer length
 *
 *  \return hex formated string
 *
 *****************************************************************************
 */
extern char* hex2Str(unsigned char * data, size_t dataLen);

#endif /* LOGGER_H */

