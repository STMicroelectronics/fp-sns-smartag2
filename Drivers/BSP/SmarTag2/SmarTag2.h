/**
  ******************************************************************************
  * @file    SmarTag2.h
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.0.0
  * @date    31-August-2022
  * @brief   Header file for the BSP Common driver
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMARTAG2_H
#define __SMARTAG2_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "SmarTag2_errno.h"
#include "SmarTag2_conf.h"   
   
/* COM Feature define */
#define USE_BSP_COM_FEATURE                 1U

/* COM define */
#define USE_COM_LOG                         1U

/* IRQ priorities */
#define BSP_BUTTON_USER_IT_PRIORITY         15U

#if (USE_BSP_COM_FEATURE > 0)
  #if (USE_COM_LOG > 0)
    #if defined(__ICCARM__) || defined(__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)) /* For IAR and ARM Compiler 5 and 6*/
      #include <stdio.h>
    #endif
  #endif
#endif
/** @addtogroup BSP
 * @{
 */

/** @defgroup SMARTAG2
 * @{
 */

/** @defgroup SMARTAG2_LOW_LEVEL
 * @{
 */

/** @defgroup STM32L4XX_NUCLEO_LOW_LEVEL_Exported_Constants LOW LEVEL Exported Constants
  * @{
  */
/**
 * @brief STM32L4XX NUCLEO BSP Driver version number 1.0.0
 */
#define __SMARTAG2_BSP_VERSION_MAIN   (uint32_t)(0x01) /*!< [31:24] main version */
#define __SMARTAG2_BSP_VERSION_SUB1   (uint32_t)(0x00) /*!< [23:16] sub1 version */
#define __SMARTAG2_BSP_VERSION_SUB2   (uint32_t)(0x00) /*!< [15:8]  sub2 version */
#define __SMARTAG2_BSP_VERSION_RC     (uint32_t)(0x00) /*!< [7:0]  release candidate */
#define __SMARTAG2_BSP_VERSION        ((__SMARTAG2_BSP_VERSION_MAIN << 24)\
                                                    |(__SMARTAG2_BSP_VERSION_SUB1 << 16)\
                                                    |(__SMARTAG2_BSP_VERSION_SUB2 << 8 )\
                                                    |(__SMARTAG2_BSP_VERSION_RC))

/** @defgroup SMARTAG2_LOW_LEVEL_Exported_Types SMARTAG2 LOW LEVEL Exported Types
 * @{
 */

 /**
  * @brief Define for SMARTAG2 board
  */
#if !defined (USE_SMARTAG2)
 #define USE_SMARTAG2
#endif
#ifndef USE_BSP_COM_FEATURE
   #define USE_BSP_COM_FEATURE                  0U
#endif

/** @defgroup SMARTAG2_LOW_LEVEL_LED SMARTAG2 LOW LEVEL LED
 * @{
 */
/** Define number of LED            **/
#define LEDn                              2U

/**  Definition for BSP USER LED 1   **/
#define LED1_PIN                     	  GPIO_PIN_12
#define LED1_GPIO_PORT                    GPIOB
#define LED1_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()

/**
 * @}
 */

/** @defgroup SMARTAG2_LOW_LEVEL_BUTTON SMARTAG2 LOW LEVEL BUTTON
 * @{
 */
/* Button state */
#define BUTTON_RELEASED                   0U
#define BUTTON_PRESSED                    1U
/** Define number of BUTTON            **/
#define BUTTONn                           1U

/**
 * @brief User push-button
 */
  /**  Definition for BSP USER BUTTON   **/

#define BUS_GPIO_INSTANCE GPIO
#define BUS_BSP_BUTTON_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_BSP_BUTTON_GPIO_PIN GPIO_PIN_2
#define BUS_BSP_BUTTON_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()
#define BUS_BSP_BUTTON_GPIO_PORT GPIOB

#define USER_BUTTON_PIN                 GPIO_PIN_2
#define USER_BUTTON_GPIO_PORT           GPIOB
#define USER_BUTTON_EXTI_IRQn           EXTI2_IRQn
#define USER_BUTTON_EXTI_LINE           EXTI_LINE_2
#define H_EXTI_2                        hpb_exti[BUTTON_USER]
/**
 * @}
 */

/** @defgroup SMARTAG2_LOW_LEVEL_COM SMARTAG2 LOW LEVEL COM
 * @{
 */
/**
 * @brief Definition for COM portx, connected to USART1
 */

#define BUS_USART1_INSTANCE USART1
#define BUS_USART1_TX_GPIO_PIN GPIO_PIN_9
#define BUS_USART1_TX_GPIO_PORT GPIOA
#define BUS_USART1_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define BUS_USART1_TX_GPIO_CLK_DISABLE() __HAL_RCC_GPIOA_CLK_DISABLE()
#define BUS_USART1_TX_GPIO_AF GPIO_AF7_USART1
#define BUS_USART1_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define BUS_USART1_RX_GPIO_PORT GPIOA
#define BUS_USART1_RX_GPIO_CLK_DISABLE() __HAL_RCC_GPIOA_CLK_DISABLE()
#define BUS_USART1_RX_GPIO_PIN GPIO_PIN_10
#define BUS_USART1_RX_GPIO_AF GPIO_AF7_USART1

/**
 * @}
 */

/** @defgroup SMARTAG2_LOW_LEVEL_Exported_Types LOW LEVEL Exported Types
  * @{
  */
#ifndef USE_BSP_COM
  #define USE_BSP_COM                           0U
#endif

#ifndef USE_COM_LOG
  #define USE_COM_LOG                           1U
#endif

#ifndef BSP_BUTTON_USER_IT_PRIORITY
  #define BSP_BUTTON_USER_IT_PRIORITY            15U
#endif

typedef enum
{
  LED1 = 0,
  LED_RED   = LED1,
}Led_TypeDef;

typedef enum
{
  BUTTON_USER = 0U,
}Button_TypeDef;

/* Keep compatibility with CMSIS Pack already delivered */
#define BUTTON_KEY BUTTON_USER

typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;

#if (USE_BSP_COM_FEATURE > 0)
typedef enum
{
  COM1 = 0U,
  COMn
}COM_TypeDef;

typedef enum
{
 COM_WORDLENGTH_8B     =   UART_WORDLENGTH_8B,
 COM_WORDLENGTH_9B     =   UART_WORDLENGTH_9B,
}COM_WordLengthTypeDef;

typedef enum
{
 COM_STOPBITS_1     =   UART_STOPBITS_1,
 COM_STOPBITS_2     =   UART_STOPBITS_2,
}COM_StopBitsTypeDef;

typedef enum
{
 COM_PARITY_NONE     =  UART_PARITY_NONE,
 COM_PARITY_EVEN     =  UART_PARITY_EVEN,
 COM_PARITY_ODD      =  UART_PARITY_ODD,
}COM_ParityTypeDef;

typedef enum
{
 COM_HWCONTROL_NONE    =  UART_HWCONTROL_NONE,
 COM_HWCONTROL_RTS     =  UART_HWCONTROL_RTS,
 COM_HWCONTROL_CTS     =  UART_HWCONTROL_CTS,
 COM_HWCONTROL_RTS_CTS =  UART_HWCONTROL_RTS_CTS,
}COM_HwFlowCtlTypeDef;

typedef struct
{
  uint32_t             BaudRate;
  COM_WordLengthTypeDef  WordLength;
  COM_StopBitsTypeDef  StopBits;
  COM_ParityTypeDef    Parity;
  COM_HwFlowCtlTypeDef HwFlowCtl;
}COM_InitTypeDef;
#endif

#define SMARTAG2_UART_InitTypeDef          COM_InitTypeDef
#define SMARTAG2_UART_StopBitsTypeDef      COM_StopBitsTypeDef
#define SMARTAG2_UART_ParityTypeDef        COM_ParityTypeDef
#define SMARTAG2_UART_HwFlowCtlTypeDef     COM_HwFlowCtlTypeDef
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1U)
typedef struct
{
  void (* pMspInitCb)(UART_HandleTypeDef *);
  void (* pMspDeInitCb)(UART_HandleTypeDef *);
} BSP_COM_Cb_t;
#endif /* (USE_HAL_UART_REGISTER_CALLBACKS == 1U) */

/**
 * @}
 */

//#define COMn                             1U
#define COM1_UART                        USART1

#define COM_POLL_TIMEOUT                 1000
//extern UART_HandleTypeDef hcom_uart[COMn];
//#define  BSP_huart1 hcom_uart[COM1]

/**
 * @}
 */

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup SMARTAG2_LOW_LEVEL_Exported_Variables LOW LEVEL Exported Constants
  * @{
  */
extern EXTI_HandleTypeDef hpb_exti[BUTTONn];
/**
  * @}
  */

/** @defgroup SMARTAG2_LOW_LEVEL_Exported_Functions SMARTAG2 LOW LEVEL Exported Functions
 * @{
 */

int32_t  BSP_GetVersion(void);
int32_t  BSP_LED_Init(void);
int32_t  BSP_LED_DeInit(void);
int32_t  BSP_LED_On(void);
int32_t  BSP_LED_Off(void);
int32_t  BSP_LED_Toggle(void);
int32_t  BSP_LED_GetState(void);
int32_t  BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
int32_t  BSP_PB_DeInit(Button_TypeDef Button);
int32_t  BSP_PB_GetState(Button_TypeDef Button);
void     BSP_PB_Callback(Button_TypeDef Button);
void     BSP_PB_IRQHandler (Button_TypeDef Button);
#if (USE_BSP_COM_FEATURE > 0)
int32_t  BSP_COM_Init(COM_TypeDef COM);
int32_t  BSP_COM_DeInit(COM_TypeDef COM);
#endif

#if (USE_COM_LOG > 0)
int32_t  BSP_COM_SelectLogPort(COM_TypeDef COM);
#endif

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1U)
int32_t BSP_COM_RegisterDefaultMspCallbacks(COM_TypeDef COM);
int32_t BSP_COM_RegisterMspCallbacks(COM_TypeDef COM , BSP_COM_Cb_t *Callback);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
#ifdef __cplusplus
}
#endif

#endif /* __SMARTAG2__H */
