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


/* stts22hh Temperature Sensor */
#define BSP_STTS22H_0_I2C_Init          BSP_I2C1_Init
#define BSP_STTS22H_0_I2C_DeInit        BSP_I2C1_DeInit
#define BSP_STTS22H_0_I2C_ReadReg       BSP_I2C1_ReadReg
#define BSP_STTS22H_0_I2C_WriteReg      BSP_I2C1_WriteReg

/* lps22df Temperature and Pressure Sensor */
#define BSP_LPS22DF_0_I2C_Init          BSP_I2C1_Init
#define BSP_LPS22DF_0_I2C_DeInit        BSP_I2C1_DeInit
#define BSP_LPS22DF_0_I2C_ReadReg       BSP_I2C1_ReadReg
#define BSP_LPS22DF_0_I2C_WriteReg      BSP_I2C1_WriteReg

/* h3lis331dl acc Sensor */
#define BSP_H3LIS331DL_0_SPI_Init       BSP_SPI1_Init
#define BSP_H3LIS331DL_0_SPI_DeInit     BSP_SPI1_DeInit
#define BSP_H3LIS331DL_0_SPI_Send       BSP_SPI1_Send
#define BSP_H3LIS331DL_0_SPI_Recv       BSP_SPI1_Recv

#define BSP_H3LIS331DL_0_CS_PORT        GPIOB
#define BSP_H3LIS331DL_0_CS_PIN         GPIO_PIN_1

#define HIGH_G_CS_GPIO_Port             BSP_H3LIS331DL_0_CS_PORT  
#define HIGH_G_CS_Pin                   BSP_H3LIS331DL_0_CS_PIN

/* LIS2DUXS12 acc Sensor */
#define BSP_LIS2DUXS12_0_SPI_Init         BSP_SPI1_Init
#define BSP_LIS2DUXS12_0_SPI_DeInit       BSP_SPI1_DeInit
#define BSP_LIS2DUXS12_0_SPI_Send         BSP_SPI1_Send
#define BSP_LIS2DUXS12_0_SPI_Recv         BSP_SPI1_Recv

#define BSP_LIS2DUXS12_0_CS_PORT          GPIOA
#define BSP_LIS2DUXS12_0_CS_PIN           GPIO_PIN_15

#define ACC_CS_GPIO_Port                BSP_LIS2DUXS12_0_CS_PORT
#define ACC_CS_Pin                      BSP_LIS2DUXS12_0_CS_PIN

/* lsm6dso32x acc and gyro Sensor */
#define BSP_LSM6DSO32X_0_SPI_Init       BSP_SPI1_Init
#define BSP_LSM6DSO32X_0_SPI_DeInit     BSP_SPI1_DeInit
#define BSP_LSM6DSO32X_0_SPI_Send       BSP_SPI1_Send
#define BSP_LSM6DSO32X_0_SPI_Recv       BSP_SPI1_Recv

#define BSP_LSM6DSO32X_0_CS_PORT        GPIOH
#define BSP_LSM6DSO32X_0_CS_PIN         GPIO_PIN_0

#define IMU_CS_GPIO_Port                BSP_LSM6DSO32X_0_CS_PORT
#define IMU_CS_Pin                      BSP_LSM6DSO32X_0_CS_PIN


/* st25dv Dual Interface EEPROM */
#define BSP_ST25DVxxKC_I2C_Init             BSP_I2C3_Init
#define BSP_ST25DVxxKC_I2C_DeInit           BSP_I2C3_DeInit
#define BSP_ST25DVxxKC_I2C_ReadReg16        BSP_I2C3_ReadReg16
#define BSP_ST25DVxxKC_I2C_WriteReg16       BSP_I2C3_WriteReg16
#define BSP_ST25DVxxKC_I2C_Recv             BSP_I2C3_Recv
#define BSP_ST25DVxxKC_I2C_IsReady          BSP_I2C3_IsReady
#define BSP_ST25DVxxKC_GetTick              HAL_GetTick
  
/* vd6283tx Light Sensor */
#define BSP_LIGHT_SENSOR_I2C_Init       BSP_I2C1_Init
#define BSP_LIGHT_SENSOR_I2C_DeInit     BSP_I2C1_DeInit
#define BSP_LIGHT_SENSOR_I2C_ReadReg    BSP_I2C1_ReadReg
#define BSP_LIGHT_SENSOR_I2C_WriteReg   BSP_I2C1_WriteReg
#define BSP_LIGHT_SENSOR_GetTick        BSP_GetTick

/** @defgroup SMARTAG2_VDD_AMB_MCU_PIN SMARTAG2 VDD AMB MCU PIN
 * @{
 */
#define VDD_AMB_MCU_Pin         GPIO_PIN_14
#define VDD_AMB_MCU_GPIO_Port   GPIOB
/**
 * @}
 */

/** @defgroup SMARTAG2_VDD_ACC_MCU_PIN SMARTAG2 VDD ACC MCU PIN
 * @{
 */
#define VDD_ACC_MCU_Pin         GPIO_PIN_15
#define VDD_ACC_MCU_GPIO_Port   GPIOB
/**
 * @}
 */

/** @defgroup SMARTAG2_VDD_EEP_PIN SMARTAG2 VDD EEP MCU PIN
 * @{
 */
#define VDD_EEP_Pin GPIO_PIN_3
#define VDD_EEP_GPIO_Port GPIOA
/**
 * @}
 */
    
/** @defgroup SMARTAG2_LPD_PIN SMARTAG2 LPD PIN
 * @{
 */
#define LPD_Pin         GPIO_PIN_8
#define LPD_GPIO_Port   GPIOB
/**
 * @}
 */

/** @defgroup SMARTAG2_GPO_PIN SMARTAG2 GPO PIN
 * @{
 */
#define GPO_Pin GPIO_PIN_14
#define GPO_GPIO_Port GPIOC
#define GPO_EXTI_LINE EXTI_LINE_14
#define GPO_EXTI_IRQn EXTI15_10_IRQn
   
extern EXTI_HandleTypeDef GPO_EXTI;
#define H_EXTI_14  GPO_EXTI
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

/** @defgroup SMARTAG2_ST25DVxxKC_Exported_Functions
  * @{
  */

void BSP_SmarTag2_ACC_PowerOff(void);
void BSP_SmarTag2_ACC_PowerOn(void);
void BSP_SmarTag2_AMB_PowerOff(void);
void BSP_SmarTag2_AMB_PowerOn(void);
void BSP_SmarTag2_EEP_PowerOff(void);
void BSP_SmarTag2_EEP_PowerOn(void);

int32_t BSP_LPD_Init( void );
int32_t BSP_LPD_DeInit( void );
int32_t BSP_LPD_ReadPin( void );
int32_t BSP_LPD_On( void );
int32_t BSP_LPD_Off( void );
int32_t BSP_LPD_Toggle( void );

int32_t BSP_GPO_Init( void );
int32_t BSP_GPO_DeInit( void );
int32_t BSP_GPO_ReadPin( void );
void BSP_GPO_IRQHandler(void);



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
