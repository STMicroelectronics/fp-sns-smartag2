/**
  ******************************************************************************
  * @file    Smartag2.c
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.0.0
  * @date    31-August-2022
  * @brief   Source file for the BSP Common driver
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

/* Includes ------------------------------------------------------------------*/
#include "SmarTag2.h"
#include "stm32l4xx_hal_exti.h"

/** @defgroup BSP BSP
 * @{
 */

/** @defgroup SMARTAG2 SMARTAG2
 * @{
 */

/** @defgroup SMARTAG2_LOW_LEVEL SMARTAG2 LOW LEVEL
 *  @brief This file provides set of firmware functions to manage Leds and push-button
 *         available on STM32L4xx-Nucleo Kit from STMicroelectronics.
 * @{
 */

/**
 * @}
 */

/** @defgroup SMARTAG2_LOW_LEVEL_Private_Defines SMARTAG2 LOW LEVEL Private Defines
 * @{
 */

/** @defgroup SMARTAG2_LOW_LEVEL_FunctionPrototypes SMARTAG2 LOW LEVEL Private Function Prototypes
 * @{
 */
typedef void (* BSP_EXTI_LineCallback) (void);
typedef void (* BSP_BUTTON_GPIO_Init) (void);

/**
 * @}
 */

/** @defgroup SMARTAG2_LOW_LEVEL_Private_Variables SMARTAG2 LOW LEVEL Private Variables
 * @{
 */
//typedef void (* BSP_LED_GPIO_Init) (void);
//static void LED_USER_GPIO_Init(void);

#ifndef BSP_USART1_BOUNDRATE
  #define BSP_USART1_BOUNDRATE 9600
#endif /* BSP_USART1_BOUNDRATE */

static GPIO_TypeDef*   BUTTON_PORT[BUTTONn] = {USER_BUTTON_GPIO_PORT};
static const uint16_t  BUTTON_PIN[BUTTONn]  = {USER_BUTTON_PIN};
static const IRQn_Type BUTTON_IRQn[BUTTONn] = {USER_BUTTON_EXTI_IRQn};
EXTI_HandleTypeDef hpb_exti[BUTTONn] = {{.Line = EXTI_LINE_2}};
static UART_HandleTypeDef hcom_uart[COMn];
#if (USE_COM_LOG > 0)
static COM_TypeDef COM_ActiveLogPort;
#endif
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1U)
static uint32_t IsUsart1MspCbValid = 0;
#endif
__weak HAL_StatusTypeDef SMARTAG2_USART1_UART_Init(UART_HandleTypeDef* huart);
//__weak void BSP_GPO_Callback(void);
//
//EXTI_HandleTypeDef GPO_EXTI={.Line=GPO_EXTI_LINE};
/**
 * @}
 */

/** @defgroup SMARTAG2_LOW_LEVEL_Private_Functions SMARTAG2 LOW LEVEL Private Functions
 * @{
 */
static void BUTTON_USER_EXTI_Callback(void);
static void BUTTON_USER_GPIO_Init(void);
#if (USE_BSP_COM_FEATURE > 0)
static void USART1_MspInit(UART_HandleTypeDef *huart);
static void USART1_MspDeInit(UART_HandleTypeDef *huart);
#endif
/**
 * @brief  This method returns the STM32L4xx NUCLEO BSP Driver revision
 * @retval version: 0xXYZR (8bits for each decimal, R for RC)
 */
int32_t BSP_GetVersion(void)
{
  return (int32_t)__SMARTAG2_BSP_VERSION;
}

/**
 * @brief  Configures LED on GPIO and/or on MFX.
 * @param  None
 * @retval HAL status
 */
int32_t BSP_LED_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable the GPIO_LED clock */
  LED1_GPIO_CLK_ENABLE();
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = LED1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  
  HAL_GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStruct);
  


  return BSP_ERROR_NONE;
}

/**
 * @brief  DeInit LEDs.
 * @param  None
 * @note Led DeInit does not disable the GPIO clock nor disable the Mfx
 * @retval HAL status
 */
int32_t BSP_LED_DeInit(void)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* Turn off LED */
  HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET);
  /* DeInit the GPIO_LED pin */
  gpio_init_structure.Pin = LED1_PIN;
  HAL_GPIO_DeInit(LED1_GPIO_PORT, gpio_init_structure.Pin);

  return BSP_ERROR_NONE;
}

/**
 * @brief  Turns selected LED On.
 * @param  None
 * @retval HAL status
 */
int32_t BSP_LED_On(void)
{
  HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_SET);

  return BSP_ERROR_NONE;
}

/**
 * @brief  Turns selected LED Off.
 * @param  None
 * @retval HAL status
 */
int32_t BSP_LED_Off(void)
{
  HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET);

  return BSP_ERROR_NONE;
}

/**
 * @brief  Toggles the LED.
 * @param  None
 * @retval HAL status
 */
int32_t BSP_LED_Toggle(void)
{
  HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);

  return BSP_ERROR_NONE;
}

/**
 * @brief  Get the status of the LED.
 * @param  None
 * @retval HAL status (1=high, 0=low)
 */
int32_t BSP_LED_GetState(void)
{
  return (int32_t)(HAL_GPIO_ReadPin (LED1_GPIO_PORT, LED1_PIN) == GPIO_PIN_RESET);
}

/**
  * @brief  Configures button GPIO and EXTI Line.
  * @param  Button: Button to be configured
  *                 This parameter can be one of the following values:
  *                 @arg  BUTTON_USER: User Push Button
  * @param  ButtonMode Button mode
  *                    This parameter can be one of the following values:
  *                    @arg  BUTTON_MODE_GPIO: Button will be used as simple IO
  *                    @arg  BUTTON_MODE_EXTI: Button will be connected to EXTI line
  *                                            with interrupt generation capability
  * @retval BSP status
  */
int32_t BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  int32_t ret = BSP_ERROR_NONE;

  static const BSP_EXTI_LineCallback ButtonCallback[BUTTONn] ={BUTTON_USER_EXTI_Callback};
  static const uint32_t  BSP_BUTTON_PRIO [BUTTONn] ={BSP_BUTTON_USER_IT_PRIORITY};
  static const uint32_t BUTTON_EXTI_LINE[BUTTONn] ={USER_BUTTON_EXTI_LINE};
  static const BSP_BUTTON_GPIO_Init ButtonGpioInit[BUTTONn] = {BUTTON_USER_GPIO_Init};

  ButtonGpioInit[Button]();

  if (ButtonMode == BUTTON_MODE_EXTI)
  {
    if(HAL_EXTI_GetHandle(&hpb_exti[Button], BUTTON_EXTI_LINE[Button]) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else if (HAL_EXTI_RegisterCallback(&hpb_exti[Button],  HAL_EXTI_COMMON_CB_ID, ButtonCallback[Button]) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
	else
    {
      /* Enable and set Button EXTI Interrupt to the lowest priority */
      HAL_NVIC_SetPriority((BUTTON_IRQn[Button]), BSP_BUTTON_PRIO[Button], 0x00);
      HAL_NVIC_EnableIRQ((BUTTON_IRQn[Button]));
    }
  }

  return ret;
}

/**
 * @brief  Push Button DeInit.
 * @param  Button Button to be configured
 *                This parameter can be one of the following values:
 *                @arg  BUTTON_USER: Wakeup Push Button
 * @note PB DeInit does not disable the GPIO clock
 * @retval BSP status
 */
int32_t BSP_PB_DeInit(Button_TypeDef Button)
{
  GPIO_InitTypeDef gpio_init_structure;

  gpio_init_structure.Pin = BUTTON_PIN[Button];
  HAL_NVIC_DisableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  HAL_GPIO_DeInit(BUTTON_PORT[Button], gpio_init_structure.Pin);

  return BSP_ERROR_NONE;
}

/**
 * @brief  Returns the selected button state.
 * @param  Button Button to be addressed
 *                This parameter can be one of the following values:
 *                @arg  BUTTON_USER
 * @retval The Button GPIO pin value (GPIO_PIN_RESET = button pressed)
 */
int32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return (int32_t)(HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]) == GPIO_PIN_RESET);
}

/**
 * @brief  User EXTI line detection callbacks.
 * @retval None
 */
void BSP_PB_IRQHandler (Button_TypeDef Button)
{
  HAL_EXTI_IRQHandler( &hpb_exti[Button] );
}

/**
 * @brief  BSP Push Button callback
 * @param  Button Specifies the pin connected EXTI line
 * @retval None.
 */
__weak void BSP_PB_Callback(Button_TypeDef Button)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Button);

  /* This function should be implemented by the user application.
     It is called into this driver when an event on Button is triggered. */
}

/**
  * @brief  User EXTI line detection callbacks.
  * @retval None
  */
static void BUTTON_USER_EXTI_Callback(void)
{
  BSP_PB_Callback(BUTTON_USER);
}

/**
  * @brief
  * @retval None
  */
static void BUTTON_USER_GPIO_Init(void) {

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PTPIN */
  GPIO_InitStruct.Pin = BUS_BSP_BUTTON_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUS_BSP_BUTTON_GPIO_PORT, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

#if (USE_BSP_COM_FEATURE > 0)
/**
 * @brief  Configures COM port.
 * @param  COM: COM port to be configured.
 *              This parameter can be COM1
 * @param  UART_Init: Pointer to a UART_HandleTypeDef structure that contains the
 *                    configuration information for the specified USART peripheral.
 * @retval BSP error code
 */
int32_t BSP_COM_Init(COM_TypeDef COM)
{
  int32_t ret = BSP_ERROR_NONE;

#if (USE_HAL_UART_REGISTER_CALLBACKS == 0U)
  /* Init the UART Msp */
  USART1_MspInit(&hcom_uart[COM]);
#else
  if(IsUsart1MspCbValid == 0U)
  {
    if(BSP_COM_RegisterDefaultMspCallbacks(COM) != BSP_ERROR_NONE)
    {
      return BSP_ERROR_MSP_FAILURE;
    }
  }
#endif

  if (SMARTAG2_USART1_UART_Init(&hcom_uart[COM]))
  {
    ret = BSP_ERROR_PERIPH_FAILURE;
  }

  return ret;
}

/**
 * @brief  DeInit COM port.
 * @param  COM COM port to be configured.
 *             This parameter can be COM1
 * @retval BSP status
 */
int32_t BSP_COM_DeInit(COM_TypeDef COM)
{
  int32_t ret = BSP_ERROR_NONE;

#if (USE_HAL_UART_REGISTER_CALLBACKS == 0U)
    USART1_MspDeInit(&hcom_uart[COM]);
#endif /* (USE_HAL_UART_REGISTER_CALLBACKS == 0U) */

  if(HAL_UART_DeInit(&hcom_uart[COM]) != HAL_OK)
  {
    ret = BSP_ERROR_PERIPH_FAILURE;
  }

  return ret;
}

/**
 * @brief  Configures COM port.
 * @param  huart USART handle
 *               This parameter can be COM1
 * @param  COM_Init Pointer to a UART_HandleTypeDef structure that contains the
 *                  configuration information for the specified USART peripheral.
 * @retval HAL error code
 */

/* USART1 init function */

__weak HAL_StatusTypeDef SMARTAG2_USART1_UART_Init(UART_HandleTypeDef* huart)
{
  HAL_StatusTypeDef ret = HAL_OK;

  huart->Instance = USART1;
  huart->Init.BaudRate = BSP_USART1_BOUNDRATE;
  huart->Init.WordLength = UART_WORDLENGTH_8B;
  huart->Init.StopBits = UART_STOPBITS_1;
  huart->Init.Parity = UART_PARITY_NONE;
  huart->Init.Mode = UART_MODE_TX_RX;
  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart->Init.OverSampling = UART_OVERSAMPLING_16;
  huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(huart) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  if (HAL_UARTEx_SetTxFifoThreshold(huart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  if (HAL_UARTEx_SetRxFifoThreshold(huart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  if (HAL_UARTEx_DisableFifoMode(huart) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  return ret;
}

#endif
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1U)
/**
 * @brief Register Default USART1 Bus Msp Callbacks
 * @retval BSP status
 */
int32_t BSP_COM_RegisterDefaultMspCallbacks(COM_TypeDef COM)
{
  int32_t ret = BSP_ERROR_NONE;

  if(COM >= COMn)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {

    __HAL_UART_RESET_HANDLE_STATE(&hcom_uart[COM]);

    /* Register default MspInit/MspDeInit Callback */
    if(HAL_UART_RegisterCallback(&hcom_uart[COM], HAL_UART_MSPINIT_CB_ID, USART1_MspInit) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else if(HAL_UART_RegisterCallback(&hcom_uart[COM], HAL_UART_MSPDEINIT_CB_ID, USART1_MspDeInit) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {
      IsUsart1MspCbValid = 1U;
    }
  }

  /* BSP status */
  return ret;
}

/**
 * @brief Register USART1 Bus Msp Callback registering
 * @param Callbacks pointer to USART1 MspInit/MspDeInit callback functions
 * @retval BSP status
 */
int32_t BSP_COM_RegisterMspCallbacks (COM_TypeDef COM , BSP_COM_Cb_t *Callback)
{
  int32_t ret = BSP_ERROR_NONE;

  if(COM >= COMn)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    __HAL_UART_RESET_HANDLE_STATE(&hcom_uart[COM]);

    /* Register MspInit/MspDeInit Callbacks */
    if(HAL_UART_RegisterCallback(&hcom_uart[COM], HAL_UART_MSPINIT_CB_ID, Callback->pMspInitCb) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else if(HAL_UART_RegisterCallback(&hcom_uart[COM], HAL_UART_MSPDEINIT_CB_ID, Callback->pMspDeInitCb) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {
      IsUsart1MspCbValid = 1U;
    }
  }

  /* BSP status */
  return ret;
}
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */

#if (USE_COM_LOG > 0)
/**
 * @brief  Select the active COM port.
 * @param  COM COM port to be activated.
 *             This parameter can be COM1
 * @retval BSP status
 */
int32_t BSP_COM_SelectLogPort(COM_TypeDef COM)
{
  if(COM_ActiveLogPort != COM)
  {
    COM_ActiveLogPort = COM;
  }
  return BSP_ERROR_NONE;
}

#if defined (__IAR_SYSTEMS_ICC__)
size_t __write(int handle, const unsigned char * buffer, size_t size)
{
  (void)HAL_UART_Transmit(&hcom_uart[COM_ActiveLogPort], (uint8_t *)buffer, 1, COM_POLL_TIMEOUT);
  return size;
}
#endif

#if defined (__ARMCC_VERSION)
extern UART_HandleTypeDef hcom_uart[];
/**
* @brief fputc call for standard output implementation
* @param ch Character to print
* @param f File pointer
* @retval Character printed
*/
int fputc(int ch, FILE *f)
{
  (void) HAL_UART_Transmit(&hcom_uart[COM_ActiveLogPort], (uint8_t*) &ch, 1, COM_POLL_TIMEOUT);
  return ch;
}



/** @brief fgetc call for standard input implementation
* @param f File pointer
* @retval Character acquired from standard input
*/
int fgetc(FILE *f)
{
  int ch;
    (void)HAL_UART_Receive(&hcom_uart[COM_ActiveLogPort], (uint8_t*) &ch, 1, COM_POLL_TIMEOUT);
    return ch;
}
#endif

#if defined (__GNUC__)
 int __io_putchar (int ch)
{
  (void)HAL_UART_Transmit(&hcom_uart[COM_ActiveLogPort], (uint8_t *)&ch, 1, COM_POLL_TIMEOUT);
  return ch;
}
#endif

#endif /* USE_COM_LOG */
/**
 * @brief  Initializes USART1 MSP.
 * @param  huart USART1 handle
 * @retval None
 */

static void USART1_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* Enable Peripheral clock */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = BUS_USART1_TX_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BUS_USART1_TX_GPIO_AF;
    HAL_GPIO_Init(BUS_USART1_TX_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUS_USART1_RX_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BUS_USART1_RX_GPIO_AF;
    HAL_GPIO_Init(BUS_USART1_RX_GPIO_PORT, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
}

static void USART1_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(BUS_USART1_TX_GPIO_PORT, BUS_USART1_TX_GPIO_PIN);

    HAL_GPIO_DeInit(BUS_USART1_RX_GPIO_PORT, BUS_USART1_RX_GPIO_PIN);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
}

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
