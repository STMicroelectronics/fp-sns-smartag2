/**
  ******************************************************************************
  * @file    Smartag2_light_sensor.c
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.0.0
  * @date    31-August-2022
  * @brief   This file includes the driver for the light sensor mounted on the
  *          Smartag2 board.
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

/* Includes ------------------------------------------------------------------
* */

#include "Smartag2_light_sensor.h"

/** @addtogroup BSP
  * @{
  */

/** @defgroup BSP_LIGHT_SENSOR LIGHT_SENSOR
  * @{
  */


/** @defgroup BSP_LIGHT_SENSOR_Exported_Types Exported Types
  * @{
  */
void *BSP_LIGHT_SENSOR_CompObj[LIGHT_SENSOR_INSTANCES_NBR] = {0};

/**
  * @}
  */

/** @defgroup BSP_LIGHT_SENSOR_Private_Variables Private Variables
  * @{
  */
static LIGHT_SENSOR_Drv_t *BSP_LIGHT_SENSOR_Drv = NULL;
static LIGHT_SENSOR_Capabilities_t BSP_LIGHT_SENSOR_Cap[LIGHT_SENSOR_INSTANCES_NBR];

/**
  * @}
  */

static int32_t VD6283TX_Probe(uint32_t Instance);
static int32_t vd6283tx_i2c_recover(void);

/**
  * @brief Initializes the light sensor.
  * @param Instance    Light sensor instance.
  * @retval BSP status
  */
int32_t BSP_LIGHT_SENSOR_Init(uint32_t Instance)
{
  int32_t ret;

  if (Instance >= LIGHT_SENSOR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* run i2c recovery before probing the device */
    (void)vd6283tx_i2c_recover();
    ret = VD6283TX_Probe(Instance);
  }

  return ret;
}

/**
  * @brief Deinitializes the light sensor.
  * @param Instance    Light sensor instance.
  * @retval BSP status
  */
int32_t BSP_LIGHT_SENSOR_DeInit(uint32_t Instance)
{
  int32_t ret;

  if (Instance >= LIGHT_SENSOR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (BSP_LIGHT_SENSOR_Drv->DeInit(BSP_LIGHT_SENSOR_CompObj[Instance]) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Read the light sensor device ID.
  * @param Instance    Light sensor instance.
  * @param pId    Pointer to the device ID.
  * @retval BSP status
  */
int32_t BSP_LIGHT_SENSOR_ReadID(uint32_t Instance, uint32_t *pId)
{
  int32_t ret;

  if (Instance >= LIGHT_SENSOR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (BSP_LIGHT_SENSOR_Drv->ReadID(BSP_LIGHT_SENSOR_CompObj[Instance], pId) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Get the light sensor capabilities.
  * @param Instance    Light sensor instance.
  * @param pCapabilities    Pointer to the light sensor capabilities.
  * @retval BSP status
  */
int32_t BSP_LIGHT_SENSOR_GetCapabilities(uint32_t Instance, LIGHT_SENSOR_Capabilities_t *pCapabilities)
{
  int32_t ret;

  if (Instance >= LIGHT_SENSOR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (BSP_LIGHT_SENSOR_Drv->GetCapabilities(BSP_LIGHT_SENSOR_CompObj[Instance], pCapabilities) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Set the exposure time.
  * @param Instance    Light sensor instance.
  * @param ExposureTime    New exposure time to be applied.
  * @warning This function must not be called when a capture is ongoing.
  * @retval BSP status
  */
int32_t BSP_LIGHT_SENSOR_SetExposureTime(uint32_t Instance, uint32_t ExposureTime)
{
  int32_t ret;

  if (Instance >= LIGHT_SENSOR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (BSP_LIGHT_SENSOR_Drv->SetExposureTime(BSP_LIGHT_SENSOR_CompObj[Instance], ExposureTime) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Get the exposure time.
  * @param Instance    Light sensor instance.
  * @param pExposureTime    Pointer to the current exposure time value.
  * @retval BSP status
  */
int32_t BSP_LIGHT_SENSOR_GetExposureTime(uint32_t Instance, uint32_t *pExposureTime)
{
  int32_t ret;

  if (Instance >= LIGHT_SENSOR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (BSP_LIGHT_SENSOR_Drv->GetExposureTime(BSP_LIGHT_SENSOR_CompObj[Instance], pExposureTime) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Set the gain of a channel.
  * @param Instance    Light sensor instance.
  * @param Channel    Device channel.
  * @param Gain    New gain to be applied on the provided channel.
  * @warning This function must not be called when a capture is ongoing.
  * @retval BSP status
  */
int32_t BSP_LIGHT_SENSOR_SetGain(uint32_t Instance, uint8_t Channel, uint32_t Gain)
{
  int32_t ret;

  if (Instance >= LIGHT_SENSOR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (BSP_LIGHT_SENSOR_Drv->SetGain(BSP_LIGHT_SENSOR_CompObj[Instance], Channel, Gain) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Get the current gain of a channel.
  * @param Instance    Light sensor instance.
  * @param Channel    Device channel.
  * @param pGain    Pointer to the current gain value.
  * @retval BSP status
  */
int32_t BSP_LIGHT_SENSOR_GetGain(uint32_t Instance, uint8_t Channel, uint32_t *pGain)
{
  int32_t ret;

  if (Instance >= LIGHT_SENSOR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (BSP_LIGHT_SENSOR_Drv->GetGain(BSP_LIGHT_SENSOR_CompObj[Instance], Channel, pGain) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Set the inter-measurement time.
  * @param Instance    Light sensor instance.
  * @param InterMeasurementTime    Inter-measurement to be applied.
  * @note This should be configured only when using the device in continuous mode.
  * @warning This function must not be called when a capture is ongoing.
  * @retval BSP status
  */
int32_t BSP_LIGHT_SENSOR_SetInterMeasurementTime(uint32_t Instance, uint32_t InterMeasurementTime)
{
  int32_t ret;

  if (Instance >= LIGHT_SENSOR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (BSP_LIGHT_SENSOR_Drv->SetInterMeasurementTime(BSP_LIGHT_SENSOR_CompObj[Instance], InterMeasurementTime) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Get the inter-measurement time.
  * @param Instance    Light sensor instance.
  * @param pInterMeasurementTime    Pointer to the current inter-measurement time.
  * @retval BSP status
  */
int32_t BSP_LIGHT_SENSOR_GetInterMeasurementTime(uint32_t Instance, uint32_t *pInterMeasurementTime)
{
  int32_t ret;

  if (Instance >= LIGHT_SENSOR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (BSP_LIGHT_SENSOR_Drv->GetInterMeasurementTime(BSP_LIGHT_SENSOR_CompObj[Instance], pInterMeasurementTime) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Start the light measurement on all channels.
  * @param Instance    Light sensor instance.
  * @param Mode    Measurement mode (continuous or single-shot)
  * @retval BSP status
  */
int32_t BSP_LIGHT_SENSOR_Start(uint32_t Instance, uint8_t Mode)
{
  int32_t ret;

  if (Instance >= LIGHT_SENSOR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (BSP_LIGHT_SENSOR_Drv->Start(BSP_LIGHT_SENSOR_CompObj[Instance], Mode) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Stop the measurement on all channels.
  * @param Instance    Light sensor instance.
  * @retval BSP status
  */
int32_t BSP_LIGHT_SENSOR_Stop(uint32_t Instance)
{
  int32_t ret;

  if (Instance >= LIGHT_SENSOR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (BSP_LIGHT_SENSOR_Drv->Stop(BSP_LIGHT_SENSOR_CompObj[Instance]) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Start flicker capture.
  * @param Instance    Light sensor instance.
  * @param Channel    The channel that will be used for flicker detection.
  * @param OutputMode    Analog or Digital depending on the hardware configuration.
  * @note The application must call BSP_LIGHT_SENSOR_Start before calling this function.
  * @warning The flicker can be started only on one channel at a time.
  * @note Calling this function will enable ALS capture on all the other channels.
  * @retval BSP status
  */
int32_t BSP_LIGHT_SENSOR_StartFlicker(uint32_t Instance, uint8_t Channel, uint8_t OutputMode)
{
  int32_t ret;

  if (Instance >= LIGHT_SENSOR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (BSP_LIGHT_SENSOR_Drv->StartFlicker(BSP_LIGHT_SENSOR_CompObj[Instance], Channel, OutputMode) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Stop flicker capture.
  * @param Instance     Light sensor instance.
  * @retval BSP status
  */
int32_t BSP_LIGHT_SENSOR_StopFlicker(uint32_t Instance)
{
  int32_t ret;

  if (Instance >= LIGHT_SENSOR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (BSP_LIGHT_SENSOR_Drv->StopFlicker(BSP_LIGHT_SENSOR_CompObj[Instance]) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Returns the measurement values for all the channels.
  * @param Instance    Light sensor instance.
  * @param pResult    Pointer to an array which will be filled with the values of each channel.
  * @note The array size must match the number of channels of the device.
  * @retval BSP status
  */
int32_t BSP_LIGHT_SENSOR_GetValues(uint32_t Instance, uint32_t *pResult)
{
  int32_t ret;

  if (Instance >= LIGHT_SENSOR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (BSP_LIGHT_SENSOR_Drv->GetValues(BSP_LIGHT_SENSOR_CompObj[Instance], pResult) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Enable and disable control features.
  * @param Instance    Light sensor instance.
  * @param ControlMode    Feature to be be enabled or disabled.
  * @param Value    Value to be applied.
  * @warning This function must not be called when a capture is ongoing.
  * @retval BSP status
  */
int32_t BSP_LIGHT_SENSOR_SetControlMode(uint32_t Instance, uint32_t ControlMode, uint32_t Value)
{
  int32_t ret;

  if (Instance >= LIGHT_SENSOR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (BSP_LIGHT_SENSOR_Drv->SetControlMode(BSP_LIGHT_SENSOR_CompObj[Instance], ControlMode, Value) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

 /**
  * @brief Get saturation value from the device.
  * @param Instance    Light sensor instance.
  * @param pValue    Pointer to the variable where the saturation value is stored.
  * @warning The saturation value is reset when the device is stopped.
  * @retval BSP status
  */
int32_t BSP_LIGHT_SENSOR_GetSaturation(uint32_t Instance, uint32_t *pValue)
{
  int32_t ret;

  if (Instance >= LIGHT_SENSOR_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (VD6283TX_GetSaturation((VD6283TX_Object_t *)BSP_LIGHT_SENSOR_CompObj[Instance], pValue) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/** @defgroup BSP_LIGHT_SENSOR_Private_Functions Private Functions
  * @{
  */

/**
  * @brief Register Bus IOs if component ID is OK.
  * @param Instance    Light sensor instance.
  * @retval BSP status
  */
static int32_t VD6283TX_Probe(uint32_t Instance)
{
  int32_t ret;
  uint32_t id;
  VD6283TX_IO_t IOCtx;
  static VD6283TX_Object_t VD6283TXObj[LIGHT_SENSOR_INSTANCES_NBR];

  /* Configure the light sensor driver */
  IOCtx.Address     = LIGHT_SENSOR_VD6283TX_ADDRESS;
  IOCtx.Init        = BSP_LIGHT_SENSOR_I2C_Init;
  IOCtx.DeInit      = BSP_LIGHT_SENSOR_I2C_DeInit;
  IOCtx.WriteReg    = BSP_LIGHT_SENSOR_I2C_WriteReg;
  IOCtx.ReadReg     = BSP_LIGHT_SENSOR_I2C_ReadReg;
  IOCtx.GetTick     = BSP_GetTick;

  if (VD6283TX_RegisterBusIO(&(VD6283TXObj[Instance]), &IOCtx) != VD6283TX_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else if (VD6283TX_ReadID(&(VD6283TXObj[Instance]), &id) != VD6283TX_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    if (id != VD6283TX_DEVICE_ID)
    {
      ret = BSP_ERROR_UNKNOWN_COMPONENT;
    }
    else
    {
      BSP_LIGHT_SENSOR_Drv = (LIGHT_SENSOR_Drv_t *) &VD6283TX_LIGHT_SENSOR_Driver;
      BSP_LIGHT_SENSOR_CompObj[Instance] = &(VD6283TXObj[Instance]);

      if (BSP_LIGHT_SENSOR_Drv->Init(BSP_LIGHT_SENSOR_CompObj[Instance]) != VD6283TX_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else if (BSP_LIGHT_SENSOR_Drv->GetCapabilities(BSP_LIGHT_SENSOR_CompObj[Instance], &BSP_LIGHT_SENSOR_Cap[Instance])
               != VD6283TX_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
  }

  return ret;
}

/**
  * @brief Reset i2c line to idle state
  */
static int32_t vd6283tx_i2c_recover(void)
{
  /* We can't assume bus state based on SDA and SCL state (we may be in a data or NAK bit so SCL=SDA=1)
  * by setting SDA high and toggling SCL at least 10 time we ensure whatever agent and state
  * all agent should end up seeing a "stop" and bus get back to an known idle i2c  bus state */

  uint8_t i;
  uint8_t retry_cnt = 0;
  static uint8_t is_already_init = 0U;

  if (is_already_init == 1U)
  {
    return BSP_ERROR_NONE;
  }

 

  HAL_GPIO_WritePin(BUS_I2C_SCL_GPIO_PORT, BUS_I2C_SCL_GPIO_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BUS_I2C_SDA_GPIO_PORT, BUS_I2C_SDA_GPIO_PIN, GPIO_PIN_SET);

  do
  {
    for (i = 0; i < 10U; i++)
    {
      HAL_GPIO_WritePin(BUS_I2C_SCL_GPIO_PORT, BUS_I2C_SCL_GPIO_PIN, GPIO_PIN_RESET);
      HAL_Delay(1);
      HAL_GPIO_WritePin(BUS_I2C_SCL_GPIO_PORT, BUS_I2C_SCL_GPIO_PIN, GPIO_PIN_SET);
      HAL_Delay(1);
    }
    retry_cnt++;
  } while ((HAL_GPIO_ReadPin(BUS_I2C_SDA_GPIO_PORT, BUS_I2C_SDA_GPIO_PIN) == GPIO_PIN_RESET) && (retry_cnt < 7U));

  if (HAL_GPIO_ReadPin(BUS_I2C_SDA_GPIO_PORT, BUS_I2C_SDA_GPIO_PIN) == GPIO_PIN_RESET)
  {
    /* We are still in a bad i2c state, return error */
    return BSP_ERROR_COMPONENT_FAILURE;
  }

  is_already_init = 1U;

  return BSP_ERROR_NONE;
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

