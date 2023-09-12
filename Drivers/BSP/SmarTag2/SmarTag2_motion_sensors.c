/**
  ******************************************************************************
  * @file    SmarTag2_motion_sensors.c
  * @author  System Research & Applications Team - Catania & Agrate Lab.
  * @version 1.0.0
  * @date    31-August-2022
  * @brief   This file provides BSP Motion Sensors interface for SmarTag2 board
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
#include "SmarTag2_motion_sensors.h"

/* Exported Variables --------------------------------------------------------*/
extern void *MotionCompObj[MOTION_INSTANCES_NBR]; /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
void *MotionCompObj[MOTION_INSTANCES_NBR];

/* Private function prototypes -----------------------------------------------*/

/* We define a jump table in order to get the correct index from the desired function. */
/* This table should have a size equal to the maximum value of a function plus 1.      */
static uint32_t FunctionIndex[5] = {0, 0, 1, 1, 2};
static MOTION_SENSOR_FuncDrv_t *MotionFuncDrv[MOTION_INSTANCES_NBR][MOTION_FUNCTIONS_NBR];
static MOTION_SENSOR_CommonDrv_t *MotionDrv[MOTION_INSTANCES_NBR];
static MOTION_SENSOR_Ctx_t MotionCtx[MOTION_INSTANCES_NBR];

#if (USE_MOTION_SENSOR_LIS2DUXS12_0 == 1)
static int32_t LIS2DUXS12_0_Probe(uint32_t Functions);
#endif
#if (USE_MOTION_SENSOR_H3LIS331DL_0 == 1)
static int32_t H3LIS331DL_0_Probe(uint32_t Functions);
#endif
#if (USE_MOTION_SENSOR_LSM6DSO32X_0 == 1)
static int32_t LSM6DSO32X_0_Probe(uint32_t Functions);
#endif

#if (USE_MOTION_SENSOR_LIS2DUXS12_0 == 1)
static int32_t BSP_LIS2DUXS12_0_Init(void);
static int32_t BSP_LIS2DUXS12_0_DeInit(void);
static int32_t BSP_LIS2DUXS12_0_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len);
static int32_t BSP_LIS2DUXS12_0_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len);
#endif

#if (USE_MOTION_SENSOR_H3LIS331DL_0 == 1)
static int32_t BSP_H3LIS331DL_0_Init(void);
static int32_t BSP_H3LIS331DL_0_DeInit(void);
static int32_t BSP_H3LIS331DL_0_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len);
static int32_t BSP_H3LIS331DL_0_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len);
#endif

#if (USE_MOTION_SENSOR_LSM6DSO32X_0 == 1)
static int32_t BSP_LSM6DSO32X_0_Init(void);
static int32_t BSP_LSM6DSO32X_0_DeInit(void);
static int32_t BSP_LSM6DSO32X_0_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len);
static int32_t BSP_LSM6DSO32X_0_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len);
#endif

/**
  * @brief  This function power off accelerometer sensors on board the SMARTAG2
  * @param  None
  * @retval None
  */
void BSP_MOTION_SENSOR_PowerOff(void)
{
    HAL_GPIO_DeInit(BSP_LSM6DSO32X_0_CS_PORT, BSP_LSM6DSO32X_0_CS_PIN);
    HAL_GPIO_DeInit(BSP_LIS2DUXS12_0_CS_PORT, BSP_LIS2DUXS12_0_CS_PIN);
    HAL_GPIO_DeInit(BSP_H3LIS331DL_0_CS_PORT, BSP_H3LIS331DL_0_CS_PIN);
    HAL_SPI_DeInit(&HANDLE_SPI);
    HAL_GPIO_WritePin(VDD_ACC_MCU_GPIO_Port, VDD_ACC_MCU_Pin, GPIO_PIN_RESET);
  HAL_Delay(20);
}

/**
  * @brief  This function power on accelerometer sensors on board the SMARTAG2
  * @param  None
  * @retval None
  */
void BSP_MOTION_SENSOR_PowerOn(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    HAL_GPIO_DeInit(BSP_LSM6DSO32X_0_CS_PORT, BSP_LSM6DSO32X_0_CS_PIN);
    HAL_GPIO_DeInit(BSP_LIS2DUXS12_0_CS_PORT, BSP_LIS2DUXS12_0_CS_PIN);
    HAL_GPIO_DeInit(BSP_H3LIS331DL_0_CS_PORT, BSP_H3LIS331DL_0_CS_PIN);

    HAL_GPIO_WritePin(BSP_LSM6DSO32X_0_CS_PORT, BSP_LSM6DSO32X_0_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BSP_LIS2DUXS12_0_CS_PORT, BSP_LIS2DUXS12_0_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BSP_H3LIS331DL_0_CS_PORT, BSP_H3LIS331DL_0_CS_PIN, GPIO_PIN_SET);

    HAL_GPIO_WritePin(VDD_ACC_MCU_GPIO_Port, VDD_ACC_MCU_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = BSP_LSM6DSO32X_0_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BSP_LSM6DSO32X_0_CS_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BSP_LIS2DUXS12_0_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BSP_LIS2DUXS12_0_CS_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BSP_H3LIS331DL_0_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BSP_H3LIS331DL_0_CS_PORT, &GPIO_InitStruct);
  HAL_Delay(20);
}

/**
  * @brief  Initializes the motion sensors
  * @param  Instance Motion sensor instance
  * @param  Functions Motion sensor functions. Could be :
  *         - MOTION_GYRO
  *         - MOTION_ACCELERO
  *         - MOTION_MAGNETO
  * @retval BSP status
  */
int32_t BSP_MOTION_SENSOR_Init(uint32_t Instance, uint32_t Functions)
{
  int32_t ret = BSP_ERROR_NONE;
  uint32_t function = MOTION_GYRO;
  uint32_t i;
  uint32_t component_functions = 0;
  MOTION_SENSOR_Capabilities_t cap;

  switch (Instance)
  {
#if (USE_MOTION_SENSOR_LIS2DUXS12_0 == 1)
    case LIS2DUXS12_0:
      if (LIS2DUXS12_0_Probe(Functions) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_NO_INIT;
      }
      if (MotionDrv[Instance]->GetCapabilities(MotionCompObj[Instance], (void *)&cap) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_UNKNOWN_COMPONENT;
      }
      if (cap.Acc == 1U)
      {
        component_functions |= MOTION_ACCELERO;
      }
      break;
#endif
#if (USE_MOTION_SENSOR_H3LIS331DL_0 == 1)
    case H3LIS331DL_0:
      if (H3LIS331DL_0_Probe(Functions) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_NO_INIT;
      }
      if (MotionDrv[Instance]->GetCapabilities(MotionCompObj[Instance], (void *)&cap) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_UNKNOWN_COMPONENT;
      }
      if (cap.Acc == 1U)
      {
        component_functions |= MOTION_ACCELERO;
      }
      break;
#endif
#if (USE_MOTION_SENSOR_LSM6DSO32X_0 == 1)
    case LSM6DSO32X_0:
      if (LSM6DSO32X_0_Probe(Functions) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_NO_INIT;
      }
      if (MotionDrv[Instance]->GetCapabilities(MotionCompObj[Instance], (void *)&cap) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_UNKNOWN_COMPONENT;
      }
      if (cap.Acc == 1U)
      {
        component_functions |= MOTION_ACCELERO;
      }
      if (cap.Gyro == 1U)
      {
        component_functions |= MOTION_GYRO;
      }
      break;
#endif
    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }

  for (i = 0; i < MOTION_FUNCTIONS_NBR; i++)
  {
    if (((Functions & function) == function) && ((component_functions & function) == function))
    {
      if (MotionFuncDrv[Instance][FunctionIndex[function]]->Enable(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_COMPONENT_FAILURE;
      }
    }
    function = function << 1;
  }

  return ret;
}

/**
 * @brief  Deinitialize Motion sensor
 * @param  Instance Motion sensor instance
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_DeInit(uint32_t Instance)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (MotionDrv[Instance]->DeInit(MotionCompObj[Instance]) != BSP_ERROR_NONE)
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
 * @brief  Get motion sensor instance capabilities
 * @param  Instance Motion sensor instance
 * @param  Capabilities pointer to motion sensor capabilities
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_GetCapabilities(uint32_t Instance, MOTION_SENSOR_Capabilities_t *Capabilities)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (MotionDrv[Instance]->GetCapabilities(MotionCompObj[Instance], Capabilities) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
 * @brief  Get WHOAMI value
 * @param  Instance Motion sensor instance
 * @param  Id WHOAMI value
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_ReadID(uint32_t Instance, uint8_t *Id)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (MotionDrv[Instance]->ReadID(MotionCompObj[Instance], Id) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
 * @brief  Enable Motion sensor
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 *         - MOTION_MAGNETO
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_Enable(uint32_t Instance, uint32_t Function)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->Enable(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Disable Motion sensor
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 *         - MOTION_MAGNETO
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_Disable(uint32_t Instance, uint32_t Function)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->Disable(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Get motion sensor axes data
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 *         - MOTION_MAGNETO
 * @param  Axes pointer to axes data structure
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_GetAxes(uint32_t Instance, uint32_t Function, BSP_MOTION_SENSOR_Axes_t *Axes)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetAxes(MotionCompObj[Instance], Axes) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Get motion sensor axes raw data
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 *         - MOTION_MAGNETO
 * @param  Axes pointer to axes raw data structure
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_GetAxesRaw(uint32_t Instance, uint32_t Function, BSP_MOTION_SENSOR_AxesRaw_t *Axes)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetAxesRaw(MotionCompObj[Instance], Axes) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Get motion sensor sensitivity
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 *         - MOTION_MAGNETO
 * @param  Sensitivity pointer to sensitivity read value
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_GetSensitivity(uint32_t Instance, uint32_t Function, float *Sensitivity)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetSensitivity(MotionCompObj[Instance],
          Sensitivity) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Get motion sensor Output Data Rate
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 *         - MOTION_MAGNETO
 * @param  Odr pointer to Output Data Rate read value
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_GetOutputDataRate(uint32_t Instance, uint32_t Function, float *Odr)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetOutputDataRate(MotionCompObj[Instance], Odr) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Get motion sensor Full Scale
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 *         - MOTION_MAGNETO
 * @param  Fullscale pointer to Fullscale read value
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_GetFullScale(uint32_t Instance, uint32_t Function, int32_t *Fullscale)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetFullScale(MotionCompObj[Instance], Fullscale) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Set motion sensor Output Data Rate
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 *         - MOTION_MAGNETO
 * @param  Odr Output Data Rate value to be set
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_SetOutputDataRate(uint32_t Instance, uint32_t Function, float Odr)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->SetOutputDataRate(MotionCompObj[Instance], Odr) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Set motion sensor Full Scale
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 *         - MOTION_MAGNETO
 * @param  Fullscale Fullscale value to be set
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_SetFullScale(uint32_t Instance, uint32_t Function, int32_t Fullscale)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->SetFullScale(MotionCompObj[Instance], Fullscale) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

#if (USE_MOTION_SENSOR_LIS2DUXS12_0  == 1)
/**
 * @brief  Register Bus IOs for LIS2DUXS12 instance
 * @param  Functions Motion sensor functions. Could be :
 *         - MOTION_GYRO and/or MOTION_ACCELERO
 * @retval BSP status
 */
static int32_t LIS2DUXS12_0_Probe(uint32_t Functions)
{
  LIS2DUXS12_IO_t            io_ctx;
  uint8_t                  id;
  static LIS2DUXS12_Object_t lis2du12_obj_0;
  LIS2DUXS12_Capabilities_t  cap;
  int32_t                  ret = BSP_ERROR_NONE;

  /* Configure the driver */
  io_ctx.BusType     = LIS2DUXS12_SPI_4WIRES_BUS; /* SPI 4-Wires */
  io_ctx.Address     = 0x0;
  io_ctx.Init        = BSP_LIS2DUXS12_0_Init;
  io_ctx.DeInit      = BSP_LIS2DUXS12_0_DeInit;
  io_ctx.ReadReg     = BSP_LIS2DUXS12_0_ReadReg;
  io_ctx.WriteReg    = BSP_LIS2DUXS12_0_WriteReg;
  io_ctx.GetTick     = BSP_GetTick;

  if (LIS2DUXS12_RegisterBusIO(&lis2du12_obj_0, &io_ctx) != LIS2DUXS12_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  } else {        
    if(lis2duxs12_exit_deep_power_down(&(lis2du12_obj_0.Ctx)) != LIS2DUXS12_OK)
    {
      ret = BSP_ERROR_UNKNOWN_COMPONENT;
    } else {
      //Wait time to boot
      HAL_Delay(30);
     if (LIS2DUXS12_ReadID(&lis2du12_obj_0, &id) != LIS2DUXS12_OK)
      {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
      }
      else if (id != LIS2DUXS12_ID)
      {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
      }
      else
      {
        (void)LIS2DUXS12_GetCapabilities(&lis2du12_obj_0, &cap);
        MotionCtx[LIS2DUXS12_0].Functions = ((uint32_t)cap.Gyro) | ((uint32_t)cap.Acc << 1) | ((uint32_t)cap.Magneto << 2);

        MotionCompObj[LIS2DUXS12_0] = &lis2du12_obj_0;
        /* The second cast (void *) is added to bypass Misra R11.3 rule */
        MotionDrv[LIS2DUXS12_0] = (MOTION_SENSOR_CommonDrv_t *)(void *)&LIS2DUXS12_COMMON_Driver;

        if ((ret == BSP_ERROR_NONE) && ((Functions & MOTION_ACCELERO) == MOTION_ACCELERO) && (cap.Acc == 1U))
        {
          /* The second cast (void *) is added to bypass Misra R11.3 rule */
          MotionFuncDrv[LIS2DUXS12_0][FunctionIndex[MOTION_ACCELERO]] = (MOTION_SENSOR_FuncDrv_t *)(void *)&LIS2DUXS12_ACC_Driver;

          if (MotionDrv[LIS2DUXS12_0]->Init(MotionCompObj[LIS2DUXS12_0]) != LIS2DUXS12_OK)
          {
            ret = BSP_ERROR_COMPONENT_FAILURE;
          }
          else
          {
            ret = BSP_ERROR_NONE;
          }
        }
        if ((ret == BSP_ERROR_NONE) && ((Functions & MOTION_GYRO) == MOTION_GYRO))
        {
          /* Return an error if the application try to initialize a function not supported by the component */
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        if ((ret == BSP_ERROR_NONE) && ((Functions & MOTION_MAGNETO) == MOTION_MAGNETO))
        {
          /* Return an error if the application try to initialize a function not supported by the component */
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
      }
    }
  }
  return ret;
}

/**
 * @brief  Initialize SPI bus for LIS2DUXS12
 * @retval BSP status
 */
static int32_t BSP_LIS2DUXS12_0_Init(void)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if(BSP_LIS2DUXS12_0_SPI_Init() == BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
 * @brief  DeInitialize SPI bus for LIS2DUXS12
 * @retval BSP status
 */
static int32_t BSP_LIS2DUXS12_0_DeInit(void)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if(BSP_LIS2DUXS12_0_SPI_DeInit() == BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
 * @brief  Write register by SPI bus for LIS2DUXS12
 * @param  Addr not used, it is only for BSP compatibility
 * @param  Reg the starting register address to be written
 * @param  pdata the pointer to the data to be written
 * @param  len the length of the data to be written
 * @retval BSP status
 */
static int32_t BSP_LIS2DUXS12_0_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;

  /* CS Enable */
  HAL_GPIO_WritePin(BSP_LIS2DUXS12_0_CS_PORT, BSP_LIS2DUXS12_0_CS_PIN, GPIO_PIN_RESET);

  if (BSP_LIS2DUXS12_0_SPI_Send(&dataReg, 1) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (BSP_LIS2DUXS12_0_SPI_Send(pdata, len) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  /* CS Disable */
  HAL_GPIO_WritePin(BSP_LIS2DUXS12_0_CS_PORT, BSP_LIS2DUXS12_0_CS_PIN, GPIO_PIN_SET);

  return ret;
}

/**
 * @brief  Read register by SPI bus for LIS2DUXS12
 * @param  Addr not used, it is only for BSP compatibility
 * @param  Reg the starting register address to be read
 * @param  pdata the pointer to the data to be read
 * @param  len the length of the data to be read
 * @retval BSP status
 */
static int32_t BSP_LIS2DUXS12_0_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;

  dataReg |= 0x80U;

  /* CS Enable */
  HAL_GPIO_WritePin(BSP_LIS2DUXS12_0_CS_PORT, BSP_LIS2DUXS12_0_CS_PIN, GPIO_PIN_RESET);

  if (BSP_LIS2DUXS12_0_SPI_Send(&dataReg, 1) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (BSP_LIS2DUXS12_0_SPI_Recv(pdata, len) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  /* CS Disable */
  HAL_GPIO_WritePin(BSP_LIS2DUXS12_0_CS_PORT, BSP_LIS2DUXS12_0_CS_PIN, GPIO_PIN_SET);

  return ret;
}
#endif

#if (USE_MOTION_SENSOR_H3LIS331DL_0  == 1)
/**
 * @brief  Register Bus IOs for H3LIS331DL instance
 * @param  Functions Motion sensor functions. Could be :
 *         - MOTION_ACCELERO
 * @retval BSP status
 */
static int32_t H3LIS331DL_0_Probe(uint32_t Functions)
{
  H3LIS331DL_IO_t            io_ctx;
  uint8_t                    id;
  static H3LIS331DL_Object_t h3lis331dl_obj_0;
  H3LIS331DL_Capabilities_t  cap;
  int32_t                    ret = BSP_ERROR_NONE;

  /* Configure the driver */
  io_ctx.BusType     = H3LIS331DL_SPI_4WIRES_BUS; /* SPI 4-Wires */
  io_ctx.Address     = 0x0;
  io_ctx.Init        = BSP_H3LIS331DL_0_Init;
  io_ctx.DeInit      = BSP_H3LIS331DL_0_DeInit;
  io_ctx.ReadReg     = BSP_H3LIS331DL_0_ReadReg;
  io_ctx.WriteReg    = BSP_H3LIS331DL_0_WriteReg;
  io_ctx.GetTick     = BSP_GetTick;

  if (H3LIS331DL_RegisterBusIO(&h3lis331dl_obj_0, &io_ctx) != H3LIS331DL_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (H3LIS331DL_ReadID(&h3lis331dl_obj_0, &id) != H3LIS331DL_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (id != H3LIS331DL_ID)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    (void)H3LIS331DL_GetCapabilities(&h3lis331dl_obj_0, &cap);
    MotionCtx[H3LIS331DL_0].Functions = ((uint32_t)cap.Gyro) | ((uint32_t)cap.Acc << 1) | ((uint32_t)cap.Magneto << 2);

    MotionCompObj[H3LIS331DL_0] = &h3lis331dl_obj_0;
    /* The second cast (void *) is added to bypass Misra R11.3 rule */
    MotionDrv[H3LIS331DL_0] = (MOTION_SENSOR_CommonDrv_t *)(void *)&H3LIS331DL_COMMON_Driver;

    if ((ret == BSP_ERROR_NONE) && ((Functions & MOTION_ACCELERO) == MOTION_ACCELERO) && (cap.Acc == 1U))
    {
      /* The second cast (void *) is added to bypass Misra R11.3 rule */
      MotionFuncDrv[H3LIS331DL_0][FunctionIndex[MOTION_ACCELERO]] = (MOTION_SENSOR_FuncDrv_t *)(void *)&H3LIS331DL_ACC_Driver;

      if (MotionDrv[H3LIS331DL_0]->Init(MotionCompObj[H3LIS331DL_0]) != H3LIS331DL_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    if ((ret == BSP_ERROR_NONE) && ((Functions & MOTION_GYRO) == MOTION_GYRO))
    {
      /* Return an error if the application try to initialize a function not supported by the component */
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    if ((ret == BSP_ERROR_NONE) && ((Functions & MOTION_MAGNETO) == MOTION_MAGNETO))
    {
      /* Return an error if the application try to initialize a function not supported by the component */
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return ret;
}

/**
 * @brief  Initialize SPI bus for H3LIS331DL
 * @retval BSP status
 */
static int32_t BSP_H3LIS331DL_0_Init(void)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if(BSP_H3LIS331DL_0_SPI_Init() == BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
 * @brief  DeInitialize SPI bus for H3LIS331DL
 * @retval BSP status
 */
static int32_t BSP_H3LIS331DL_0_DeInit(void)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if(BSP_H3LIS331DL_0_SPI_DeInit() == BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
 * @brief  Write register by SPI bus for H3LIS331DL
 * @param  Addr not used, it is only for BSP compatibility
 * @param  Reg the starting register address to be written
 * @param  pdata the pointer to the data to be written
 * @param  len the length of the data to be written
 * @retval BSP status
 */
static int32_t BSP_H3LIS331DL_0_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;

  /* CS Enable */
  HAL_GPIO_WritePin(BSP_H3LIS331DL_0_CS_PORT, BSP_H3LIS331DL_0_CS_PIN, GPIO_PIN_RESET);

  if (BSP_H3LIS331DL_0_SPI_Send(&dataReg, 1) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (BSP_H3LIS331DL_0_SPI_Send(pdata, len) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  /* CS Disable */
  HAL_GPIO_WritePin(BSP_H3LIS331DL_0_CS_PORT, BSP_H3LIS331DL_0_CS_PIN, GPIO_PIN_SET);

  return ret;
}

/**
 * @brief  Read register by SPI bus for H3LIS331DL
 * @param  Addr not used, it is only for BSP compatibility
 * @param  Reg the starting register address to be read
 * @param  pdata the pointer to the data to be read
 * @param  len the length of the data to be read
 * @retval BSP status
 */
static int32_t BSP_H3LIS331DL_0_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;

  dataReg |= 0x80;

  /* CS Enable */
  HAL_GPIO_WritePin(BSP_H3LIS331DL_0_CS_PORT, BSP_H3LIS331DL_0_CS_PIN, GPIO_PIN_RESET);

  if (BSP_H3LIS331DL_0_SPI_Send(&dataReg, 1) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (BSP_H3LIS331DL_0_SPI_Recv(pdata, len) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  /* CS Disable */
  HAL_GPIO_WritePin(BSP_H3LIS331DL_0_CS_PORT, BSP_H3LIS331DL_0_CS_PIN, GPIO_PIN_SET);

  return ret;
}
#endif

#if (USE_MOTION_SENSOR_LSM6DSO32X_0  == 1)
/**
 * @brief  Register Bus IOs for LSM6DSO32X instance
 * @param  Functions Motion sensor functions. Could be :
 *         - MOTION_GYRO and/or MOTION_ACCELERO
 * @retval BSP status
 */
static int32_t LSM6DSO32X_0_Probe(uint32_t Functions)
{
  LSM6DSO32X_IO_t            io_ctx;
  uint8_t                    id;
  static LSM6DSO32X_Object_t lsm6dso32x_obj_0;
  LSM6DSO32X_Capabilities_t  cap;
  int32_t                    ret = BSP_ERROR_NONE;

  /* Configure the driver */
  io_ctx.BusType     = LSM6DSO32X_SPI_4WIRES_BUS; /* SPI 4-Wires */
  io_ctx.Address     = 0x0;
  io_ctx.Init        = BSP_LSM6DSO32X_0_Init;
  io_ctx.DeInit      = BSP_LSM6DSO32X_0_DeInit;
  io_ctx.ReadReg     = BSP_LSM6DSO32X_0_ReadReg;
  io_ctx.WriteReg    = BSP_LSM6DSO32X_0_WriteReg;
  io_ctx.GetTick     = BSP_GetTick;

  if (LSM6DSO32X_RegisterBusIO(&lsm6dso32x_obj_0, &io_ctx) != LSM6DSO32X_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (LSM6DSO32X_ReadID(&lsm6dso32x_obj_0, &id) != LSM6DSO32X_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (id != LSM6DSO32X_ID)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    (void)LSM6DSO32X_GetCapabilities(&lsm6dso32x_obj_0, &cap);
    MotionCtx[LSM6DSO32X_0].Functions = ((uint32_t)cap.Gyro) | ((uint32_t)cap.Acc << 1) | ((uint32_t)cap.Magneto << 2);

    MotionCompObj[LSM6DSO32X_0] = &lsm6dso32x_obj_0;
    /* The second cast (void *) is added to bypass Misra R11.3 rule */
    MotionDrv[LSM6DSO32X_0] = (MOTION_SENSOR_CommonDrv_t *)(void *)&LSM6DSO32X_COMMON_Driver;

    if ((ret == BSP_ERROR_NONE) && ((Functions & MOTION_GYRO) == MOTION_GYRO) && (cap.Gyro == 1U))
    {
      /* The second cast (void *) is added to bypass Misra R11.3 rule */
      MotionFuncDrv[LSM6DSO32X_0][FunctionIndex[MOTION_GYRO]] = (MOTION_SENSOR_FuncDrv_t *)(void *)&LSM6DSO32X_GYRO_Driver;

      if (MotionDrv[LSM6DSO32X_0]->Init(MotionCompObj[LSM6DSO32X_0]) != LSM6DSO32X_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    if ((ret == BSP_ERROR_NONE) && ((Functions & MOTION_ACCELERO) == MOTION_ACCELERO) && (cap.Acc == 1U))
    {
      /* The second cast (void *) is added to bypass Misra R11.3 rule */
      MotionFuncDrv[LSM6DSO32X_0][FunctionIndex[MOTION_ACCELERO]] = (MOTION_SENSOR_FuncDrv_t *)(void *)&LSM6DSO32X_ACC_Driver;

      if (MotionDrv[LSM6DSO32X_0]->Init(MotionCompObj[LSM6DSO32X_0]) != LSM6DSO32X_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    if ((ret == BSP_ERROR_NONE) && ((Functions & MOTION_MAGNETO) == MOTION_MAGNETO))
    {
      /* Return an error if the application try to initialize a function not supported by the component */
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return ret;
}

/**
 * @brief  Initialize SPI bus for LSM6DSO32X
 * @retval BSP status
 */
static int32_t BSP_LSM6DSO32X_0_Init(void)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if(BSP_LSM6DSO32X_0_SPI_Init() == BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
 * @brief  DeInitialize SPI bus for LSM6DSO32X
 * @retval BSP status
 */
static int32_t BSP_LSM6DSO32X_0_DeInit(void)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if(BSP_LSM6DSO32X_0_SPI_DeInit() == BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
 * @brief  Write register by SPI bus for LSM6DSO32X
 * @param  Addr not used, it is only for BSP compatibility
 * @param  Reg the starting register address to be written
 * @param  pdata the pointer to the data to be written
 * @param  len the length of the data to be written
 * @retval BSP status
 */
static int32_t BSP_LSM6DSO32X_0_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;

  /* CS Enable */
  HAL_GPIO_WritePin(BSP_LSM6DSO32X_0_CS_PORT, BSP_LSM6DSO32X_0_CS_PIN, GPIO_PIN_RESET);

  if (BSP_LSM6DSO32X_0_SPI_Send(&dataReg, 1) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (BSP_LSM6DSO32X_0_SPI_Send(pdata, len) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  /* CS Disable */
  HAL_GPIO_WritePin(BSP_LSM6DSO32X_0_CS_PORT, BSP_LSM6DSO32X_0_CS_PIN, GPIO_PIN_SET);

  return ret;
}

/**
 * @brief  Read register by SPI bus for LSM6DSO32X
 * @param  Addr not used, it is only for BSP compatibility
 * @param  Reg the starting register address to be read
 * @param  pdata the pointer to the data to be read
 * @param  len the length of the data to be read
 * @retval BSP status
 */
static int32_t BSP_LSM6DSO32X_0_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;

  dataReg |= 0x80U;

  /* CS Enable */
  HAL_GPIO_WritePin(BSP_LSM6DSO32X_0_CS_PORT, BSP_LSM6DSO32X_0_CS_PIN, GPIO_PIN_RESET);

  if (BSP_LSM6DSO32X_0_SPI_Send(&dataReg, 1) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (BSP_LSM6DSO32X_0_SPI_Recv(pdata, len) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  /* CS Disable */
  HAL_GPIO_WritePin(BSP_LSM6DSO32X_0_CS_PORT, BSP_LSM6DSO32X_0_CS_PIN, GPIO_PIN_SET);

  return ret;
}
#endif

