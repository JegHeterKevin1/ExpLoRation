/**
 ******************************************************************************
 * @file    IIS2DLPC_ACC_driver_HL.c
 * @author  MEMS Application Team
 * @brief   This file provides a set of high-level functions needed to manage
            the IIS2DLPC sensor
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "IIS2DLPC_ACC_driver_HL.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup IIS2DLPC IIS2DLPC
 * @{
 */

/* Link function for sensor peripheral */
extern uint8_t Sensor_IO_Write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
extern uint8_t Sensor_IO_Read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);

/** @addtogroup IIS2DLPC_Private_Function_Prototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef IIS2DLPC_Set_ODR_When_Enabled(DrvContextTypeDef *handle, SensorOdr_t odr);
static DrvStatusTypeDef IIS2DLPC_Set_ODR_When_Disabled(DrvContextTypeDef *handle, SensorOdr_t odr);
static DrvStatusTypeDef IIS2DLPC_Set_ODR_Value_When_Enabled(DrvContextTypeDef *handle, float odr);
static DrvStatusTypeDef IIS2DLPC_Set_ODR_Value_When_Disabled(DrvContextTypeDef *handle, float odr);

/**
 * @}
 */

/** @addtogroup IIS2DLPC_Callable_Private_Function_Prototypes Callable private function prototypes
 * @{
 */

static DrvStatusTypeDef IIS2DLPC_Init(DrvContextTypeDef *handle);
static DrvStatusTypeDef IIS2DLPC_DeInit(DrvContextTypeDef *handle);
static DrvStatusTypeDef IIS2DLPC_Sensor_Enable(DrvContextTypeDef *handle);
static DrvStatusTypeDef IIS2DLPC_Sensor_Disable(DrvContextTypeDef *handle);
static DrvStatusTypeDef IIS2DLPC_Get_WhoAmI(DrvContextTypeDef *handle, uint8_t *who_am_i);
static DrvStatusTypeDef IIS2DLPC_Check_WhoAmI(DrvContextTypeDef *handle);
static DrvStatusTypeDef IIS2DLPC_Get_Axes(DrvContextTypeDef *handle, SensorAxes_t *acceleration);
static DrvStatusTypeDef IIS2DLPC_Get_AxesRaw(DrvContextTypeDef *handle, SensorAxesRaw_t *value);
static DrvStatusTypeDef IIS2DLPC_Get_Sensitivity(DrvContextTypeDef *handle, float *sensitivity);
static DrvStatusTypeDef IIS2DLPC_Get_ODR(DrvContextTypeDef *handle, float *odr);
static DrvStatusTypeDef IIS2DLPC_Set_ODR(DrvContextTypeDef *handle, SensorOdr_t odr);
static DrvStatusTypeDef IIS2DLPC_Set_ODR_Value(DrvContextTypeDef *handle, float odr);
static DrvStatusTypeDef IIS2DLPC_Get_FS(DrvContextTypeDef *handle, float *fullScale);
static DrvStatusTypeDef IIS2DLPC_Set_FS(DrvContextTypeDef *handle, SensorFs_t fs);
static DrvStatusTypeDef IIS2DLPC_Set_FS_Value(DrvContextTypeDef *handle, float fullScale);
static DrvStatusTypeDef IIS2DLPC_Read_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t *data);
static DrvStatusTypeDef IIS2DLPC_Write_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t data);
static DrvStatusTypeDef IIS2DLPC_Get_DRDY_Status(DrvContextTypeDef *handle, uint8_t *status);

/**
 * @}
 */

/** @addtogroup IIS2DLPC_Public_Variables Public variables
 * @{
 */

/**
 * @brief IIS2DLPC accelero driver structure
 */
ACCELERO_Drv_t IIS2DLPC_Drv =
{
  IIS2DLPC_Init,
  IIS2DLPC_DeInit,
  IIS2DLPC_Sensor_Enable,
  IIS2DLPC_Sensor_Disable,
  IIS2DLPC_Get_WhoAmI,
  IIS2DLPC_Check_WhoAmI,
  IIS2DLPC_Get_Axes,
  IIS2DLPC_Get_AxesRaw,
  IIS2DLPC_Get_Sensitivity,
  IIS2DLPC_Get_ODR,
  IIS2DLPC_Set_ODR,
  IIS2DLPC_Set_ODR_Value,
  IIS2DLPC_Get_FS,
  IIS2DLPC_Set_FS,
  IIS2DLPC_Set_FS_Value,
  0,
  0,
  IIS2DLPC_Read_Reg,
  IIS2DLPC_Write_Reg,
  IIS2DLPC_Get_DRDY_Status
};

/**
 * @}
 */

/** @addtogroup IIS2DLPC_Callable_Private_Functions Callable private functions
 * @{
 */

/**
 * @brief Initialize the IIS2DLPC sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Init(DrvContextTypeDef *handle)
{
  iis2dlpc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  if (IIS2DLPC_Check_WhoAmI(handle) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Enable BDU. */
  if (iis2dlpc_block_data_update_set(&ctx, PROPERTY_ENABLE) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* FIFO mode selection - FIFO disable. */
  if (iis2dlpc_fifo_mode_set(&ctx, IIS2DLPC_BYPASS_MODE) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Output data rate selection - power down. */
  if (iis2dlpc_data_rate_set(&ctx, IIS2DLPC_XL_ODR_OFF) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Set default power mode - High resolution. */
  if (iis2dlpc_power_mode_set(&ctx, IIS2DLPC_HIGH_PERFORMANCE) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Select default output data rate. */
  if (IIS2DLPC_Set_ODR_When_Disabled(handle, ODR_MID_HIGH) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Full scale selection. */
  if (IIS2DLPC_Set_FS(handle, FS_LOW) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  handle->isInitialized = 1;

  return COMPONENT_OK;
}

/**
 * @brief Deinitialize the IIS2DLPC sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_DeInit(DrvContextTypeDef *handle)
{
  ACCELERO_Data_t *pData = (ACCELERO_Data_t *)handle->pData;
  IIS2DLPC_Data_t *pComponentData = (IIS2DLPC_Data_t *)pData->pComponentData;

  if (IIS2DLPC_Check_WhoAmI(handle) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Disable the component */
  if (IIS2DLPC_Sensor_Disable(handle) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Reset previous output data rate. */
  pComponentData->Previous_ODR = 0.0f;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}

/**
 * @brief Enable the IIS2DLPC sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Sensor_Enable(DrvContextTypeDef *handle)
{
  ACCELERO_Data_t *pData = (ACCELERO_Data_t *)handle->pData;
  IIS2DLPC_Data_t *pComponentData = (IIS2DLPC_Data_t *)pData->pComponentData;

  /* Check if the component is already enabled */
  if (handle->isEnabled == 1)
  {
    return COMPONENT_OK;
  }

  /* Output data rate selection. */
  if (IIS2DLPC_Set_ODR_Value_When_Enabled(handle, pComponentData->Previous_ODR) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  handle->isEnabled = 1;

  return COMPONENT_OK;
}

/**
 * @brief Disable the IIS2DLPC sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Sensor_Disable(DrvContextTypeDef *handle)
{
  iis2dlpc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  ACCELERO_Data_t *pData = (ACCELERO_Data_t *)handle->pData;
  IIS2DLPC_Data_t *pComponentData = (IIS2DLPC_Data_t *)pData->pComponentData;

  /* Check if the component is already disabled */
  if (handle->isEnabled == 0)
  {
    return COMPONENT_OK;
  }

  /* Store actual output data rate. */
  if (IIS2DLPC_Get_ODR(handle, &(pComponentData->Previous_ODR)) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Output data rate selection - power down. */
  if (iis2dlpc_data_rate_set(&ctx, IIS2DLPC_XL_ODR_OFF) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  handle->isEnabled = 0;

  return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the IIS2DLPC sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Get_WhoAmI(DrvContextTypeDef *handle, uint8_t *who_am_i)
{
  iis2dlpc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  /* Read WHO AM I register */
  if (iis2dlpc_device_id_get(&ctx, (uint8_t *)who_am_i) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Check the WHO_AM_I ID of the IIS2DLPC sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Check_WhoAmI(DrvContextTypeDef *handle)
{
  uint8_t who_am_i = 0x00;

  if (IIS2DLPC_Get_WhoAmI(handle, &who_am_i) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  if (who_am_i != handle->who_am_i)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the IIS2DLPC accelerometer sensor axes
 * @param handle the device handle
 * @param acceleration pointer to where acceleration data write to
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Get_Axes(DrvContextTypeDef *handle, SensorAxes_t *acceleration)
{
  SensorAxesRaw_t dataRaw;
  float sensitivity = 0;

  /* Read raw data from IIS2DLPC output register. */
  if (IIS2DLPC_Get_AxesRaw(handle, &dataRaw) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Get IIS2DLPC actual sensitivity. */
  if (IIS2DLPC_Get_Sensitivity(handle, &sensitivity) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Calculate the data. */
  acceleration->AXIS_X = (int32_t)(dataRaw.AXIS_X * sensitivity);
  acceleration->AXIS_Y = (int32_t)(dataRaw.AXIS_Y * sensitivity);
  acceleration->AXIS_Z = (int32_t)(dataRaw.AXIS_Z * sensitivity);

  return COMPONENT_OK;
}

/**
 * @brief Get the IIS2DLPC accelerometer sensor raw axes
 * @param handle the device handle
 * @param acceleration_raw pointer to where acceleration raw data write to
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Get_AxesRaw(DrvContextTypeDef *handle, SensorAxesRaw_t *value)
{
  iis2dlpc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  uint8_t regValue[6] = { 0, 0, 0, 0, 0, 0 };
  int16_t dataRaw[3];
  iis2dlpc_mode_t mode;

  /* Read raw data from IIS2DLPC output register. */
  if (iis2dlpc_acceleration_raw_get(&ctx, regValue) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  dataRaw[0] = ((((int16_t)regValue[1]) << 8) + (int16_t)regValue[0]);
  dataRaw[1] = ((((int16_t)regValue[3]) << 8) + (int16_t)regValue[2]);
  dataRaw[2] = ((((int16_t)regValue[5]) << 8) + (int16_t)regValue[4]);

  /* Read actual power mode selection from sensor. */
  if (iis2dlpc_power_mode_get(&ctx, &mode) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Select raw data format according to power mode selected. */
  if (mode == IIS2DLPC_CONT_LOW_PWR_12bit || mode == IIS2DLPC_SINGLE_LOW_PWR_12bit
      || mode == IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_12bit || mode == IIS2DLPC_SINGLE_LOW_LOW_NOISE_PWR_12bit)
  {
    /* 12 bits left justified */
    dataRaw[0] >>= 4;
    dataRaw[1] >>= 4;
    dataRaw[2] >>= 4;
  }
  else
  {
    /* 14 bits left justified */
    dataRaw[0] >>= 2;
    dataRaw[1] >>= 2;
    dataRaw[2] >>= 2;
  }

  /* Set the raw data. */
  value->AXIS_X = dataRaw[0];
  value->AXIS_Y = dataRaw[1];
  value->AXIS_Z = dataRaw[2];

  return COMPONENT_OK;
}

/**
 * @brief Get the IIS2DLPC accelerometer sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer to where sensitivity write to
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Get_Sensitivity(DrvContextTypeDef *handle, float *sensitivity)
{
  iis2dlpc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  iis2dlpc_mode_t mode;
  iis2dlpc_fs_t fullScale;

  /* Read actual power mode selection from sensor. */
  if (iis2dlpc_power_mode_get(&ctx, &mode) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Read actual full scale selection from sensor. */
  if (iis2dlpc_full_scale_get(&ctx, &fullScale) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Store the sensitivity based on actual operating mode and full scale. */
  if (mode == IIS2DLPC_CONT_LOW_PWR_12bit || mode == IIS2DLPC_SINGLE_LOW_PWR_12bit
      || mode == IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_12bit || mode == IIS2DLPC_SINGLE_LOW_LOW_NOISE_PWR_12bit)
  {
    switch (fullScale)
    {
      case IIS2DLPC_2g:
        *sensitivity = (float)IIS2DLPC_ACC_SENSITIVITY_FOR_FS_2G_LOPOW1_MODE;
        break;
      case IIS2DLPC_4g:
        *sensitivity = (float)IIS2DLPC_ACC_SENSITIVITY_FOR_FS_4G_LOPOW1_MODE;
        break;
      case IIS2DLPC_8g:
        *sensitivity = (float)IIS2DLPC_ACC_SENSITIVITY_FOR_FS_8G_LOPOW1_MODE;
        break;
      case IIS2DLPC_16g:
        *sensitivity = (float)IIS2DLPC_ACC_SENSITIVITY_FOR_FS_16G_LOPOW1_MODE;
        break;
      default:
        *sensitivity = -1.0f;
        return COMPONENT_ERROR;
    }
  }
  else
  {
    switch (fullScale)
    {
      case IIS2DLPC_2g:
        *sensitivity = (float)IIS2DLPC_ACC_SENSITIVITY_FOR_FS_2G_OTHER_MODES;
        break;
      case IIS2DLPC_4g:
        *sensitivity = (float)IIS2DLPC_ACC_SENSITIVITY_FOR_FS_4G_OTHER_MODES;
        break;
      case IIS2DLPC_8g:
        *sensitivity = (float)IIS2DLPC_ACC_SENSITIVITY_FOR_FS_8G_OTHER_MODES;
        break;
      case IIS2DLPC_16g:
        *sensitivity = (float)IIS2DLPC_ACC_SENSITIVITY_FOR_FS_16G_OTHER_MODES;
        break;
      default:
        *sensitivity = -1.0f;
        return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the IIS2DLPC accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr pointer to where output data rate write to
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Get_ODR(DrvContextTypeDef *handle, float *odr)
{
  iis2dlpc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  iis2dlpc_mode_t mode;
  iis2dlpc_odr_t odr_low_level;

  /* Read actual power mode selection from sensor. */
  if (iis2dlpc_power_mode_get(&ctx, &mode) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Read actual output data rate selection from sensor. */
  if (iis2dlpc_data_rate_get(&ctx, &odr_low_level) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  switch (odr_low_level)
  {
    case IIS2DLPC_XL_ODR_OFF:
      *odr = 0.0f;
      break;
    case IIS2DLPC_XL_ODR_1Hz6_LP_ONLY:
      if (mode == IIS2DLPC_HIGH_PERFORMANCE || mode == IIS2DLPC_HIGH_PERFORMANCE_LOW_NOISE)
      {
        *odr = 12.5f;
      }
      else
      {
        *odr = 1.6f;
      }
      break;
    case IIS2DLPC_XL_ODR_12Hz5:
      *odr = 12.5f;
      break;
    case IIS2DLPC_XL_ODR_25Hz:
      *odr = 25.0f;
      break;
    case IIS2DLPC_XL_ODR_50Hz:
      *odr = 50.0f;
      break;
    case IIS2DLPC_XL_ODR_100Hz:
      *odr = 100.0f;
      break;
    case IIS2DLPC_XL_ODR_200Hz:
      *odr = 200.0f;
      break;
    case IIS2DLPC_XL_ODR_400Hz:
      if (mode == IIS2DLPC_HIGH_PERFORMANCE || mode == IIS2DLPC_HIGH_PERFORMANCE_LOW_NOISE)
      {
        *odr = 400.0f;
      }
      else
      {
        *odr = 200.0f;
      }
      break;
    case IIS2DLPC_XL_ODR_800Hz:
      if (mode == IIS2DLPC_HIGH_PERFORMANCE || mode == IIS2DLPC_HIGH_PERFORMANCE_LOW_NOISE)
      {
        *odr = 800.0f;
      }
      else
      {
        *odr = 200.0f;
      }
      break;
    case IIS2DLPC_XL_ODR_1k6Hz:
      if (mode == IIS2DLPC_HIGH_PERFORMANCE || mode == IIS2DLPC_HIGH_PERFORMANCE_LOW_NOISE)
      {
        *odr = 1600.0f;
      }
      else
      {
        *odr = 200.0f;
      }
      break;
    default:
      *odr = -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the IIS2DLPC accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Set_ODR(DrvContextTypeDef *handle, SensorOdr_t odr)
{
  if (handle->isEnabled == 1)
  {
    if (IIS2DLPC_Set_ODR_When_Enabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if (IIS2DLPC_Set_ODR_When_Disabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the IIS2DLPC accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Set_ODR_Value(DrvContextTypeDef *handle, float odr)
{
  if (handle->isEnabled == 1)
  {
    if (IIS2DLPC_Set_ODR_Value_When_Enabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if (IIS2DLPC_Set_ODR_Value_When_Disabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the IIS2DLPC accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale pointer to where full scale write to
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Get_FS(DrvContextTypeDef *handle, float *fullScale)
{
  iis2dlpc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  iis2dlpc_fs_t fs_low_level;

  if (iis2dlpc_full_scale_get(&ctx, &fs_low_level) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  switch (fs_low_level)
  {
    case IIS2DLPC_2g:
      *fullScale = 2.0f;
      break;
    case IIS2DLPC_4g:
      *fullScale = 4.0f;
      break;
    case IIS2DLPC_8g:
      *fullScale = 8.0f;
      break;
    case IIS2DLPC_16g:
      *fullScale = 16.0f;
      break;
    default:
      *fullScale = -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the IIS2DLPC accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale the functional full scale to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Set_FS(DrvContextTypeDef *handle, SensorFs_t fullScale)
{
  iis2dlpc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  iis2dlpc_fs_t new_fs;

  switch (fullScale)
  {

    case FS_LOW:
      new_fs = IIS2DLPC_2g;
      break;

    case FS_MID_LOW:
    case FS_MID:
      new_fs = IIS2DLPC_4g;
      break;

    case FS_MID_HIGH:
      new_fs = IIS2DLPC_8g;
      break;

    case FS_HIGH:
      new_fs = IIS2DLPC_16g;
      break;

    default:
      return COMPONENT_ERROR;
  }

  if (iis2dlpc_full_scale_set(&ctx, new_fs) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the IIS2DLPC accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale the full scale value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Set_FS_Value(DrvContextTypeDef *handle, float fullScale)
{
  iis2dlpc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  iis2dlpc_fs_t new_fs;

  new_fs = (fullScale <= 2.0f) ?  IIS2DLPC_2g
           : (fullScale <= 4.0f) ?  IIS2DLPC_4g
           : (fullScale <= 8.0f) ?  IIS2DLPC_8g
           :                          IIS2DLPC_16g;

  if (iis2dlpc_full_scale_set(&ctx, new_fs) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Read_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t *data)
{
  iis2dlpc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  return (DrvStatusTypeDef)iis2dlpc_read_reg(&ctx, reg, data, 1);
}

/**
 * @brief Write the data to register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Write_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t data)
{
  iis2dlpc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  return (DrvStatusTypeDef)iis2dlpc_write_reg(&ctx, reg, &data, 1);
}

/**
 * @brief Get data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Get_DRDY_Status(DrvContextTypeDef *handle, uint8_t *status)
{
  iis2dlpc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  if (iis2dlpc_flag_data_ready_get(&ctx, status) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @}
 */

/** @addtogroup IIS2DLPC_Private_Functions Private functions
 * @{
 */

/**
 * @brief Set the IIS2DLPC accelerometer sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Set_ODR_When_Enabled(DrvContextTypeDef *handle, SensorOdr_t odr)
{
  iis2dlpc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  iis2dlpc_odr_t new_odr;

  switch (odr)
  {

    case ODR_LOW:
      new_odr = IIS2DLPC_XL_ODR_12Hz5;
      break;

    case ODR_MID_LOW:
      new_odr = IIS2DLPC_XL_ODR_25Hz;
      break;

    case ODR_MID:
      new_odr = IIS2DLPC_XL_ODR_50Hz;
      break;

    case ODR_MID_HIGH:
      new_odr = IIS2DLPC_XL_ODR_100Hz;
      break;

    case ODR_HIGH:
      new_odr = IIS2DLPC_XL_ODR_200Hz;
      break;

    default:
      return COMPONENT_ERROR;
  }

  if (iis2dlpc_data_rate_set(&ctx, new_odr) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the IIS2DLPC accelerometer sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Set_ODR_When_Disabled(DrvContextTypeDef *handle, SensorOdr_t odr)
{
  ACCELERO_Data_t *pData = (ACCELERO_Data_t *)handle->pData;
  IIS2DLPC_Data_t *pComponentData = (IIS2DLPC_Data_t *)pData->pComponentData;

  switch (odr)
  {

    case ODR_LOW:
      pComponentData->Previous_ODR =  12.5f;
      break;

    case ODR_MID_LOW:
      pComponentData->Previous_ODR =  25.0f;
      break;

    case ODR_MID:
      pComponentData->Previous_ODR =  50.0f;
      break;

    case ODR_MID_HIGH:
      pComponentData->Previous_ODR = 100.0f;
      break;

    case ODR_HIGH:
      pComponentData->Previous_ODR = 200.0f;
      break;

    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the IIS2DLPC accelerometer sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Set_ODR_Value_When_Enabled(DrvContextTypeDef *handle, float odr)
{
  iis2dlpc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  iis2dlpc_odr_t new_odr;

  new_odr = (odr <=   1.6f) ? IIS2DLPC_XL_ODR_1Hz6_LP_ONLY
            : (odr <=  12.5f) ? IIS2DLPC_XL_ODR_12Hz5
            : (odr <=  25.0f) ? IIS2DLPC_XL_ODR_25Hz
            : (odr <=  50.0f) ? IIS2DLPC_XL_ODR_50Hz
            : (odr <= 100.0f) ? IIS2DLPC_XL_ODR_100Hz
            : (odr <= 200.0f) ? IIS2DLPC_XL_ODR_200Hz
            : (odr <= 400.0f) ? IIS2DLPC_XL_ODR_400Hz
            : (odr <= 800.0f) ? IIS2DLPC_XL_ODR_800Hz
            :                     IIS2DLPC_XL_ODR_1k6Hz;

  if (iis2dlpc_data_rate_set(&ctx, new_odr) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  if (odr <= 1.6f)
  {
    /* Set low-power mode for 1.6 Hz ODR */
    if (iis2dlpc_power_mode_set(&ctx, IIS2DLPC_CONT_LOW_PWR_2) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  if (odr > 200.0f)
  {
    /* Set high-performance mode for ODR higher then 200 Hz */
    if (iis2dlpc_power_mode_set(&ctx, IIS2DLPC_HIGH_PERFORMANCE) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the IIS2DLPC accelerometer sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DLPC_Set_ODR_Value_When_Disabled(DrvContextTypeDef *handle, float odr)
{
  ACCELERO_Data_t *pData = (ACCELERO_Data_t *)handle->pData;
  IIS2DLPC_Data_t *pComponentData = (IIS2DLPC_Data_t *)pData->pComponentData;

  pComponentData->Previous_ODR = (odr <=   1.6f) ?    1.6f
                                 : (odr <=  12.5f) ?   12.5f
                                 : (odr <=  25.0f) ?   25.0f
                                 : (odr <=  50.0f) ?   50.0f
                                 : (odr <= 100.0f) ?  100.0f
                                 : (odr <= 200.0f) ?  200.0f
                                 : (odr <= 400.0f) ?  400.0f
                                 : (odr <= 800.0f) ?  800.0f
                                 :                     1600.0f;

  return COMPONENT_OK;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
