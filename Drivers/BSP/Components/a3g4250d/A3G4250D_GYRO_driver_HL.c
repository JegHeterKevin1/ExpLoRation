/**
 ******************************************************************************
 * @file    A3G4250D_GYRO_driver_HL.c
 * @author  MEMS Application Team
 * @brief   This file provides a set of high-level functions needed to manage
            the A3G4250D sensor
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

#include "A3G4250D_GYRO_driver_HL.h"


/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup A3G4250D A3G4250D
 * @{
 */

/* Link function for sensor peripheral */
extern uint8_t Sensor_IO_Write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
extern uint8_t Sensor_IO_Read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);

/** @addtogroup A3G4250D_Callable_Private_Function_Prototypes Callable private function prototypes
 * @{
 */

static DrvStatusTypeDef A3G4250D_Init(DrvContextTypeDef *handle);
static DrvStatusTypeDef A3G4250D_DeInit(DrvContextTypeDef *handle);
static DrvStatusTypeDef A3G4250D_Sensor_Enable(DrvContextTypeDef *handle);
static DrvStatusTypeDef A3G4250D_Sensor_Disable(DrvContextTypeDef *handle);
static DrvStatusTypeDef A3G4250D_Get_WhoAmI(DrvContextTypeDef *handle, uint8_t *who_am_i);
static DrvStatusTypeDef A3G4250D_Check_WhoAmI(DrvContextTypeDef *handle);
static DrvStatusTypeDef A3G4250D_Get_Axes(DrvContextTypeDef *handle, SensorAxes_t *angular_velocity);
static DrvStatusTypeDef A3G4250D_Get_AxesRaw(DrvContextTypeDef *handle, SensorAxesRaw_t *value);
static DrvStatusTypeDef A3G4250D_Get_Sensitivity(DrvContextTypeDef *handle, float *sensitivity);
static DrvStatusTypeDef A3G4250D_Get_ODR(DrvContextTypeDef *handle, float *odr);
static DrvStatusTypeDef A3G4250D_Set_ODR(DrvContextTypeDef *handle, SensorOdr_t odr);
static DrvStatusTypeDef A3G4250D_Set_ODR_Value(DrvContextTypeDef *handle, float odr);
static DrvStatusTypeDef A3G4250D_Get_FS(DrvContextTypeDef *handle, float *fullScale);
static DrvStatusTypeDef A3G4250D_Set_FS(DrvContextTypeDef *handle, SensorFs_t fullScale);
static DrvStatusTypeDef A3G4250D_Set_FS_Value(DrvContextTypeDef *handle, float fullScale);
static DrvStatusTypeDef A3G4250D_Read_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t *data);
static DrvStatusTypeDef A3G4250D_Write_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t data);
static DrvStatusTypeDef A3G4250D_Get_DRDY_Status(DrvContextTypeDef *handle, uint8_t *status);

/**
 * @}
 */

/** @addtogroup A3G4250D_Private_Function_Prototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef A3G4250D_Get_Axes_Raw(DrvContextTypeDef *handle, int16_t *pData);
static DrvStatusTypeDef A3G4250D_Set_ODR_When_Enabled(DrvContextTypeDef *handle, SensorOdr_t odr);
static DrvStatusTypeDef A3G4250D_Set_ODR_When_Disabled(DrvContextTypeDef *handle, SensorOdr_t odr);
static DrvStatusTypeDef A3G4250D_Set_ODR_Value_When_Enabled(DrvContextTypeDef *handle, float odr);
static DrvStatusTypeDef A3G4250D_Set_ODR_Value_When_Disabled(DrvContextTypeDef *handle, float odr);

/**
 * @}
 */

/** @addtogroup A3G4250D_Public_Variables Public variables
 * @{
 */

/**
 * @brief A3G4250D gyro driver structure
 */
GYRO_Drv_t A3G4250D_Drv =
{
  A3G4250D_Init,
  A3G4250D_DeInit,
  A3G4250D_Sensor_Enable,
  A3G4250D_Sensor_Disable,
  A3G4250D_Get_WhoAmI,
  A3G4250D_Check_WhoAmI,
  A3G4250D_Get_Axes,
  A3G4250D_Get_AxesRaw,
  A3G4250D_Get_Sensitivity,
  A3G4250D_Get_ODR,
  A3G4250D_Set_ODR,
  A3G4250D_Set_ODR_Value,
  A3G4250D_Get_FS,
  A3G4250D_Set_FS,
  A3G4250D_Set_FS_Value,
  0,
  0,
  A3G4250D_Read_Reg,
  A3G4250D_Write_Reg,
  A3G4250D_Get_DRDY_Status
};

/** @addtogroup A3G4250D_Callable_Private_Functions Callable private functions
 * @{
 */

/**
 * @brief Initialize the A3G4250D gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Init(DrvContextTypeDef *handle)
{
  a3g4250d_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  if (A3G4250D_Check_WhoAmI(handle) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Output data rate selection - power down */
  if (a3g4250d_data_rate_set(&ctx, A3G4250D_ODR_OFF) == 1)
  {
    return COMPONENT_ERROR;
  }

  /* Select default output data rate. */
  if (A3G4250D_Set_ODR_When_Disabled(handle, ODR_MID_HIGH) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Full scale selection. */
  if (A3G4250D_Set_FS(handle, FS_MID) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  handle->isInitialized = 1;

  return COMPONENT_OK;
}

/**
 * @brief Deinitialize the A3G4250D gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_DeInit(DrvContextTypeDef *handle)
{
  GYRO_Data_t *pData = (GYRO_Data_t *)handle->pData;
  A3G4250D_Data_t *pComponentData = (A3G4250D_Data_t *)pData->pComponentData;

  if (A3G4250D_Check_WhoAmI(handle) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Disable the component */
  if (A3G4250D_Sensor_Disable(handle) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Reset output data rate. */
  pComponentData->Previous_ODR = 0.0f;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}

/**
 * @brief Enable the A3G4250D gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Sensor_Enable(DrvContextTypeDef *handle)
{
  GYRO_Data_t *pData = (GYRO_Data_t *)handle->pData;
  A3G4250D_Data_t *pComponentData = (A3G4250D_Data_t *)pData->pComponentData;

  /* Check if the component is already enabled */
  if (handle->isEnabled == 1)
  {
    return COMPONENT_OK;
  }

  /* Output data rate selection. */
  if (A3G4250D_Set_ODR_Value_When_Enabled(handle, pComponentData->Previous_ODR) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  handle->isEnabled = 1;

  return COMPONENT_OK;
}

/**
 * @brief Disable the A3G4250D gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Sensor_Disable(DrvContextTypeDef *handle)
{
  a3g4250d_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  GYRO_Data_t *pData = (GYRO_Data_t *)handle->pData;
  A3G4250D_Data_t *pComponentData = (A3G4250D_Data_t *)pData->pComponentData;

  /* Check if the component is already disabled */
  if (handle->isEnabled == 0)
  {
    return COMPONENT_OK;
  }

  /* Store actual output data rate. */
  if (A3G4250D_Get_ODR(handle, &(pComponentData->Previous_ODR)) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Output data rate selection - power down */
  if (a3g4250d_data_rate_set(&ctx, A3G4250D_ODR_OFF) == 1)
  {
    return COMPONENT_ERROR;
  }

  handle->isEnabled = 0;

  return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the A3G4250D gyroscope sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Get_WhoAmI(DrvContextTypeDef *handle, uint8_t *who_am_i)
{
  a3g4250d_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  /* Read WHO AM I register */
  if (a3g4250d_device_id_get(&ctx, (uint8_t *)who_am_i) == 1)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Check the WHO_AM_I ID of the A3G4250D gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Check_WhoAmI(DrvContextTypeDef *handle)
{
  uint8_t who_am_i = 0x00;

  if (A3G4250D_Get_WhoAmI(handle, &who_am_i) == COMPONENT_ERROR)
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
 * @brief Get the A3G4250D gyroscope sensor axes
 * @param handle the device handle
 * @param angular_velocity pointer where the values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Get_Axes(DrvContextTypeDef *handle, SensorAxes_t *angular_velocity)
{
  int16_t dataRaw[3];
  float   sensitivity = 0;

  /* Read raw data from A3G4250D output register. */
  if (A3G4250D_Get_Axes_Raw(handle, dataRaw) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Get A3G4250D actual sensitivity. */
  if (A3G4250D_Get_Sensitivity(handle, &sensitivity) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Calculate the data. */
  angular_velocity->AXIS_X = (int32_t)(dataRaw[0] * sensitivity);
  angular_velocity->AXIS_Y = (int32_t)(dataRaw[1] * sensitivity);
  angular_velocity->AXIS_Z = (int32_t)(dataRaw[2] * sensitivity);

  return COMPONENT_OK;
}

/**
 * @brief Get the A3G4250D gyroscope sensor raw axes
 * @param handle the device handle
 * @param value pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Get_AxesRaw(DrvContextTypeDef *handle, SensorAxesRaw_t *value)
{
  int16_t dataRaw[3];

  /* Read raw data from A3G4250D output register. */
  if (A3G4250D_Get_Axes_Raw(handle, dataRaw) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Set the raw data. */
  value->AXIS_X = dataRaw[0];
  value->AXIS_Y = dataRaw[1];
  value->AXIS_Z = dataRaw[2];

  return COMPONENT_OK;
}

/**
 * @brief Get the A3G4250D gyroscope sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Get_Sensitivity(DrvContextTypeDef *handle, float *sensitivity)
{
  *sensitivity = (float)A3G4250D_GYRO_SENSITIVITY_FOR_FS_245DPS;

  return COMPONENT_OK;
}

/**
 * @brief Get the A3G4250D gyroscope sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Get_ODR(DrvContextTypeDef *handle, float *odr)
{
  a3g4250d_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  a3g4250d_dr_t odr_raw;

  if (a3g4250d_data_rate_get(&ctx, &odr_raw) == 1)
  {
    return COMPONENT_ERROR;
  }

  switch (odr_raw)
  {
    case A3G4250D_ODR_OFF:
      *odr =    0.0f;
      break;
    case A3G4250D_ODR_100Hz:
      *odr =  100.0f;
      break;
    case A3G4250D_ODR_200Hz:
      *odr =  200.0f;
      break;
    case A3G4250D_ODR_400Hz:
      *odr =  400.0f;
      break;
    case A3G4250D_ODR_800Hz:
      *odr =  800.0f;
      break;
    default:
      *odr =   -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the A3G4250D gyroscope sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Set_ODR(DrvContextTypeDef *handle, SensorOdr_t odr)
{
  if (handle->isEnabled == 1)
  {
    if (A3G4250D_Set_ODR_When_Enabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if (A3G4250D_Set_ODR_When_Disabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the A3G4250D gyroscope sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Set_ODR_Value(DrvContextTypeDef *handle, float odr)
{
  if (handle->isEnabled == 1)
  {
    if (A3G4250D_Set_ODR_Value_When_Enabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if (A3G4250D_Set_ODR_Value_When_Disabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the A3G4250D gyroscope sensor full scale
 * @param handle the device handle
 * @param fullScale pointer where the full scale is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Get_FS(DrvContextTypeDef *handle, float *fullScale)
{
  *fullScale =  245.0f;

  return COMPONENT_OK;
}

/**
 * @brief Set the A3G4250D gyroscope sensor full scale
 * @param handle the device handle
 * @param fullScale the functional full scale to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Set_FS(DrvContextTypeDef *handle, SensorFs_t fullScale)
{
  return COMPONENT_OK;
}

/**
 * @brief Set the A3G4250D gyroscope sensor full scale
 * @param handle the device handle
 * @param fullScale the full scale value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Set_FS_Value(DrvContextTypeDef *handle, float fullScale)
{
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
static DrvStatusTypeDef A3G4250D_Read_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t *data)
{
  a3g4250d_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  if (a3g4250d_read_reg(&ctx, reg, data, 1) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Write the data to register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Write_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t data)
{
  a3g4250d_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  if (a3g4250d_write_reg(&ctx, reg, &data, 1) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get gyroscope data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Get_DRDY_Status(DrvContextTypeDef *handle, uint8_t *status)
{
  a3g4250d_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  if (a3g4250d_flag_data_ready_get(&ctx, status) == 1)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @}
 */

/** @addtogroup A3G4250D_Private_Functions Private functions
 * @{
 */


/**
 * @brief Get the A3G4250D gyroscope sensor raw axes
 * @param handle the device handle
 * @param pData pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Get_Axes_Raw(DrvContextTypeDef *handle, int16_t *pData)
{
  a3g4250d_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};

  /* Read output registers from A3G4250D_OUTX_L to A3G4250D_OUTZ_H. */
  if (a3g4250d_angular_rate_raw_get(&ctx, (uint8_t *)regValue) == 1)
  {
    return COMPONENT_ERROR;
  }

  /* Format the data. */
  pData[0] = ((((int16_t)regValue[1]) << 8) + (int16_t)regValue[0]);
  pData[1] = ((((int16_t)regValue[3]) << 8) + (int16_t)regValue[2]);
  pData[2] = ((((int16_t)regValue[5]) << 8) + (int16_t)regValue[4]);

  return COMPONENT_OK;
}

/**
 * @brief Set the A3G4250D gyroscope sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Set_ODR_When_Enabled(DrvContextTypeDef *handle, SensorOdr_t odr)
{
  a3g4250d_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  a3g4250d_dr_t new_odr;

  switch (odr)
  {
    case ODR_LOW:
    case ODR_MID_LOW:
      new_odr = A3G4250D_ODR_100Hz;
      break;
    case ODR_MID:
      new_odr = A3G4250D_ODR_200Hz;
      break;
    case ODR_MID_HIGH:
      new_odr = A3G4250D_ODR_400Hz;
      break;
    case ODR_HIGH:
      new_odr = A3G4250D_ODR_800Hz;
      break;
    default:
      return COMPONENT_ERROR;
  }

  if (a3g4250d_data_rate_set(&ctx, new_odr) == 1)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the A3G4250D gyroscope sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Set_ODR_When_Disabled(DrvContextTypeDef *handle, SensorOdr_t odr)
{
  GYRO_Data_t *pData = (GYRO_Data_t *)handle->pData;
  A3G4250D_Data_t *pComponentData = (A3G4250D_Data_t *)pData->pComponentData;

  switch (odr)
  {
    case ODR_LOW:
    case ODR_MID_LOW:
      pComponentData->Previous_ODR = 100.0f;
      break;
    case ODR_MID:
      pComponentData->Previous_ODR = 200.0f;
      break;
    case ODR_MID_HIGH:
      pComponentData->Previous_ODR = 400.0f;
      break;
    case ODR_HIGH:
      pComponentData->Previous_ODR = 800.0f;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the A3G4250D gyroscope sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Set_ODR_Value_When_Enabled(DrvContextTypeDef *handle, float odr)
{
  a3g4250d_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  a3g4250d_dr_t new_odr;

  new_odr = (odr <= 100.0f)  ? A3G4250D_ODR_100Hz
            : (odr <= 200.0f)  ? A3G4250D_ODR_200Hz
            : (odr <= 400.0f)  ? A3G4250D_ODR_400Hz
            :                      A3G4250D_ODR_800Hz;

  if (a3g4250d_data_rate_set(&ctx, new_odr) == 1)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the A3G4250D gyroscope sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef A3G4250D_Set_ODR_Value_When_Disabled(DrvContextTypeDef *handle, float odr)
{

  GYRO_Data_t *pData = (GYRO_Data_t *)handle->pData;
  A3G4250D_Data_t *pComponentData = (A3G4250D_Data_t *)pData->pComponentData;

  pComponentData->Previous_ODR = (odr <= 100.0f)  ? 100.0f
                                 : (odr <= 200.0f)  ? 200.0f
                                 : (odr <= 400.0f)  ? 400.0f
                                 :                      800.0f;

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
