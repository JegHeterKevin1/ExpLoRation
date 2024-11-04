/**
 *******************************************************************************
 * @file    IIS2MDC_MAG_driver_HL.c
 * @author  MEMS Application Team
 * @brief   This file provides a set of high-level functions needed to manage
            the IIS2MDC sensor
 *******************************************************************************
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
#include "IIS2MDC_MAG_driver_HL.h"
#include <math.h>



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup IIS2MDC_MAG IIS2MDC_MAG
 * @{
 */

/* Link function for sensor peripheral */
extern uint8_t Sensor_IO_Write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
extern uint8_t Sensor_IO_Read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);

/** @addtogroup IIS2MDC_MAG_Private_FunctionPrototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef IIS2MDC_Get_Axes_Raw(DrvContextTypeDef *handle, int16_t *pData);

/**
 * @}
 */

/** @addtogroup IIS2MDC_MAG_Callable_Private_FunctionPrototypes Callable private function prototypes
 * @{
 */

static DrvStatusTypeDef IIS2MDC_Init(DrvContextTypeDef *handle);
static DrvStatusTypeDef IIS2MDC_DeInit(DrvContextTypeDef *handle);
static DrvStatusTypeDef IIS2MDC_Sensor_Enable(DrvContextTypeDef *handle);
static DrvStatusTypeDef IIS2MDC_Sensor_Disable(DrvContextTypeDef *handle);
static DrvStatusTypeDef IIS2MDC_Get_WhoAmI(DrvContextTypeDef *handle, uint8_t *who_am_i);
static DrvStatusTypeDef IIS2MDC_Check_WhoAmI(DrvContextTypeDef *handle);
static DrvStatusTypeDef IIS2MDC_Get_Axes(DrvContextTypeDef *handle, SensorAxes_t *magnetic_field);
static DrvStatusTypeDef IIS2MDC_Get_AxesRaw(DrvContextTypeDef *handle, SensorAxesRaw_t *value);
static DrvStatusTypeDef IIS2MDC_Get_Sensitivity(DrvContextTypeDef *handle, float *sensitivity);
static DrvStatusTypeDef IIS2MDC_Get_ODR(DrvContextTypeDef *handle, float *odr);
static DrvStatusTypeDef IIS2MDC_Set_ODR(DrvContextTypeDef *handle, SensorOdr_t odr);
static DrvStatusTypeDef IIS2MDC_Set_ODR_Value(DrvContextTypeDef *handle, float odr);
static DrvStatusTypeDef IIS2MDC_Get_FS(DrvContextTypeDef *handle, float *fullScale);
static DrvStatusTypeDef IIS2MDC_Set_FS(DrvContextTypeDef *handle, SensorFs_t fullScale);
static DrvStatusTypeDef IIS2MDC_Set_FS_Value(DrvContextTypeDef *handle, float fullScale);
static DrvStatusTypeDef IIS2MDC_Read_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t *data);
static DrvStatusTypeDef IIS2MDC_Write_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t data);
static DrvStatusTypeDef IIS2MDC_Get_DRDY_Status(DrvContextTypeDef *handle, uint8_t *status);

/**
 * @}
 */

/** @addtogroup IIS2MDC_MAG_Private_Variables Private variables
 * @{
 */

/**
 * @brief IIS2MDC_MAG driver structure
 */
MAGNETO_Drv_t IIS2MDCDrv =
{
  IIS2MDC_Init,
  IIS2MDC_DeInit,
  IIS2MDC_Sensor_Enable,
  IIS2MDC_Sensor_Disable,
  IIS2MDC_Get_WhoAmI,
  IIS2MDC_Check_WhoAmI,
  IIS2MDC_Get_Axes,
  IIS2MDC_Get_AxesRaw,
  IIS2MDC_Get_Sensitivity,
  IIS2MDC_Get_ODR,
  IIS2MDC_Set_ODR,
  IIS2MDC_Set_ODR_Value,
  IIS2MDC_Get_FS,
  IIS2MDC_Set_FS,
  IIS2MDC_Set_FS_Value,
  IIS2MDC_Read_Reg,
  IIS2MDC_Write_Reg,
  IIS2MDC_Get_DRDY_Status
};

/** @addtogroup IIS2MDC_MAG_Callable_Private_Functions Callable private functions
 * @{
 */

/**
 * @brief Initialize the IIS2MDC sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2MDC_Init(DrvContextTypeDef *handle)
{
  iis2mdc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  if (IIS2MDC_Check_WhoAmI(handle) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Operating mode selection - power down */
  if (iis2mdc_operating_mode_set(&ctx, IIS2MDC_POWER_DOWN) == 1)
  {
    return COMPONENT_ERROR;
  }

  /* Enable BDU */
  if (iis2mdc_block_data_update_set(&ctx, PROPERTY_ENABLE) == 1)
  {
    return COMPONENT_ERROR;
  }

  if (IIS2MDC_Set_ODR(handle, ODR_HIGH) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  if (IIS2MDC_Set_FS(handle, FS_LOW) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  if (iis2mdc_self_test_set(&ctx, PROPERTY_DISABLE) == 1)
  {
    return COMPONENT_ERROR;
  }

  handle->isInitialized = 1;

  return COMPONENT_OK;
}


/**
 * @brief Deinitialize the IIS2MDC sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2MDC_DeInit(DrvContextTypeDef *handle)
{
  if (IIS2MDC_Check_WhoAmI(handle) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Disable the component */
  if (IIS2MDC_Sensor_Disable(handle) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  handle->isInitialized = 0;

  return COMPONENT_OK;
}



/**
 * @brief Enable the IIS2MDC sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2MDC_Sensor_Enable(DrvContextTypeDef *handle)
{
  iis2mdc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  /* Check if the component is already enabled */
  if (handle->isEnabled == 1)
  {
    return COMPONENT_OK;
  }

  /* Operating mode selection */
  if (iis2mdc_operating_mode_set(&ctx, IIS2MDC_CONTINUOUS_MODE) == 1)
  {
    return COMPONENT_ERROR;
  }

  handle->isEnabled = 1;

  return COMPONENT_OK;
}



/**
 * @brief Disable the IIS2MDC sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2MDC_Sensor_Disable(DrvContextTypeDef *handle)
{
  iis2mdc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  /* Check if the component is already disabled */
  if (handle->isEnabled == 0)
  {
    return COMPONENT_OK;
  }

  /* Operating mode selection - power down */
  if (iis2mdc_operating_mode_set(&ctx, IIS2MDC_POWER_DOWN) == 1)
  {
    return COMPONENT_ERROR;
  }

  handle->isEnabled = 0;

  return COMPONENT_OK;
}



/**
 * @brief Get the WHO_AM_I ID of the IIS2MDC sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2MDC_Get_WhoAmI(DrvContextTypeDef *handle, uint8_t *who_am_i)
{
  iis2mdc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  /* Read WHO AM I register */
  if (iis2mdc_device_id_get(&ctx, (uint8_t *)who_am_i) == 1)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Check the WHO_AM_I ID of the IIS2MDC sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2MDC_Check_WhoAmI(DrvContextTypeDef *handle)
{

  uint8_t who_am_i = 0x00;

  if (IIS2MDC_Get_WhoAmI(handle, &who_am_i) == COMPONENT_ERROR)
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
 * @brief Get the IIS2MDC sensor axes
 * @param handle the device handle
 * @param magnetic_field pointer where the values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2MDC_Get_Axes(DrvContextTypeDef *handle, SensorAxes_t *magnetic_field)
{

  int16_t pDataRaw[3];
  float sensitivity = 0;

  /* Read raw data from IIS2MDC output register. */
  if (IIS2MDC_Get_Axes_Raw(handle, pDataRaw) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Get IIS2MDC actual sensitivity. */
  if (IIS2MDC_Get_Sensitivity(handle, &sensitivity) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Calculate the data. */
  magnetic_field->AXIS_X = (int32_t)(pDataRaw[0] * sensitivity);
  magnetic_field->AXIS_Y = (int32_t)(pDataRaw[1] * sensitivity);
  magnetic_field->AXIS_Z = (int32_t)(pDataRaw[2] * sensitivity);

  return COMPONENT_OK;
}



/**
 * @brief Get the IIS2MDC sensor raw axes
 * @param handle the device handle
 * @param value pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2MDC_Get_AxesRaw(DrvContextTypeDef *handle, SensorAxesRaw_t *value)
{

  int16_t pDataRaw[3];

  /* Read raw data from IIS2MDC output register. */
  if (IIS2MDC_Get_Axes_Raw(handle, pDataRaw) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Set the raw data. */
  value->AXIS_X = pDataRaw[0];
  value->AXIS_Y = pDataRaw[1];
  value->AXIS_Z = pDataRaw[2];

  return COMPONENT_OK;
}



/**
 * @brief Get the IIS2MDC sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written [LSB/gauss]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2MDC_Get_Sensitivity(DrvContextTypeDef *handle, float *sensitivity)
{
  *sensitivity = 1.5f;

  return COMPONENT_OK;
}



/**
 * @brief Get the IIS2MDC sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2MDC_Get_ODR(DrvContextTypeDef *handle, float *odr)
{
  iis2mdc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  iis2mdc_odr_t odr_low_level;

  if (iis2mdc_data_rate_get(&ctx, &odr_low_level) == 1)
  {
    return COMPONENT_ERROR;
  }

  switch (odr_low_level)
  {
    case IIS2MDC_ODR_10Hz:
      *odr = 10.000f;
      break;

    case IIS2MDC_ODR_20Hz:
      *odr = 20.000f;
      break;

    case IIS2MDC_ODR_50Hz:
      *odr = 50.000f;
      break;

    case IIS2MDC_ODR_100Hz:
      *odr = 100.000f;
      break;

    default:
      *odr = -1.000f;
      return COMPONENT_ERROR;
  }
  return COMPONENT_OK;
}



/**
 * @brief Set the IIS2MDC sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2MDC_Set_ODR(DrvContextTypeDef *handle, SensorOdr_t odr)
{
  iis2mdc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  iis2mdc_odr_t new_odr;

  switch (odr)
  {
    case ODR_LOW:
      new_odr = IIS2MDC_ODR_10Hz;
      break;

    case ODR_MID_LOW:
      new_odr = IIS2MDC_ODR_20Hz;
      break;

    case ODR_MID:
      new_odr = IIS2MDC_ODR_50Hz;
      break;

    case ODR_MID_HIGH:
      new_odr = IIS2MDC_ODR_100Hz;
      break;

    case ODR_HIGH:
      new_odr = IIS2MDC_ODR_100Hz;
      break;

    default:
      return COMPONENT_ERROR;
  }

  if (iis2mdc_data_rate_set(&ctx, new_odr) == 1)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the IIS2MDC sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2MDC_Set_ODR_Value(DrvContextTypeDef *handle, float odr)
{
  iis2mdc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  iis2mdc_odr_t new_odr;

  new_odr = (odr <= 10.000f) ? IIS2MDC_ODR_10Hz
            : (odr <= 20.000f) ? IIS2MDC_ODR_20Hz
            : (odr <= 50.000f) ? IIS2MDC_ODR_50Hz
            :                      IIS2MDC_ODR_100Hz;

  if (iis2mdc_data_rate_set(&ctx, new_odr) == 1)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the IIS2MDC sensor full scale
 * @param handle the device handle
 * @param fullScale pointer where the full scale is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2MDC_Get_FS(DrvContextTypeDef *handle, float *fullScale)
{
  *fullScale = 50.0f;

  return COMPONENT_OK;
}



/**
 * @brief Set the IIS2MDC sensor full scale
 * @param handle the device handle
 * @param fullScale the functional full scale to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2MDC_Set_FS(DrvContextTypeDef *handle, SensorFs_t fullScale)
{
  return COMPONENT_OK;
}



/**
 * @brief Set the IIS2MDC sensor full scale
 * @param handle the device handle
 * @param fullScale the full scale value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2MDC_Set_FS_Value(DrvContextTypeDef *handle, float fullScale)
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
static DrvStatusTypeDef IIS2MDC_Read_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t *data)
{
  iis2mdc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  return (DrvStatusTypeDef)iis2mdc_read_reg(&ctx, reg, data, 1);
}



/**
 * @brief Write the data to register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2MDC_Write_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t data)
{
  iis2mdc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  return (DrvStatusTypeDef)iis2mdc_write_reg(&ctx, reg, &data, 1);
}



/**
 * @brief Get magnetometer data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2MDC_Get_DRDY_Status(DrvContextTypeDef *handle, uint8_t *status)
{
  iis2mdc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  if (iis2mdc_mag_data_ready_get(&ctx, status) == 1)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @}
 */

/** @addtogroup IIS2MDC_MAG_Private_Functions Private functions
 * @{
 */

/**
 * @brief Get the IIS2MDC sensor raw axes
 * @param handle the device handle
 * @param pData pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2MDC_Get_Axes_Raw(DrvContextTypeDef *handle, int16_t *pData)
{
  iis2mdc_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};
  int16_t *regValueInt16;

  /* Read output registers from IIS2MDC_MAG_OUTX_L to IIS2MDC_MAG_OUTZ_H. */
  if (iis2mdc_magnetic_raw_get(&ctx, regValue) == 1)
  {
    return COMPONENT_ERROR;
  }

  regValueInt16 = (int16_t *)regValue;

  /* Format the data. */
  pData[0] = regValueInt16[0];
  pData[1] = regValueInt16[1];
  pData[2] = regValueInt16[2];

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
