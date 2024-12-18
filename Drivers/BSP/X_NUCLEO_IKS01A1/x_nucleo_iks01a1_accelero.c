/**
 ******************************************************************************
 * @file    x_nucleo_iks01a1_accelero.c
 * @author  MEMS Application Team
 * @version V4.2.0
 * @date    01-February-2018
 * @brief   This file provides a set of functions needed to manage the accelerometer sensor
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

#include "x_nucleo_iks01a1_accelero.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1 X_NUCLEO_IKS01A1
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1_ACCELERO Accelerometer
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1_ACCELERO_Private_Variables Private variables
 * @{
 */

static DrvContextTypeDef ACCELERO_SensorHandle[ ACCELERO_SENSORS_MAX_NUM ];
static ACCELERO_Data_t ACCELERO_Data[ ACCELERO_SENSORS_MAX_NUM ]; // Accelerometer - all.
static LSM6DS0_X_Data_t LSM6DS0_X_0_Data;                         // Accelerometer - sensor 0.
static LSM6DS3_X_Data_t LSM6DS3_X_0_Data;                         // Accelerometer - sensor 1.
static LSM303AGR_X_Data_t LSM303AGR_X_0_Data;                     // Accelerometer - sensor 2.
static LIS2DH12_Data_t LIS2DH12_0_Data;                           // Accelerometer - sensor 3.
static H3LIS331DL_Data_t H3LIS331DL_0_Data;                       // Accelerometer - sensor 4.
static LIS2DW12_Data_t LIS2DW12_0_Data;                           // Accelerometer - sensor 5.
static LSM6DSL_X_Data_t LSM6DSL_X_0_Data;                         // Accelerometer - sensor 6.

/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A1_ACCELERO_Private_FunctionPrototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef BSP_LSM6DS0_ACCELERO_Init(void **handle);
static DrvStatusTypeDef BSP_LSM6DS3_ACCELERO_Init(void **handle);
static DrvStatusTypeDef BSP_LSM303AGR_ACCELERO_Init(void **handle);
static DrvStatusTypeDef BSP_LIS2DH12_ACCELERO_Init(void **handle);
static DrvStatusTypeDef BSP_H3LIS331DL_ACCELERO_Init(void **handle);
static DrvStatusTypeDef BSP_LIS2DW12_ACCELERO_Init(void **handle);
static DrvStatusTypeDef BSP_LSM6DSL_ACCELERO_Init(void **handle);

/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A1_ACCELERO_Public_Functions Public functions
 * @{
 */

/**
 * @brief Initialize an accelerometer sensor
 * @param id the accelerometer sensor identifier
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Init(ACCELERO_ID_t id, void **handle)
{

  *handle = NULL;

  switch (id)
  {
    default:
    case ACCELERO_SENSORS_AUTO:
      /* Try to init accelerometers with following priority order: */
      if (BSP_LSM6DS3_ACCELERO_Init(handle) == COMPONENT_OK)
      {
        return COMPONENT_OK;
      }
      if (BSP_H3LIS331DL_ACCELERO_Init(handle) == COMPONENT_OK)
      {
        return COMPONENT_OK;
      }
      if (BSP_LIS2DH12_ACCELERO_Init(handle) == COMPONENT_OK)
      {
        return COMPONENT_OK;
      }
      if (BSP_LIS2DW12_ACCELERO_Init(handle) == COMPONENT_OK)
      {
        return COMPONENT_OK;
      }
      if (BSP_LSM6DSL_ACCELERO_Init(handle) == COMPONENT_OK)
      {
        return COMPONENT_OK;
      }
      if (BSP_LSM303AGR_ACCELERO_Init(handle) == COMPONENT_OK)
      {
        return COMPONENT_OK;
      }
      if (BSP_LSM6DS0_ACCELERO_Init(handle) == COMPONENT_OK)
      {
        return COMPONENT_OK;
      }
      break;

    case LSM6DS0_X_0:
      return BSP_LSM6DS0_ACCELERO_Init(handle);

    case LSM6DS3_X_0:
      return BSP_LSM6DS3_ACCELERO_Init(handle);

    case LSM6DSL_X_0:
      return BSP_LSM6DSL_ACCELERO_Init(handle);

    case LSM303AGR_X_0:
      return BSP_LSM303AGR_ACCELERO_Init(handle);

    case LIS2DH12_0:
      return BSP_LIS2DH12_ACCELERO_Init(handle);

    case H3LIS331DL_0:
      return BSP_H3LIS331DL_ACCELERO_Init(handle);

    case LIS2DW12_0:
      return BSP_LIS2DW12_ACCELERO_Init(handle);
  }

  return COMPONENT_ERROR;
}



/**
 * @brief Deinitialize accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_DeInit(void **handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)(*handle);
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (driver->DeInit == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->DeInit(ctx) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  memset(ctx, 0, sizeof(DrvContextTypeDef));

  *handle = NULL;

  return COMPONENT_OK;
}



/**
 * @brief Enable accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Sensor_Enable(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (driver->Sensor_Enable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Sensor_Enable(ctx) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Disable accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Sensor_Disable(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (driver->Sensor_Disable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Sensor_Disable(ctx) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Check if the accelerometer sensor is initialized
 * @param handle the device handle
 * @param status the pointer to the initialization status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_IsInitialized(void *handle, uint8_t *status)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isInitialized;

  return COMPONENT_OK;
}


/**
 * @brief Check if the accelerometer sensor is enabled
 * @param handle the device handle
 * @param status the pointer to the enable status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_IsEnabled(void *handle, uint8_t *status)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isEnabled;

  return COMPONENT_OK;
}


/**
 * @brief Check if the accelerometer sensor is combo
 * @param handle the device handle
 * @param status the pointer to the combo status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_IsCombo(void *handle, uint8_t *status)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isCombo;

  return COMPONENT_OK;
}


/**
 * @brief Get the accelerometer sensor instance
 * @param handle the device handle
 * @param instance the pointer to the device instance
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Instance(void *handle, uint8_t *instance)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (instance == NULL)
  {
    return COMPONENT_ERROR;
  }

  *instance = ctx->instance;

  return COMPONENT_OK;
}



/**
 * @brief Get the WHO_AM_I ID of the accelerometer sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_WhoAmI(void *handle, uint8_t *who_am_i)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (driver->Get_WhoAmI == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Get_WhoAmI(ctx, who_am_i) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Check the WHO_AM_I ID of the accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Check_WhoAmI(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (driver->Check_WhoAmI == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Check_WhoAmI(ctx) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the accelerometer sensor axes
 * @param handle the device handle
 * @param acceleration pointer where the values of the axes are written [mg]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Axes(void *handle, SensorAxes_t *acceleration)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (acceleration == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Get_Axes == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Get_Axes(ctx, acceleration) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the accelerometer sensor raw axes
 * @param handle the device handle
 * @param value pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_AxesRaw(void *handle, SensorAxesRaw_t *value)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (value == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Get_AxesRaw == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Get_AxesRaw(ctx, value) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Get the accelerometer sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written [mg/LSB]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Sensitivity(void *handle, float *sensitivity)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (sensitivity == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Get_Sensitivity == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Get_Sensitivity(ctx, sensitivity) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_ODR(void *handle, float *odr)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (odr == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Get_ODR == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Get_ODR(ctx, odr) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_ODR(void *handle, SensorOdr_t odr)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (driver->Set_ODR == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Set_ODR(ctx, odr) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_ODR_Value(void *handle, float odr)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (driver->Set_ODR_Value == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Set_ODR_Value(ctx, odr) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale pointer where the full scale is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_FS(void *handle, float *fullScale)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (fullScale == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Get_FS == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Get_FS(ctx, fullScale) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale the functional full scale to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_FS(void *handle, SensorFs_t fullScale)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (driver->Set_FS == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Set_FS(ctx, fullScale) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale the full scale value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_FS_Value(void *handle, float fullScale)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (driver->Set_FS_Value == NULL)
  {
    return COMPONENT_ERROR;
  }
  if (driver->Set_FS_Value(ctx, fullScale) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the accelerometer sensor axes status
 * @param handle the device handle
 * @param xyz_enabled the pointer to the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Axes_Status(void *handle, uint8_t *xyz_enabled)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (xyz_enabled == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Get_Axes_Status == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Get_Axes_Status(ctx, xyz_enabled) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the enabled/disabled status of the accelerometer sensor axes
 * @param handle the device handle
 * @param enable_xyz vector of the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Axes_Status(void *handle, uint8_t *enable_xyz)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (enable_xyz == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Set_Axes_Status == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Set_Axes_Status(ctx, enable_xyz) == COMPONENT_ERROR)
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
DrvStatusTypeDef BSP_ACCELERO_Read_Reg(void *handle, uint8_t reg, uint8_t *data)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (data == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Read_Reg == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Read_Reg(ctx, reg, data) == COMPONENT_ERROR)
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
DrvStatusTypeDef BSP_ACCELERO_Write_Reg(void *handle, uint8_t reg, uint8_t data)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (driver->Write_Reg == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Write_Reg(ctx, reg, data) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get accelerometer data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_DRDY_Status(void *handle, uint8_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ACCELERO_Drv_t *driver = NULL;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = (ACCELERO_Drv_t *)ctx->pVTable;

  if (driver->Get_DRDY_Status == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (driver->Get_DRDY_Status(ctx, status) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A1_ACCELERO_Public_Functions_Ext Public functions for extended features
 * @{
 */

/**
 * @brief Enable the free fall detection (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param int_pin the interrupt pin to be used
 * @note  This function sets the LSM6DS3 accelerometer ODR to 416Hz and the LSM6DS3 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Enable_Free_Fall_Detection_Ext(void *handle, SensorIntPin_t int_pin)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Enable_Free_Fall_Detection == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Enable_Free_Fall_Detection(ctx, int_pin);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Disable the free fall detection (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Disable_Free_Fall_Detection_Ext(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Disable_Free_Fall_Detection == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Disable_Free_Fall_Detection(ctx);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get the status of the free fall detection (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param status the pointer to the status of free fall detection: 0 means no detection, 1 means detection happened
 * @note This function is deprecated and has been replaced by BSP_ACCELERO_Get_Event_Status_Ext
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Free_Fall_Detection_Status_Ext(void *handle, uint8_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_Free_Fall_Detection_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_Free_Fall_Detection_Status(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set the free fall detection threshold (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param thr the threshold to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Free_Fall_Threshold_Ext(void *handle, uint8_t thr)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Set_Free_Fall_Threshold == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Set_Free_Fall_Threshold(ctx, thr);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Enable the pedometer feature (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @note  This function sets the LSM6DS3 accelerometer ODR to 26Hz and the LSM6DS3 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Enable_Pedometer_Ext(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Enable_Pedometer == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Enable_Pedometer(ctx);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Disable the pedometer feature (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Disable_Pedometer_Ext(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Disable_Pedometer == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Disable_Pedometer(ctx);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get the pedometer status (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param status the pointer to the pedometer status: 0 means no step detected, 1 means step detected
 * @note This function is deprecated and has been replaced by BSP_ACCELERO_Get_Event_Status_Ext
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Pedometer_Status_Ext(void *handle, uint8_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_Pedometer_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_Pedometer_Status(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get the step counter (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param step_count the pointer to the step counter
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Step_Count_Ext(void *handle, uint16_t *step_count)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (step_count == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_Step_Count == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_Step_Count(ctx, step_count);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Reset of the step counter (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Reset_Step_Counter_Ext(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Enable_Step_Counter_Reset == NULL || extDriver->Disable_Step_Counter_Reset == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      if (extDriver->Enable_Step_Counter_Reset(ctx) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      HAL_Delay(10);

      if (extDriver->Disable_Step_Counter_Reset(ctx) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the pedometer threshold (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param thr the threshold to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Pedometer_Threshold_Ext(void *handle, uint8_t thr)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Set_Pedometer_Threshold == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Set_Pedometer_Threshold(ctx, thr);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Enable the tilt detection (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param int_pin the interrupt pin to be used
 * @note  This function sets the LSM6DS3 accelerometer ODR to 26Hz and the LSM6DS3 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Enable_Tilt_Detection_Ext(void *handle, SensorIntPin_t int_pin)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Enable_Tilt_Detection == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Enable_Tilt_Detection(ctx, int_pin);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Disable the tilt detection (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Disable_Tilt_Detection_Ext(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Disable_Tilt_Detection == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Disable_Tilt_Detection(ctx);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get the tilt detection status (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param status the pointer to the tilt detection status: 0 means no tilt detected, 1 means tilt detected
 * @note This function is deprecated and has been replaced by BSP_ACCELERO_Get_Event_Status_Ext
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Tilt_Detection_Status_Ext(void *handle, uint8_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_Tilt_Detection_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_Tilt_Detection_Status(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Enable the wake up detection (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param int_pin the interrupt pin to be used
 * @note  This function sets the LSM6DS3 accelerometer ODR to 416Hz and the LSM6DS3 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Enable_Wake_Up_Detection_Ext(void *handle, SensorIntPin_t int_pin)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Enable_Wake_Up_Detection == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Enable_Wake_Up_Detection(ctx, int_pin);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Disable the wake up detection (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Disable_Wake_Up_Detection_Ext(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Disable_Wake_Up_Detection == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Disable_Wake_Up_Detection(ctx);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get the status of the wake up detection (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param status the pointer to the status of the wake up detection: 0 means no detection, 1 means detection happened
 * @note This function is deprecated and has been replaced by BSP_ACCELERO_Get_Event_Status_Ext
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Wake_Up_Detection_Status_Ext(void *handle, uint8_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_Wake_Up_Detection_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_Wake_Up_Detection_Status(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set the wake up threshold (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param thr the threshold to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Wake_Up_Threshold_Ext(void *handle, uint8_t thr)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Set_Wake_Up_Threshold == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Set_Wake_Up_Threshold(ctx, thr);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Enable the single tap detection (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param int_pin the interrupt pin to be used
 * @note  This function sets the LSM6DS3 accelerometer ODR to 416Hz and the LSM6DS3 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Enable_Single_Tap_Detection_Ext(void *handle, SensorIntPin_t int_pin)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Enable_Single_Tap_Detection == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Enable_Single_Tap_Detection(ctx, int_pin);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Disable the single tap detection (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Disable_Single_Tap_Detection_Ext(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Disable_Single_Tap_Detection == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Disable_Single_Tap_Detection(ctx);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get the single tap detection status (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param status the pointer to the single tap detection status: 0 means no single tap detected, 1 means single tap detected
 * @note This function is deprecated and has been replaced by BSP_ACCELERO_Get_Event_Status_Ext
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Single_Tap_Detection_Status_Ext(void *handle, uint8_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_Single_Tap_Detection_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_Single_Tap_Detection_Status(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Enable the double tap detection (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param int_pin the interrupt pin to be used
 * @note  This function sets the LSM6DS3 accelerometer ODR to 416Hz and the LSM6DS3 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Enable_Double_Tap_Detection_Ext(void *handle, SensorIntPin_t int_pin)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Enable_Double_Tap_Detection == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Enable_Double_Tap_Detection(ctx, int_pin);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Disable the double tap detection (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Disable_Double_Tap_Detection_Ext(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Disable_Double_Tap_Detection == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Disable_Double_Tap_Detection(ctx);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get the double tap detection status (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param status the pointer to the double tap detection status: 0 means no double tap detected, 1 means double tap detected
 * @note This function is deprecated and has been replaced by BSP_ACCELERO_Get_Event_Status_Ext
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext(void *handle, uint8_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_Double_Tap_Detection_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_Double_Tap_Detection_Status(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set the tap threshold (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param thr the threshold to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Tap_Threshold_Ext(void *handle, uint8_t thr)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Set_Tap_Threshold == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Set_Tap_Threshold(ctx, thr);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set the tap shock time window (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param time the shock time window to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Tap_Shock_Time_Ext(void *handle, uint8_t time)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Set_Tap_Shock_Time == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Set_Tap_Shock_Time(ctx, time);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set the tap quiet time window (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param time the quiet time window to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Tap_Quiet_Time_Ext(void *handle, uint8_t time)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Set_Tap_Quiet_Time == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Set_Tap_Quiet_Time(ctx, time);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set the tap duration of the time window (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param time the duration of the time window to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Tap_Duration_Time_Ext(void *handle, uint8_t time)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Set_Tap_Duration_Time == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Set_Tap_Duration_Time(ctx, time);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Enable the 6D orientation detection (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param int_pin the interrupt pin to be used
 * @note  This function sets the LSM6DS3 accelerometer ODR to 416Hz and the LSM6DS3 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Enable_6D_Orientation_Ext(void *handle, SensorIntPin_t int_pin)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Enable_6D_Orientation == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Enable_6D_Orientation(ctx, int_pin);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Disable the 6D orientation detection (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Disable_6D_Orientation_Ext(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Disable_6D_Orientation == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Disable_6D_Orientation(ctx);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get the status of the 6D orientation detection (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param status the pointer to the status of the 6D orientation detection: 0 means no detection, 1 means detection happened
 * @note This function is deprecated and has been replaced by BSP_ACCELERO_Get_Event_Status_Ext
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_Status_Ext(void *handle, uint8_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_6D_Orientation_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_6D_Orientation_Status(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get the 6D orientation XL axis (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param xl the pointer to the 6D orientation XL axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_XL_Ext(void *handle, uint8_t *xl)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (xl == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_6D_Orientation_XL == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_6D_Orientation_XL(ctx, xl);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get the 6D orientation XH axis (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param xh the pointer to the 6D orientation XH axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_XH_Ext(void *handle, uint8_t *xh)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (xh == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_6D_Orientation_XH == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_6D_Orientation_XH(ctx, xh);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get the 6D orientation YL axis (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param yl the pointer to the 6D orientation YL axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_YL_Ext(void *handle, uint8_t *yl)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (yl == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_6D_Orientation_YL == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_6D_Orientation_YL(ctx, yl);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get the 6D orientation YH axis (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param yh the pointer to the 6D orientation YH axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_YH_Ext(void *handle, uint8_t *yh)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (yh == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_6D_Orientation_YH == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_6D_Orientation_YH(ctx, yh);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get the 6D orientation ZL axis (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param zl the pointer to the 6D orientation ZL axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_ZL_Ext(void *handle, uint8_t *zl)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (zl == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_6D_Orientation_ZL == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_6D_Orientation_ZL(ctx, zl);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get the 6D orientation ZH axis (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param zh the pointer to the 6D orientation ZH axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_ZH_Ext(void *handle, uint8_t *zh)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (zh == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_6D_Orientation_ZH == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_6D_Orientation_ZH(ctx, zh);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}


/**
 * @brief Get the status of all hardware events (available only for LSM6DS3 sensor)
 * @param handle the device handle
 * @param status the pointer to the status of all hardware events
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Event_Status_Ext(void *handle, ACCELERO_Event_Status_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Get_Event_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Get_Event_Status(ctx, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}


/**
 * @brief Set FIFO output data rate (available only for LSM6DS3 and LSM6DSL sensors)
 * @param handle the device handle
 * @param odr the output data rate
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_ODR_Value_Ext(void *handle, float odr)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 and LSM6DSL */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Set_ODR_Value == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_ODR_Value(handle, odr);
    }
  }

  else if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
  {
    LSM6DSL_X_ExtDrv_t *extDriver = (LSM6DSL_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Set_ODR_Value == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_ODR_Value(handle, odr);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get FIFO full status (available only for LSM6DS3 and LSM6DSL sensors)
 * @param handle the device handle
 * @param *status FIFO full status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Full_Status_Ext(void *handle, uint8_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 and LSM6DSL */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Get_Full_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Full_Status(handle, status);
    }
  }

  else if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
  {
    LSM6DSL_X_ExtDrv_t *extDriver = (LSM6DSL_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Get_Full_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Full_Status(handle, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get FIFO empty status (available only for LSM6DS3 and LSM6DSL sensors)
 * @param handle the device handle
 * @param *status FIFO empty status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Empty_Status_Ext(void *handle, uint8_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 and LSM6DSL */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Get_Empty_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Empty_Status(handle, status);
    }
  }

  else if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
  {
    LSM6DSL_X_ExtDrv_t *extDriver = (LSM6DSL_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Get_Empty_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Empty_Status(handle, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get FIFO_OVR bit status (available only for LSM6DS3, LSM6DSL and LIS2DH12 sensors)
 * @param handle the device handle
 * @param *status FIFO_OVR bit status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Overrun_Status_Ext(void *handle, uint8_t *status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  void *extDriver = ctx->pExtVTable;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (status == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3, LSM6DSL and LIS2DH12 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    if (((LSM6DS3_X_ExtDrv_t *)extDriver)->FIFO_Get_Overrun_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return ((LSM6DS3_X_ExtDrv_t *)extDriver)->FIFO_Get_Overrun_Status(handle, status);
    }
  }

  else if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
  {
    if (((LSM6DSL_X_ExtDrv_t *)extDriver)->FIFO_Get_Overrun_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return ((LSM6DSL_X_ExtDrv_t *)extDriver)->FIFO_Get_Overrun_Status(handle, status);
    }
  }

  else if (ctx->who_am_i == LIS2DH12_WHO_AM_I)
  {
    if (((LIS2DH12_ExtDrv_t *)extDriver)->FIFO_Get_Overrun_Status == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return ((LIS2DH12_ExtDrv_t *)extDriver)->FIFO_Get_Overrun_Status(handle, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get FIFO pattern (available only for LSM6DS3 and LSM6DSL sensors)
 * @param handle the device handle
 * @param *pattern FIFO pattern
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Pattern_Ext(void *handle, uint16_t *pattern)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (pattern == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 and LSM6DSL */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Get_Pattern == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Pattern(handle, pattern);
    }
  }

  else if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
  {
    LSM6DSL_X_ExtDrv_t *extDriver = (LSM6DSL_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Get_Pattern == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Pattern(handle, pattern);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get FIFO data (available only for LSM6DS3 and LSM6DSL sensors)
 * @param handle the device handle
 * @param *aData FIFO data array
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Data_Ext(void *handle, uint8_t *aData)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (aData == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 and LSM6DSL */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Get_Data == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Data(handle, aData);
    }
  }

  else if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
  {
    LSM6DSL_X_ExtDrv_t *extDriver = (LSM6DSL_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Get_Data == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Data(handle, aData);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get number of unread FIFO samples (available only for LSM6DS3, LSM6DSL and LIS2DH12 sensors)
 * @param handle the device handle
 * @param *nSamples Number of unread FIFO samples
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Num_Of_Samples_Ext(void *handle, uint16_t *nSamples)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  void *extDriver = ctx->pExtVTable;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (nSamples == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3, LSM6DSL and LIS2DH12 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    if (((LSM6DS3_X_ExtDrv_t *)extDriver)->FIFO_Get_Num_Of_Samples == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return ((LSM6DS3_X_ExtDrv_t *)extDriver)->FIFO_Get_Num_Of_Samples(handle, nSamples);
    }
  }

  else if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
  {
    if (((LSM6DSL_X_ExtDrv_t *)extDriver)->FIFO_Get_Num_Of_Samples == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return ((LSM6DSL_X_ExtDrv_t *)extDriver)->FIFO_Get_Num_Of_Samples(handle, nSamples);
    }
  }

  else if (ctx->who_am_i == LIS2DH12_WHO_AM_I)
  {
    if (((LIS2DH12_ExtDrv_t *)extDriver)->FIFO_Get_Num_Of_Samples == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return ((LIS2DH12_ExtDrv_t *)extDriver)->FIFO_Get_Num_Of_Samples(handle, nSamples);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set FIFO decimation for accelerometer (available only for LSM6DS3 and LSM6DSL sensors)
 * @param handle the device handle
 * @param decimation FIFO decimation for accelerometer
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_Decimation_Ext(void *handle, uint8_t decimation)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 and LSM6DSL */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_X_Set_Decimation == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_X_Set_Decimation(handle, decimation);
    }
  }

  else if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
  {
    LSM6DSL_X_ExtDrv_t *extDriver = (LSM6DSL_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_X_Set_Decimation == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_X_Set_Decimation(handle, decimation);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get single accelero axis from the FIFO (available only for LSM6DS3 and LSM6DSL sensors)
 * @param handle the device handle
 * @param acceleration the pointer to the acceleration value
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Axis_Ext(void *handle, int32_t *acceleration)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (acceleration == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 and LSM6DSL */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_X_Get_Axis == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_X_Get_Axis(handle, acceleration);
    }
  }

  if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
  {
    LSM6DSL_X_ExtDrv_t *extDriver = (LSM6DSL_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_X_Get_Axis == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_X_Get_Axis(handle, acceleration);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set FIFO mode
 * @param handle the device handle
 * @param mode FIFO mode
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_Mode_Ext(void *handle, uint8_t mode)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  void *extDriver = ctx->pExtVTable;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3, LSM6DSL and LIS2DH12 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    if (((LSM6DS3_X_ExtDrv_t *)extDriver)->FIFO_Set_Mode == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return ((LSM6DS3_X_ExtDrv_t *)extDriver)->FIFO_Set_Mode(handle, mode);
    }
  }

  else if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
  {
    if (((LSM6DSL_X_ExtDrv_t *)extDriver)->FIFO_Set_Mode == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return ((LSM6DSL_X_ExtDrv_t *)extDriver)->FIFO_Set_Mode(handle, mode);
    }
  }

  else if (ctx->who_am_i == LIS2DH12_WHO_AM_I)
  {
    if (((LIS2DH12_ExtDrv_t *)extDriver)->FIFO_Set_Mode == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return ((LIS2DH12_ExtDrv_t *)extDriver)->FIFO_Set_Mode(handle, mode);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set FIFO_FULL interrupt on INT1 pin
 * @param handle the device handle
 * @param status FIFO_FULL interrupt on INT1 pin enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_INT1_FIFO_Full_Ext(void *handle, uint8_t status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 and LSM6DSL */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Set_INT1_FIFO_Full == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_INT1_FIFO_Full(handle, status);
    }
  }

  if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
  {
    LSM6DSL_X_ExtDrv_t *extDriver = (LSM6DSL_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Set_INT1_FIFO_Full == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_INT1_FIFO_Full(handle, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set FIFO_OVR interrupt on INT1 pin
 * @param handle the device handle
 * @param status FIFO_OVR interrupt on INT1 pin enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_INT1_FIFO_Overrun_Ext(void *handle, uint8_t status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LIS2DH12 */
  if (ctx->who_am_i == LIS2DH12_WHO_AM_I)
  {
    LIS2DH12_ExtDrv_t *extDriver = (LIS2DH12_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Set_INT1_FIFO_Overrun == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_INT1_FIFO_Overrun(handle, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set FIFO watermark level
 * @param handle the device handle
 * @param watermark FIFO watermark level
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_Watermark_Level_Ext(void *handle, uint16_t watermark)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 and LSM6DSL */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Set_Watermark_Level == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_Watermark_Level(handle, watermark);
    }
  }

  else if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
  {
    LSM6DSL_X_ExtDrv_t *extDriver = (LSM6DSL_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Set_Watermark_Level == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_Watermark_Level(handle, watermark);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set FIFO to stop on FTH interrupt
 * @param handle the device handle
 * @param status FIFO stop on FTH interrupt enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_Stop_On_Fth_Ext(void *handle, uint8_t status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 and LSM6DSL */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Set_Stop_On_Fth == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_Stop_On_Fth(handle, status);
    }
  }

  else if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
  {
    LSM6DSL_X_ExtDrv_t *extDriver = (LSM6DSL_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->FIFO_Set_Stop_On_Fth == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_Stop_On_Fth(handle, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set interrupt latch
 * @param handle the device handle
 * @param status interrupt latch enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Interrupt_Latch_Ext(void *handle, uint8_t status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Set_Interrupt_Latch == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Set_Interrupt_Latch(handle, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set accelero self-test
 * @param handle the device handle
 * @param status self-test enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_SelfTest_Ext(void *handle, uint8_t status)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DS3 */
  if (ctx->who_am_i == LSM6DS3_ACC_GYRO_WHO_AM_I)
  {
    LSM6DS3_X_ExtDrv_t *extDriver = (LSM6DS3_X_ExtDrv_t *)ctx->pExtVTable;

    if (extDriver->Set_SelfTest == NULL)
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Set_SelfTest(handle, status);
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get the accelerometer sensor super raw data from one axis
 * @param handle the device handle
 * @param acceleration pointer where the super raw value of the axis is written
 * @param axis axis to be read
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_SuperRawAxes_Ext(void *handle, int16_t *acceleration, ACTIVE_AXIS_t axis)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->instance == LIS2DH12_0)
  {
    if (ctx->who_am_i == LIS2DH12_WHO_AM_I)
    {
      LIS2DH12_ExtDrv_t *extDriver = (LIS2DH12_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Get_AxesSuperRaw == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Get_AxesSuperRaw(ctx, acceleration, axis) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM303AGR_X_0)
  {
    if (ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I)
    {
      LSM303AGR_X_ExtDrv_t *extDriver = (LSM303AGR_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Get_AxesSuperRaw == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Get_AxesSuperRaw(ctx, acceleration, axis) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM6DS0_X_0)
  {
    if (ctx->who_am_i == LSM6DS0_ACC_GYRO_WHO_AM_I)
    {
      LSM6DS0_X_ExtDrv_t *extDriver = (LSM6DS0_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Get_AxesSuperRaw == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Get_AxesSuperRaw(ctx, acceleration, axis) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }

  return COMPONENT_ERROR;
}



/**
 * @brief Get the accelerometer sensor operating mode
 * @param handle the device handle
 * @param opMode pointer where the operating mode value is stored
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_OpMode_Ext(void *handle, OP_MODE_t *opMode)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->instance == LIS2DH12_0)
  {
    if (ctx->who_am_i == LIS2DH12_WHO_AM_I)
    {
      LIS2DH12_ExtDrv_t *extDriver = (LIS2DH12_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Get_OpMode == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Get_OpMode(ctx, opMode) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM303AGR_X_0)
  {
    if (ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I)
    {
      LSM303AGR_X_ExtDrv_t *extDriver = (LSM303AGR_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Get_OpMode == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Get_OpMode(ctx, opMode) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }

  return COMPONENT_ERROR;
}



/**
 * @brief Set the accelerometer sensor operating mode
 * @param handle the device handle
 * @param opMode the value of operating mode to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_OpMode_Ext(void *handle, OP_MODE_t opMode)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->instance == LIS2DH12_0)
  {
    if (ctx->who_am_i == LIS2DH12_WHO_AM_I)
    {
      LIS2DH12_ExtDrv_t *extDriver = (LIS2DH12_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Set_OpMode == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Set_OpMode(ctx, opMode) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM303AGR_X_0)
  {
    if (ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I)
    {
      LSM303AGR_X_ExtDrv_t *extDriver = (LSM303AGR_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Set_OpMode == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Set_OpMode(ctx, opMode) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }

  return COMPONENT_ERROR;
}



/**
 * @brief Set the accelerometer sensor active axis
 * @param handle the device handle
 * @param axis pointer where the active axis value is stored
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Active_Axis_Ext(void *handle, ACTIVE_AXIS_t *axis)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->instance == LIS2DH12_0)
  {
    if (ctx->who_am_i == LIS2DH12_WHO_AM_I)
    {
      LIS2DH12_ExtDrv_t *extDriver = (LIS2DH12_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Get_Active_Axis == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Get_Active_Axis(ctx, axis) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM303AGR_X_0)
  {
    if (ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I)
    {
      LSM303AGR_X_ExtDrv_t *extDriver = (LSM303AGR_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Get_Active_Axis == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Get_Active_Axis(ctx, axis) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM6DSL_X_0)
  {
    if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
    {
      LSM6DSL_X_ExtDrv_t *extDriver = (LSM6DSL_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Get_Active_Axis == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Get_Active_Axis(ctx, axis) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM6DS0_X_0)
  {
    if (ctx->who_am_i == LSM6DS0_ACC_GYRO_WHO_AM_I)
    {
      LSM6DS0_X_ExtDrv_t *extDriver = (LSM6DS0_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Get_Active_Axis == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Get_Active_Axis(ctx, axis) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }

  return COMPONENT_ERROR;
}



/**
 * @brief Set the accelerometer sensor active axis
 * @param handle the device handle
 * @param axis the value of active axis to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_Active_Axis_Ext(void *handle, ACTIVE_AXIS_t axis)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->instance == LIS2DH12_0)
  {
    if (ctx->who_am_i == LIS2DH12_WHO_AM_I)
    {
      LIS2DH12_ExtDrv_t *extDriver = (LIS2DH12_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Set_Active_Axis == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Set_Active_Axis(ctx, axis) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM303AGR_X_0)
  {
    if (ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I)
    {
      LSM303AGR_X_ExtDrv_t *extDriver = (LSM303AGR_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Set_Active_Axis == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Set_Active_Axis(ctx, axis) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM6DSL_X_0)
  {
    if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
    {
      LSM6DSL_X_ExtDrv_t *extDriver = (LSM6DSL_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Set_Active_Axis == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Set_Active_Axis(ctx, axis) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM6DS0_X_0)
  {
    if (ctx->who_am_i == LSM6DS0_ACC_GYRO_WHO_AM_I)
    {
      LSM6DS0_X_ExtDrv_t *extDriver = (LSM6DS0_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Set_Active_Axis == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Set_Active_Axis(ctx, axis) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }

  return COMPONENT_ERROR;
}



/**
 * @brief Enable the accelerometer sensor HP Filter
 * @param handle the device handle
 * @param mode the value of HP Filter mode to be set
 * @param cutoff the value of HP Filter cutoff to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Enable_HP_Filter_Ext(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->instance == LIS2DH12_0)
  {
    if (ctx->who_am_i == LIS2DH12_WHO_AM_I)
    {
      LIS2DH12_ExtDrv_t *extDriver = (LIS2DH12_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Enable_HP_Filter == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Enable_HP_Filter(ctx) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM303AGR_X_0)
  {
    if (ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I)
    {
      LSM303AGR_X_ExtDrv_t *extDriver = (LSM303AGR_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Enable_HP_Filter == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Enable_HP_Filter(ctx) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM6DSL_X_0)
  {
    if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
    {
      LSM6DSL_X_ExtDrv_t *extDriver = (LSM6DSL_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Enable_HP_Filter == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Enable_HP_Filter(ctx) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM6DS0_X_0)
  {
    if (ctx->who_am_i == LSM6DS0_ACC_GYRO_WHO_AM_I)
    {
      LSM6DS0_X_ExtDrv_t *extDriver = (LSM6DS0_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Enable_HP_Filter == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Enable_HP_Filter(ctx) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }

  return COMPONENT_ERROR;
}



/**
 * @brief Disable the accelerometer sensor HP Filter
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Disable_HP_Filter_Ext(void *handle)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->instance == LIS2DH12_0)
  {
    if (ctx->who_am_i == LIS2DH12_WHO_AM_I)
    {
      LIS2DH12_ExtDrv_t *extDriver = (LIS2DH12_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Disable_HP_Filter == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Disable_HP_Filter(ctx) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM303AGR_X_0)
  {
    if (ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I)
    {
      LSM303AGR_X_ExtDrv_t *extDriver = (LSM303AGR_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Disable_HP_Filter == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Disable_HP_Filter(ctx) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM6DSL_X_0)
  {
    if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
    {
      LSM6DSL_X_ExtDrv_t *extDriver = (LSM6DSL_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Disable_HP_Filter == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Disable_HP_Filter(ctx) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM6DS0_X_0)
  {
    if (ctx->who_am_i == LSM6DS0_ACC_GYRO_WHO_AM_I)
    {
      LSM6DS0_X_ExtDrv_t *extDriver = (LSM6DS0_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Disable_HP_Filter == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Disable_HP_Filter(ctx) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }

  return COMPONENT_ERROR;
}



/**
 * @brief Clear the accelerometer sensor DRDY
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_ClearDRDY_Ext(void *handle, ACTIVE_AXIS_t axisActive)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->instance == LIS2DH12_0)
  {
    if (ctx->who_am_i == LIS2DH12_WHO_AM_I)
    {
      LIS2DH12_ExtDrv_t *extDriver = (LIS2DH12_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->ClearDRDY == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->ClearDRDY(ctx, axisActive) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM303AGR_X_0)
  {
    if (ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I)
    {
      LSM303AGR_X_ExtDrv_t *extDriver = (LSM303AGR_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->ClearDRDY == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->ClearDRDY(ctx, axisActive) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM6DSL_X_0)
  {
    if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
    {
      LSM6DSL_X_ExtDrv_t *extDriver = (LSM6DSL_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->ClearDRDY == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->ClearDRDY(ctx, axisActive) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM6DS0_X_0)
  {
    if (ctx->who_am_i == LSM6DS0_ACC_GYRO_WHO_AM_I)
    {
      LSM6DS0_X_ExtDrv_t *extDriver = (LSM6DS0_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->ClearDRDY == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->ClearDRDY(ctx, axisActive) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }

  return COMPONENT_ERROR;
}



/**
 * @brief Set the accelerometer sensor DRDY on INT1
 * @param handle the device handle
 * @param drdyStatus enable/disable DRDY on INT1 value
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_INT1_DRDY_Ext(void *handle, INT1_DRDY_CONFIG_t drdyStatus)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->instance == LIS2DH12_0)
  {
    if (ctx->who_am_i == LIS2DH12_WHO_AM_I)
    {
      LIS2DH12_ExtDrv_t *extDriver = (LIS2DH12_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Set_INT1_DRDY == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Set_INT1_DRDY(ctx, drdyStatus) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM303AGR_X_0)
  {
    if (ctx->who_am_i == LSM303AGR_ACC_WHO_AM_I)
    {
      LSM303AGR_X_ExtDrv_t *extDriver = (LSM303AGR_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Set_INT1_DRDY == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Set_INT1_DRDY(ctx, drdyStatus) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM6DSL_X_0)
  {
    if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
    {
      LSM6DSL_X_ExtDrv_t *extDriver = (LSM6DSL_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Set_INT1_DRDY == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Set_INT1_DRDY(ctx, drdyStatus) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }
  else if (ctx->instance == LSM6DS0_X_0)
  {
    if (ctx->who_am_i == LSM6DS0_ACC_GYRO_WHO_AM_I)
    {
      LSM6DS0_X_ExtDrv_t *extDriver = (LSM6DS0_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Set_INT1_DRDY == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Set_INT1_DRDY(ctx, drdyStatus) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }

  return COMPONENT_ERROR;
}



/**
 * @brief Set the sensor DRDY mode
 * @param handle the device handle
 * @param drdyMode mode of DRDY
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Set_DRDY_Mode_Ext(void *handle, DRDY_MODE_t drdyMode)
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if (ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->pExtVTable == NULL)
  {
    return COMPONENT_ERROR;
  }

  if (ctx->instance == LSM6DSL_X_0)
  {
    if (ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I)
    {
      LSM6DSL_X_ExtDrv_t *extDriver = (LSM6DSL_X_ExtDrv_t *)ctx->pExtVTable;

      if (extDriver->Set_DRDY_Mode == NULL)
      {
        return COMPONENT_ERROR;
      }

      if (extDriver->Set_DRDY_Mode(ctx, drdyMode) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }

      return COMPONENT_OK;
    }
  }

  return COMPONENT_ERROR;
}



/**
 * @brief Check if LSM6DS0 sensor is available
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_LSM6DS0_AVAIL_Ext(void **handle)
{
  ACCELERO_Drv_t *driver = NULL;

  if (Sensor_IO_Init() == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  ACCELERO_SensorHandle[ TEST_X_0 ].who_am_i      = LSM6DS0_ACC_GYRO_WHO_AM_I;
  ACCELERO_SensorHandle[ TEST_X_0 ].address       = LSM6DS0_ACC_GYRO_I2C_ADDRESS_HIGH;
  ACCELERO_SensorHandle[ TEST_X_0 ].instance      = LSM6DS0_X_0;
  ACCELERO_SensorHandle[ TEST_X_0 ].isInitialized = 0;
  ACCELERO_SensorHandle[ TEST_X_0 ].isEnabled     = 0;
  ACCELERO_SensorHandle[ TEST_X_0 ].isCombo       = 1;
  ACCELERO_SensorHandle[ TEST_X_0 ].pData         = (void *)&ACCELERO_Data[ LSM6DS0_X_0 ];
  ACCELERO_SensorHandle[ TEST_X_0 ].pVTable       = (void *)&LSM6DS0_X_Drv;
  ACCELERO_SensorHandle[ TEST_X_0 ].pExtVTable    = (void *)&LSM6DS0_X_ExtDrv;
  LSM6DS0_X_0_Data.comboData                      = &LSM6DS0_Combo_Data[0];
  ACCELERO_Data[ TEST_X_0 ].pComponentData        = (void *)&LSM6DS0_X_0_Data;
  ACCELERO_Data[ TEST_X_0 ].pExtData              = 0;

  *handle = (void *)&ACCELERO_SensorHandle[ TEST_X_0 ];
  driver = (ACCELERO_Drv_t *)((DrvContextTypeDef *)(*handle))->pVTable;

  if (driver->Check_WhoAmI((DrvContextTypeDef *)(*handle)) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  memset((*handle), 0, sizeof(DrvContextTypeDef));
  *handle = NULL;

  return COMPONENT_OK;
}



/**
 * @brief Check if LSM6DSL sensor is available
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_LSM6DSL_AVAIL_Ext(void **handle)
{
  ACCELERO_Drv_t *driver = NULL;

  if (Sensor_IO_Init() == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  ACCELERO_SensorHandle[ TEST_X_0 ].who_am_i      = LSM6DSL_ACC_GYRO_WHO_AM_I;
  ACCELERO_SensorHandle[ TEST_X_0 ].address       = LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW;
  ACCELERO_SensorHandle[ TEST_X_0 ].instance      = LSM6DSL_X_0;
  ACCELERO_SensorHandle[ TEST_X_0 ].isInitialized = 0;
  ACCELERO_SensorHandle[ TEST_X_0 ].isEnabled     = 0;
  ACCELERO_SensorHandle[ TEST_X_0 ].isCombo       = 1;
  ACCELERO_SensorHandle[ TEST_X_0 ].pData         = (void *)&ACCELERO_Data[ LSM6DSL_X_0 ];
  ACCELERO_SensorHandle[ TEST_X_0 ].pVTable       = (void *)&LSM6DSL_X_Drv;
  ACCELERO_SensorHandle[ TEST_X_0 ].pExtVTable    = (void *)&LSM6DSL_X_ExtDrv;
  LSM6DSL_X_0_Data.comboData                      = &LSM6DSL_Combo_Data[0];
  ACCELERO_Data[ TEST_X_0 ].pComponentData        = (void *)&LSM6DSL_X_0_Data;
  ACCELERO_Data[ TEST_X_0 ].pExtData              = 0;

  *handle = (void *)&ACCELERO_SensorHandle[ TEST_X_0 ];
  driver = (ACCELERO_Drv_t *)((DrvContextTypeDef *)(*handle))->pVTable;

  if (driver->Check_WhoAmI((DrvContextTypeDef *)(*handle)) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  memset((*handle), 0, sizeof(DrvContextTypeDef));
  *handle = NULL;

  return COMPONENT_OK;
}



/**
 * @brief Check if LSM303AGR sensor is available
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_LSM303AGR_AVAIL_Ext(void **handle)
{
  ACCELERO_Drv_t *driver = NULL;

  if (Sensor_IO_Init() == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  ACCELERO_SensorHandle[ TEST_X_0 ].who_am_i      = LSM303AGR_ACC_WHO_AM_I;
  ACCELERO_SensorHandle[ TEST_X_0 ].address       = LSM303AGR_ACC_I2C_ADDRESS;
  ACCELERO_SensorHandle[ TEST_X_0 ].instance      = LSM303AGR_X_0;
  ACCELERO_SensorHandle[ TEST_X_0 ].isInitialized = 0;
  ACCELERO_SensorHandle[ TEST_X_0 ].isEnabled     = 0;
  ACCELERO_SensorHandle[ TEST_X_0 ].isCombo       = 1;
  ACCELERO_SensorHandle[ TEST_X_0 ].pData         = (void *)&ACCELERO_Data[ LSM303AGR_X_0 ];
  ACCELERO_SensorHandle[ TEST_X_0 ].pVTable       = (void *)&LSM303AGR_X_Drv;
  ACCELERO_SensorHandle[ TEST_X_0 ].pExtVTable    = (void *)&LSM303AGR_X_ExtDrv;
  ACCELERO_Data[ TEST_X_0 ].pComponentData        = (void *)&LSM303AGR_X_0_Data;
  ACCELERO_Data[ TEST_X_0 ].pExtData              = 0;

  *handle = (void *)&ACCELERO_SensorHandle[ TEST_X_0 ];
  driver = (ACCELERO_Drv_t *)((DrvContextTypeDef *)(*handle))->pVTable;

  if (driver->Check_WhoAmI((DrvContextTypeDef *)(*handle)) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  memset((*handle), 0, sizeof(DrvContextTypeDef));
  *handle = NULL;

  return COMPONENT_OK;
}



/**
 * @brief Check if LIS2DH12 sensor is available
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_LIS2DH12_AVAIL_Ext(void **handle)
{
  ACCELERO_Drv_t *driver = NULL;

  if (Sensor_IO_Init() == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  ACCELERO_SensorHandle[ TEST_X_0 ].who_am_i      = LIS2DH12_WHO_AM_I;
  ACCELERO_SensorHandle[ TEST_X_0 ].address       = LIS2DH12_I2C_ADDRESS_LOW;
  ACCELERO_SensorHandle[ TEST_X_0 ].instance      = LIS2DH12_0;
  ACCELERO_SensorHandle[ TEST_X_0 ].isInitialized = 0;
  ACCELERO_SensorHandle[ TEST_X_0 ].isEnabled     = 0;
  ACCELERO_SensorHandle[ TEST_X_0 ].isCombo       = 0;
  ACCELERO_SensorHandle[ TEST_X_0 ].pData         = (void *)&ACCELERO_Data[ LIS2DH12_0 ];
  ACCELERO_SensorHandle[ TEST_X_0 ].pVTable       = (void *)&LIS2DH12_Drv;
  ACCELERO_SensorHandle[ TEST_X_0 ].pExtVTable    = (void *)&LIS2DH12_ExtDrv;
  ACCELERO_Data[ TEST_X_0 ].pComponentData        = (void *)&LIS2DH12_0_Data;
  ACCELERO_Data[ TEST_X_0 ].pExtData              = 0;

  *handle = (void *)&ACCELERO_SensorHandle[ TEST_X_0 ];
  driver = (ACCELERO_Drv_t *)((DrvContextTypeDef *)(*handle))->pVTable;

  if (driver->Check_WhoAmI((DrvContextTypeDef *)(*handle)) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  memset((*handle), 0, sizeof(DrvContextTypeDef));
  *handle = NULL;

  return COMPONENT_OK;
}

/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A1_ACCELERO_Private_Functions Private functions
 * @{
 */

/**
 * @brief Initialize an LSM6DS0 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef BSP_LSM6DS0_ACCELERO_Init(void **handle)
{
  ACCELERO_Drv_t *driver = NULL;

  if (Sensor_IO_Init() == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  ACCELERO_SensorHandle[ LSM6DS0_X_0 ].who_am_i      = LSM6DS0_ACC_GYRO_WHO_AM_I;
  ACCELERO_SensorHandle[ LSM6DS0_X_0 ].ifType        = 0; /* I2C interface */
  ACCELERO_SensorHandle[ LSM6DS0_X_0 ].address       = LSM6DS0_ACC_GYRO_I2C_ADDRESS_HIGH;
  ACCELERO_SensorHandle[ LSM6DS0_X_0 ].instance      = LSM6DS0_X_0;
  ACCELERO_SensorHandle[ LSM6DS0_X_0 ].isInitialized = 0;
  ACCELERO_SensorHandle[ LSM6DS0_X_0 ].isEnabled     = 0;
  ACCELERO_SensorHandle[ LSM6DS0_X_0 ].isCombo       = 1;
  ACCELERO_SensorHandle[ LSM6DS0_X_0 ].pData         = (void *)&ACCELERO_Data[ LSM6DS0_X_0 ];
  ACCELERO_SensorHandle[ LSM6DS0_X_0 ].pVTable       = (void *)&LSM6DS0_X_Drv;
  ACCELERO_SensorHandle[ LSM6DS0_X_0 ].pExtVTable    = (void *)&LSM6DS0_X_ExtDrv;

  LSM6DS0_X_0_Data.comboData = &LSM6DS0_Combo_Data[0];
  ACCELERO_Data[ LSM6DS0_X_0 ].pComponentData = (void *)&LSM6DS0_X_0_Data;
  ACCELERO_Data[ LSM6DS0_X_0 ].pExtData       = 0;

  *handle = (void *)&ACCELERO_SensorHandle[ LSM6DS0_X_0 ];

  driver = (ACCELERO_Drv_t *)((DrvContextTypeDef *)(*handle))->pVTable;

  if (driver->Init == NULL)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if (driver->Init((DrvContextTypeDef *)(*handle)) == COMPONENT_ERROR)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  /* Configure interrupt lines for LSM6DS0 */
  LSM6DS0_Sensor_IO_ITConfig();

  return COMPONENT_OK;
}



/**
 * @brief Initialize an LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef BSP_LSM6DS3_ACCELERO_Init(void **handle)
{
  ACCELERO_Drv_t *driver = NULL;

  if (Sensor_IO_Init() == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].who_am_i      = LSM6DS3_ACC_GYRO_WHO_AM_I;
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].ifType        = 0; /* I2C interface */
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].address       = LSM6DS3_ACC_GYRO_I2C_ADDRESS_LOW;
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].instance      = LSM6DS3_X_0;
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].isInitialized = 0;
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].isEnabled     = 0;
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].isCombo       = 1;
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].pData         = (void *)&ACCELERO_Data[ LSM6DS3_X_0 ];
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].pVTable       = (void *)&LSM6DS3_X_Drv;
  ACCELERO_SensorHandle[ LSM6DS3_X_0 ].pExtVTable    = (void *)&LSM6DS3_X_ExtDrv;

  ACCELERO_Data[ LSM6DS3_X_0 ].pComponentData = (void *)&LSM6DS3_X_0_Data;
  ACCELERO_Data[ LSM6DS3_X_0 ].pExtData       = 0;

  *handle = (void *)&ACCELERO_SensorHandle[ LSM6DS3_X_0 ];

  driver = (ACCELERO_Drv_t *)((DrvContextTypeDef *)(*handle))->pVTable;

  if (driver->Init == NULL)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if (driver->Init((DrvContextTypeDef *)(*handle)) == COMPONENT_ERROR)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  /* Configure interrupt lines common for all sensors in DIL24 socket */
  DIL24_Sensor_IO_ITConfig();

  return COMPONENT_OK;
}



/**
 * @brief Initialize an LSM6DSL accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef BSP_LSM6DSL_ACCELERO_Init(void **handle)
{
  ACCELERO_Drv_t *driver = NULL;

  if (Sensor_IO_Init() == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  ACCELERO_SensorHandle[ LSM6DSL_X_0 ].who_am_i      = LSM6DSL_ACC_GYRO_WHO_AM_I;
  ACCELERO_SensorHandle[ LSM6DSL_X_0 ].ifType        = 0; /* I2C interface */
  ACCELERO_SensorHandle[ LSM6DSL_X_0 ].address       = LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW;
  ACCELERO_SensorHandle[ LSM6DSL_X_0 ].instance      = LSM6DSL_X_0;
  ACCELERO_SensorHandle[ LSM6DSL_X_0 ].isInitialized = 0;
  ACCELERO_SensorHandle[ LSM6DSL_X_0 ].isEnabled     = 0;
  ACCELERO_SensorHandle[ LSM6DSL_X_0 ].isCombo       = 1;
  ACCELERO_SensorHandle[ LSM6DSL_X_0 ].pData         = (void *)&ACCELERO_Data[ LSM6DSL_X_0 ];
  ACCELERO_SensorHandle[ LSM6DSL_X_0 ].pVTable       = (void *)&LSM6DSL_X_Drv;
  ACCELERO_SensorHandle[ LSM6DSL_X_0 ].pExtVTable    = (void *)&LSM6DSL_X_ExtDrv;

  ACCELERO_Data[ LSM6DSL_X_0 ].pComponentData = (void *)&LSM6DSL_X_0_Data;
  ACCELERO_Data[ LSM6DSL_X_0 ].pExtData       = 0;

  *handle = (void *)&ACCELERO_SensorHandle[ LSM6DSL_X_0 ];

  driver = (ACCELERO_Drv_t *)((DrvContextTypeDef *)(*handle))->pVTable;

  if (driver->Init == NULL)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if (driver->Init((DrvContextTypeDef *)(*handle)) == COMPONENT_ERROR)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  /* Configure interrupt lines common for all sensors in DIL24 socket */
  DIL24_Sensor_IO_ITConfig();

  return COMPONENT_OK;
}



/**
 * @brief Initialize an LSM303AGR accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef BSP_LSM303AGR_ACCELERO_Init(void **handle)
{
  ACCELERO_Drv_t *driver = NULL;

  if (Sensor_IO_Init() == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].who_am_i      = LSM303AGR_ACC_WHO_AM_I;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].ifType        = 0; /* I2C interface */
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].address       = LSM303AGR_ACC_I2C_ADDRESS;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].instance      = LSM303AGR_X_0;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].isInitialized = 0;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].isEnabled     = 0;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].isCombo       = 1;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].pData         = (void *)&ACCELERO_Data[ LSM303AGR_X_0 ];
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].pVTable       = (void *)&LSM303AGR_X_Drv;
  ACCELERO_SensorHandle[ LSM303AGR_X_0 ].pExtVTable    = (void *)&LSM303AGR_X_ExtDrv;

  LSM303AGR_X_0_Data.comboData = &LSM303AGR_Combo_Data[0];
  ACCELERO_Data[ LSM303AGR_X_0 ].pComponentData = (void *)&LSM303AGR_X_0_Data;
  ACCELERO_Data[ LSM303AGR_X_0 ].pExtData       = 0;

  *handle = (void *)&ACCELERO_SensorHandle[ LSM303AGR_X_0 ];

  driver = (ACCELERO_Drv_t *)((DrvContextTypeDef *)(*handle))->pVTable;

  if (driver->Init == NULL)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if (driver->Init((DrvContextTypeDef *)(*handle)) == COMPONENT_ERROR)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  /* Configure interrupt lines common for all sensors in DIL24 socket */
  DIL24_Sensor_IO_ITConfig();

  return COMPONENT_OK;
}



/**
 * @brief Initialize an LIS2DH12 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef BSP_LIS2DH12_ACCELERO_Init(void **handle)
{
  ACCELERO_Drv_t *driver = NULL;

  if (Sensor_IO_Init() == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  ACCELERO_SensorHandle[ LIS2DH12_0 ].who_am_i      = LIS2DH12_WHO_AM_I;
  ACCELERO_SensorHandle[ LIS2DH12_0 ].ifType        = 0; /* I2C interface */
  ACCELERO_SensorHandle[ LIS2DH12_0 ].address       = LIS2DH12_I2C_ADDRESS_LOW;
  ACCELERO_SensorHandle[ LIS2DH12_0 ].instance      = LIS2DH12_0;
  ACCELERO_SensorHandle[ LIS2DH12_0 ].isInitialized = 0;
  ACCELERO_SensorHandle[ LIS2DH12_0 ].isEnabled     = 0;
  ACCELERO_SensorHandle[ LIS2DH12_0 ].isCombo       = 0;
  ACCELERO_SensorHandle[ LIS2DH12_0 ].pData         = (void *)&ACCELERO_Data[ LIS2DH12_0 ];
  ACCELERO_SensorHandle[ LIS2DH12_0 ].pVTable       = (void *)&LIS2DH12_Drv;
  ACCELERO_SensorHandle[ LIS2DH12_0 ].pExtVTable    = (void *)&LIS2DH12_ExtDrv;

  ACCELERO_Data[ LIS2DH12_0 ].pComponentData = (void *)&LIS2DH12_0_Data;
  ACCELERO_Data[ LIS2DH12_0 ].pExtData       = 0;

  *handle = (void *)&ACCELERO_SensorHandle[ LIS2DH12_0 ];

  driver = (ACCELERO_Drv_t *)((DrvContextTypeDef *)(*handle))->pVTable;

  if (driver->Init == NULL)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if (driver->Init((DrvContextTypeDef *)(*handle)) == COMPONENT_ERROR)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  /* Configure interrupt lines common for all sensors in DIL24 socket */
  DIL24_Sensor_IO_ITConfig();

  return COMPONENT_OK;
}



/**
 * @brief Initialize an H3LIS331DL accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef BSP_H3LIS331DL_ACCELERO_Init(void **handle)
{
  ACCELERO_Drv_t *driver = NULL;

  if (Sensor_IO_Init() == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  ACCELERO_SensorHandle[ H3LIS331DL_0 ].who_am_i      = H3LIS331DL_WHO_AM_I;
  ACCELERO_SensorHandle[ H3LIS331DL_0 ].ifType        = 0; /* I2C interface */
  ACCELERO_SensorHandle[ H3LIS331DL_0 ].address       = H3LIS331DL_I2C_ADDRESS_LOW;
  ACCELERO_SensorHandle[ H3LIS331DL_0 ].instance      = H3LIS331DL_0;
  ACCELERO_SensorHandle[ H3LIS331DL_0 ].isInitialized = 0;
  ACCELERO_SensorHandle[ H3LIS331DL_0 ].isEnabled     = 0;
  ACCELERO_SensorHandle[ H3LIS331DL_0 ].isCombo       = 0;
  ACCELERO_SensorHandle[ H3LIS331DL_0 ].pData         = (void *)&ACCELERO_Data[ H3LIS331DL_0 ];
  ACCELERO_SensorHandle[ H3LIS331DL_0 ].pVTable       = (void *)&H3LIS331DL_Drv;
  ACCELERO_SensorHandle[ H3LIS331DL_0 ].pExtVTable    = 0;

  ACCELERO_Data[ H3LIS331DL_0 ].pComponentData = (void *)&H3LIS331DL_0_Data;
  ACCELERO_Data[ H3LIS331DL_0 ].pExtData       = 0;

  *handle = (void *)&ACCELERO_SensorHandle[ H3LIS331DL_0 ];

  driver = (ACCELERO_Drv_t *)((DrvContextTypeDef *)(*handle))->pVTable;

  if (driver->Init == NULL)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if (driver->Init((DrvContextTypeDef *)(*handle)) == COMPONENT_ERROR)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  /* Configure interrupt lines common for all sensors in DIL24 socket */
  DIL24_Sensor_IO_ITConfig();

  return COMPONENT_OK;
}



/**
 * @brief Initialize an LIS2DW12 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef BSP_LIS2DW12_ACCELERO_Init(void **handle)
{
  ACCELERO_Drv_t *driver = NULL;

  if (Sensor_IO_Init() == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  ACCELERO_SensorHandle[ LIS2DW12_0 ].who_am_i      = LIS2DW12_ACC_WHO_AM_I;
  ACCELERO_SensorHandle[ LIS2DW12_0 ].ifType        = 0; /* I2C interface */
  ACCELERO_SensorHandle[ LIS2DW12_0 ].address       = LIS2DW12_I2C_ADDRESS_LOW;
  ACCELERO_SensorHandle[ LIS2DW12_0 ].instance      = LIS2DW12_0;
  ACCELERO_SensorHandle[ LIS2DW12_0 ].isInitialized = 0;
  ACCELERO_SensorHandle[ LIS2DW12_0 ].isEnabled     = 0;
  ACCELERO_SensorHandle[ LIS2DW12_0 ].isCombo       = 0;
  ACCELERO_SensorHandle[ LIS2DW12_0 ].pData         = (void *)&ACCELERO_Data[ LIS2DW12_0 ];
  ACCELERO_SensorHandle[ LIS2DW12_0 ].pVTable       = (void *)&LIS2DW12_Drv;
  ACCELERO_SensorHandle[ LIS2DW12_0 ].pExtVTable    = 0;

  ACCELERO_Data[ LIS2DW12_0 ].pComponentData = (void *)&LIS2DW12_0_Data;
  ACCELERO_Data[ LIS2DW12_0 ].pExtData       = 0;

  *handle = (void *)&ACCELERO_SensorHandle[ LIS2DW12_0 ];

  driver = (ACCELERO_Drv_t *)((DrvContextTypeDef *)(*handle))->pVTable;

  if (driver->Init == NULL)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if (driver->Init((DrvContextTypeDef *)(*handle)) == COMPONENT_ERROR)
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  /* Configure interrupt lines common for all sensors in DIL24 socket */
  DIL24_Sensor_IO_ITConfig();

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
