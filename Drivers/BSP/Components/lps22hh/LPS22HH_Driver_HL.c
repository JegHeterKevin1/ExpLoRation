/**
 *******************************************************************************
 * @file    LPS22HH_Driver_HL.c
 * @author  MEMS Application Team
 * @brief   This file provides a set of high-level functions needed to manage
            the LPS22HH sensor
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
#include "LPS22HH_Driver_HL.h"
#include <math.h>

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup LPS22HH LPS22HH
 * @{
 */

/* Link function for sensor peripheral */
extern uint8_t Sensor_IO_Write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
extern uint8_t Sensor_IO_Read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);

/** @addtogroup LPS22HH_Callable_Private_Function_Prototypes Callable private function prototypes
 * @{
 */

static DrvStatusTypeDef LPS22HH_P_Init(DrvContextTypeDef *handle);
static DrvStatusTypeDef LPS22HH_P_DeInit(DrvContextTypeDef *handle);
static DrvStatusTypeDef LPS22HH_P_Sensor_Enable(DrvContextTypeDef *handle);
static DrvStatusTypeDef LPS22HH_P_Sensor_Disable(DrvContextTypeDef *handle);
static DrvStatusTypeDef LPS22HH_P_Get_WhoAmI(DrvContextTypeDef *handle, uint8_t *who_am_i);
static DrvStatusTypeDef LPS22HH_P_Check_WhoAmI(DrvContextTypeDef *handle);
static DrvStatusTypeDef LPS22HH_P_Get_Press(DrvContextTypeDef *handle, float *pressure);
static DrvStatusTypeDef LPS22HH_P_Get_ODR(DrvContextTypeDef *handle, float *odr);
static DrvStatusTypeDef LPS22HH_P_Set_ODR(DrvContextTypeDef *handle, SensorOdr_t odr);
static DrvStatusTypeDef LPS22HH_P_Set_ODR_Value(DrvContextTypeDef *handle, float odr);
static DrvStatusTypeDef LPS22HH_P_Read_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t *data);
static DrvStatusTypeDef LPS22HH_P_Write_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t data);
static DrvStatusTypeDef LPS22HH_P_Get_DRDY_Status(DrvContextTypeDef *handle, uint8_t *status);

static DrvStatusTypeDef LPS22HH_T_Init(DrvContextTypeDef *handle);
static DrvStatusTypeDef LPS22HH_T_DeInit(DrvContextTypeDef *handle);
static DrvStatusTypeDef LPS22HH_T_Sensor_Enable(DrvContextTypeDef *handle);
static DrvStatusTypeDef LPS22HH_T_Sensor_Disable(DrvContextTypeDef *handle);
static DrvStatusTypeDef LPS22HH_T_Get_WhoAmI(DrvContextTypeDef *handle, uint8_t *who_am_i);
static DrvStatusTypeDef LPS22HH_T_Check_WhoAmI(DrvContextTypeDef *handle);
static DrvStatusTypeDef LPS22HH_T_Get_Temp(DrvContextTypeDef *handle, float *temperature);
static DrvStatusTypeDef LPS22HH_T_Get_ODR(DrvContextTypeDef *handle, float *odr);
static DrvStatusTypeDef LPS22HH_T_Set_ODR(DrvContextTypeDef *handle, SensorOdr_t odr);
static DrvStatusTypeDef LPS22HH_T_Set_ODR_Value(DrvContextTypeDef *handle, float odr);
static DrvStatusTypeDef LPS22HH_T_Read_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t *data);
static DrvStatusTypeDef LPS22HH_T_Write_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t data);
static DrvStatusTypeDef LPS22HH_T_Get_DRDY_Status(DrvContextTypeDef *handle, uint8_t *status);

/**
 * @}
 */

/** @addtogroup LPS22HH_Private_Function_Prototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef LPS22HH_Initialize(DrvContextTypeDef *handle, LPS22HH_Combo_Data_t *combo);
static DrvStatusTypeDef LPS22HH_Get_WhoAmI(DrvContextTypeDef *handle, uint8_t *who_am_i);
static DrvStatusTypeDef LPS22HH_Check_WhoAmI(DrvContextTypeDef *handle);
static DrvStatusTypeDef LPS22HH_Get_Press(DrvContextTypeDef *handle, float *pressure);
static DrvStatusTypeDef LPS22HH_Get_Temp(DrvContextTypeDef *handle, float *temperature);
static DrvStatusTypeDef LPS22HH_Get_ODR(DrvContextTypeDef *handle, float *odr);
static DrvStatusTypeDef LPS22HH_Set_ODR_When_Enabled(DrvContextTypeDef *handle, SensorOdr_t odr,
                                                     LPS22HH_Combo_Data_t *combo);
static DrvStatusTypeDef LPS22HH_Set_ODR_When_Disabled(DrvContextTypeDef *handle, SensorOdr_t odr,
                                                      LPS22HH_Combo_Data_t *combo);
static DrvStatusTypeDef LPS22HH_Set_ODR_Value_When_Enabled(DrvContextTypeDef *handle, float odr,
                                                           LPS22HH_Combo_Data_t *combo);
static DrvStatusTypeDef LPS22HH_Set_ODR_Value_When_Disabled(DrvContextTypeDef *handle, float odr,
                                                            LPS22HH_Combo_Data_t *combo);
static DrvStatusTypeDef LPS22HH_Power_Down(DrvContextTypeDef *handle);
static DrvStatusTypeDef LPS22HH_Read_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t *data);
static DrvStatusTypeDef LPS22HH_Write_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t data);

/**
 * @}
 */

/** @addtogroup LPS22HH_Public_Variables Public variables
 * @{
 */

/**
 * @brief LPS22HH pressure driver structure
 */
PRESSURE_Drv_t LPS22HH_P_Drv =
{
  LPS22HH_P_Init,
  LPS22HH_P_DeInit,
  LPS22HH_P_Sensor_Enable,
  LPS22HH_P_Sensor_Disable,
  LPS22HH_P_Get_WhoAmI,
  LPS22HH_P_Check_WhoAmI,
  LPS22HH_P_Get_Press,
  LPS22HH_P_Get_ODR,
  LPS22HH_P_Set_ODR,
  LPS22HH_P_Set_ODR_Value,
  LPS22HH_P_Read_Reg,
  LPS22HH_P_Write_Reg,
  LPS22HH_P_Get_DRDY_Status
};

/**
 * @brief LPS22HH temperature driver structure
 */
TEMPERATURE_Drv_t LPS22HH_T_Drv =
{
  LPS22HH_T_Init,
  LPS22HH_T_DeInit,
  LPS22HH_T_Sensor_Enable,
  LPS22HH_T_Sensor_Disable,
  LPS22HH_T_Get_WhoAmI,
  LPS22HH_T_Check_WhoAmI,
  LPS22HH_T_Get_Temp,
  LPS22HH_T_Get_ODR,
  LPS22HH_T_Set_ODR,
  LPS22HH_T_Set_ODR_Value,
  LPS22HH_T_Read_Reg,
  LPS22HH_T_Write_Reg,
  LPS22HH_T_Get_DRDY_Status
};

/**
 * @brief LPS22HH combo data structure definition
 */
LPS22HH_Combo_Data_t LPS22HH_Combo_Data[LPS22HH_SENSORS_MAX_NUM];

/**
 * @}
 */

/** @addtogroup LPS22HH_Callable_Private_Functions Callable private functions
 * @{
 */

/**
 * @brief Initialize the LPS22HH pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_P_Init(DrvContextTypeDef *handle)
{
  PRESSURE_Data_t *pData = (PRESSURE_Data_t *)handle->pData;
  LPS22HH_P_Data_t *pComponentData = (LPS22HH_P_Data_t *)pData->pComponentData;
  LPS22HH_Combo_Data_t *comboData = pComponentData->comboData;

  /* Check if the LPS22HH temperature sensor is already initialized. */
  /* If yes, skip the initialize function, if not call initialize function */
  if (comboData->isTempInitialized == 0)
  {
    if (LPS22HH_Initialize(handle, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  comboData->isPressInitialized = 1;

  handle->isInitialized = 1;

  return COMPONENT_OK;
}

/**
 * @brief Deinitialize the LPS22HH pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_P_DeInit(DrvContextTypeDef *handle)
{
  PRESSURE_Data_t *pData = (PRESSURE_Data_t *)handle->pData;
  LPS22HH_P_Data_t *pComponentData = (LPS22HH_P_Data_t *)pData->pComponentData;
  LPS22HH_Combo_Data_t *comboData = pComponentData->comboData;

  /* Check if the LPS22HH temperature sensor is already initialized. */
  /* If yes, skip the deinitialize function, if not call deinitialize function */
  if (comboData->isTempInitialized == 0)
  {
    if (LPS22HH_P_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  comboData->isPressInitialized = 0;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}

/**
 * @brief Enable the LPS22HH pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_P_Sensor_Enable(DrvContextTypeDef *handle)
{
  PRESSURE_Data_t *pData = (PRESSURE_Data_t *)handle->pData;
  LPS22HH_P_Data_t *pComponentData = (LPS22HH_P_Data_t *)pData->pComponentData;
  LPS22HH_Combo_Data_t *comboData = pComponentData->comboData;

  /* Check if the component is already enabled */
  if (handle->isEnabled == 1)
  {
    return COMPONENT_OK;
  }

  if (LPS22HH_Set_ODR_Value_When_Enabled(handle, comboData->Last_ODR, comboData) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  comboData->isPressEnabled = 1;

  handle->isEnabled = 1;

  return COMPONENT_OK;
}

/**
 * @brief Disable the LPS22HH pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_P_Sensor_Disable(DrvContextTypeDef *handle)
{
  PRESSURE_Data_t *pData = (PRESSURE_Data_t *)handle->pData;
  LPS22HH_P_Data_t *pComponentData = (LPS22HH_P_Data_t *)pData->pComponentData;
  LPS22HH_Combo_Data_t *comboData = pComponentData->comboData;

  /* Check if the component is already disabled */
  if (handle->isEnabled == 0)
  {
    return COMPONENT_OK;
  }

  /* Check if the LPS22HH temperature sensor is still enable. */
  /* If yes, skip the disable function, if not call disable function */
  if (comboData->isTempEnabled == 0)
  {
    /* Power down the device */
    if (LPS22HH_Power_Down(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  comboData->isPressEnabled = 0;

  handle->isEnabled = 0;

  return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the LPS22HH pressure sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_P_Get_WhoAmI(DrvContextTypeDef *handle, uint8_t *who_am_i)
{

  return LPS22HH_Get_WhoAmI(handle, who_am_i);
}

/**
 * @brief Check the WHO_AM_I ID of the LPS22HH pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_P_Check_WhoAmI(DrvContextTypeDef *handle)
{

  return LPS22HH_Check_WhoAmI(handle);
}

/**
 * @brief Get the pressure value of the LPS22HH pressure sensor
 * @param handle the device handle
 * @param pressure pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_P_Get_Press(DrvContextTypeDef *handle, float *pressure)
{

  return LPS22HH_Get_Press(handle, pressure);
}

/**
 * @brief Get the LPS22HH pressure sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_P_Get_ODR(DrvContextTypeDef *handle, float *odr)
{

  return LPS22HH_Get_ODR(handle, odr);
}

/**
 * @brief Set the LPS22HH pressure sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_P_Set_ODR(DrvContextTypeDef *handle, SensorOdr_t odr)
{
  PRESSURE_Data_t *pData = (PRESSURE_Data_t *)handle->pData;
  LPS22HH_P_Data_t *pComponentData = (LPS22HH_P_Data_t *)pData->pComponentData;
  LPS22HH_Combo_Data_t *comboData = pComponentData->comboData;

  if (handle->isEnabled == 1)
  {
    if (LPS22HH_Set_ODR_When_Enabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if (LPS22HH_Set_ODR_When_Disabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LPS22HH pressure sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_P_Set_ODR_Value(DrvContextTypeDef *handle, float odr)
{
  PRESSURE_Data_t *pData = (PRESSURE_Data_t *)handle->pData;
  LPS22HH_P_Data_t *pComponentData = (LPS22HH_P_Data_t *)pData->pComponentData;
  LPS22HH_Combo_Data_t *comboData = pComponentData->comboData;

  if (handle->isEnabled == 1)
  {
    if (LPS22HH_Set_ODR_Value_When_Enabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if (LPS22HH_Set_ODR_Value_When_Disabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
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
static DrvStatusTypeDef LPS22HH_P_Read_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t *data)
{
  if (LPS22HH_Read_Reg(handle, reg, data) == COMPONENT_ERROR)
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
static DrvStatusTypeDef LPS22HH_P_Write_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t data)
{
  if (LPS22HH_Write_Reg(handle, reg, data) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get pressure data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_P_Get_DRDY_Status(DrvContextTypeDef *handle, uint8_t *status)
{
  lps22hh_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  if (lps22hh_press_flag_data_ready_get(&ctx, status) == 1)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Initialize the LPS22HH temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_T_Init(DrvContextTypeDef *handle)
{
  TEMPERATURE_Data_t *pData = (TEMPERATURE_Data_t *)handle->pData;
  LPS22HH_T_Data_t *pComponentData = (LPS22HH_T_Data_t *)pData->pComponentData;
  LPS22HH_Combo_Data_t *comboData = pComponentData->comboData;

  /* Check if the LPS22HH pressure sensor is already initialized. */
  /* If yes, skip the initialize function, if not call initialize function */
  if (comboData->isPressInitialized == 0)
  {
    if (LPS22HH_Initialize(handle, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  comboData->isTempInitialized = 1;

  handle->isInitialized = 1;

  return COMPONENT_OK;
}

/**
 * @brief Denitialize the LPS22HH temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_T_DeInit(DrvContextTypeDef *handle)
{
  TEMPERATURE_Data_t *pData = (TEMPERATURE_Data_t *)handle->pData;
  LPS22HH_T_Data_t *pComponentData = (LPS22HH_T_Data_t *)pData->pComponentData;
  LPS22HH_Combo_Data_t *comboData = pComponentData->comboData;

  /* Check if the LPS22HH pressure sensor is already initialized. */
  /* If yes, skip the deinitialize function, if not call deinitialize function */
  if (comboData->isPressInitialized == 0)
  {
    if (LPS22HH_T_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  comboData->isTempInitialized = 0;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}

/**
 * @brief Enable the LPS22HH temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_T_Sensor_Enable(DrvContextTypeDef *handle)
{
  TEMPERATURE_Data_t *pData = (TEMPERATURE_Data_t *)handle->pData;
  LPS22HH_T_Data_t *pComponentData = (LPS22HH_T_Data_t *)pData->pComponentData;
  LPS22HH_Combo_Data_t *comboData = pComponentData->comboData;

  /* Check if the component is already enabled */
  if (handle->isEnabled == 1)
  {
    return COMPONENT_OK;
  }

  if (LPS22HH_Set_ODR_Value_When_Enabled(handle, comboData->Last_ODR, comboData) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  comboData->isTempEnabled = 1;

  handle->isEnabled = 1;

  return COMPONENT_OK;
}

/**
 * @brief Disable the LPS22HH temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_T_Sensor_Disable(DrvContextTypeDef *handle)
{
  TEMPERATURE_Data_t *pData = (TEMPERATURE_Data_t *)handle->pData;
  LPS22HH_T_Data_t *pComponentData = (LPS22HH_T_Data_t *)pData->pComponentData;
  LPS22HH_Combo_Data_t *comboData = pComponentData->comboData;

  /* Check if the component is already disabled */
  if (handle->isEnabled == 0)
  {
    return COMPONENT_OK;
  }

  /* Check if the LPS22HH pressure sensor is still enable. */
  /* If yes, skip the disable function, if not call disable function */
  if (comboData->isPressEnabled == 0)
  {
    /* Power down the device */
    if (LPS22HH_Power_Down(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  comboData->isTempEnabled = 0;

  handle->isEnabled = 0;

  return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the LPS22HH temperature sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_T_Get_WhoAmI(DrvContextTypeDef *handle, uint8_t *who_am_i)
{

  return LPS22HH_Get_WhoAmI(handle, who_am_i);
}

/**
 * @brief Check the WHO_AM_I ID of the LPS22HH temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_T_Check_WhoAmI(DrvContextTypeDef *handle)
{

  return LPS22HH_Check_WhoAmI(handle);
}

/**
 * @brief Get the temperature value of the LPS22HH temperature sensor
 * @param handle the device handle
 * @param temperature pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_T_Get_Temp(DrvContextTypeDef *handle, float *temperature)
{

  return LPS22HH_Get_Temp(handle, temperature);
}

/**
 * @brief Get the LPS22HH temperature sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_T_Get_ODR(DrvContextTypeDef *handle, float *odr)
{

  return LPS22HH_Get_ODR(handle, odr);
}

/**
 * @brief Set the LPS22HH temperature sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_T_Set_ODR(DrvContextTypeDef *handle, SensorOdr_t odr)
{
  TEMPERATURE_Data_t *pData = (TEMPERATURE_Data_t *)handle->pData;
  LPS22HH_T_Data_t *pComponentData = (LPS22HH_T_Data_t *)pData->pComponentData;
  LPS22HH_Combo_Data_t *comboData = pComponentData->comboData;

  if (handle->isEnabled == 1)
  {
    if (LPS22HH_Set_ODR_When_Enabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if (LPS22HH_Set_ODR_When_Disabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LPS22HH temperature sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_T_Set_ODR_Value(DrvContextTypeDef *handle, float odr)
{
  TEMPERATURE_Data_t *pData = (TEMPERATURE_Data_t *)handle->pData;
  LPS22HH_T_Data_t *pComponentData = (LPS22HH_T_Data_t *)pData->pComponentData;
  LPS22HH_Combo_Data_t *comboData = pComponentData->comboData;

  if (handle->isEnabled == 1)
  {
    if (LPS22HH_Set_ODR_Value_When_Enabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if (LPS22HH_Set_ODR_Value_When_Disabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
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
static DrvStatusTypeDef LPS22HH_T_Read_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t *data)
{

  if (LPS22HH_Read_Reg(handle, reg, data) == COMPONENT_ERROR)
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
static DrvStatusTypeDef LPS22HH_T_Write_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t data)
{

  if (LPS22HH_Write_Reg(handle, reg, data) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get temperature data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_T_Get_DRDY_Status(DrvContextTypeDef *handle, uint8_t *status)
{
  lps22hh_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  if (lps22hh_temp_flag_data_ready_get(&ctx, status) == 1)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @}
 */

/** @addtogroup LPS22HH_Private_Functions Private functions
 * @{
 */

/**
 * @brief Initialize the LPS22HH sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_Initialize(DrvContextTypeDef *handle, LPS22HH_Combo_Data_t *combo)
{
  lps22hh_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  /* Disable MIPI I3C(SM) interface */
  if (lps22hh_i3c_interface_set(&ctx, LPS22HH_I3C_DISABLE) == 1)
  {
    return COMPONENT_ERROR;
  }

  if (LPS22HH_Check_WhoAmI(handle) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  combo->Last_ODR = 25.0f;

  /* Power down the device, set Low Noise Enable (bit 5), clear One Shot (bit 4) */
  if (lps22hh_data_rate_set(&ctx, (lps22hh_odr_t)(LPS22HH_POWER_DOWN | 0x10)) == 1)
  {
    return COMPONENT_ERROR;
  }

  /* Disable low-pass filter on LPS22HH pressure data */
  if (lps22hh_lp_bandwidth_set(&ctx, LPS22HH_LPF_ODR_DIV_2) == 1)
  {
    return COMPONENT_ERROR;
  }

  /* Set block data update mode */
  if (lps22hh_block_data_update_set(&ctx, PROPERTY_ENABLE) == 1)
  {
    return COMPONENT_ERROR;
  }

  /* Set automatic increment for multi-byte read/write */
  if (lps22hh_auto_increment_set(&ctx, PROPERTY_ENABLE) == 1)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the LPS22HH sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_Get_WhoAmI(DrvContextTypeDef *handle, uint8_t *who_am_i)
{
  lps22hh_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  /* Read WHO AM I register */
  if (lps22hh_device_id_get(&ctx, who_am_i) == 1)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Check the WHO_AM_I ID of the LPS22HH sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_Check_WhoAmI(DrvContextTypeDef *handle)
{
  uint8_t who_am_i = 0x00;

  if (LPS22HH_Get_WhoAmI(handle, &who_am_i) == COMPONENT_ERROR)
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
 * @brief Get the pressure value (hPa) of the LPS22HH sensor
 * @param handle the device handle
 * @param pressure pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_Get_Press(DrvContextTypeDef *handle, float *pressure)
{
  lps22hh_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  int32_t int32data = 0;

  /* Read data from LPS22HH */
  if (lps22hh_pressure_raw_get(&ctx, (uint8_t *)&int32data) == 1)
  {
    return COMPONENT_ERROR;
  }

  *pressure = (float)int32data / 4096.0f;

  return COMPONENT_OK;
}

/**
 * @brief Get the temperature value (°C) of the LPS22HH sensor
 * @param handle the device handle
 * @param temperature pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_Get_Temp(DrvContextTypeDef *handle, float *temperature)
{
  lps22hh_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  int16_t int16data = 0;

  /* Read data from LPS22HH */
  if (lps22hh_temperature_raw_get(&ctx, (uint8_t *)&int16data) == 1)
  {
    return COMPONENT_ERROR;
  }

  *temperature = (float)int16data / 100.0f;

  return COMPONENT_OK;
}

/**
 * @brief Get the LPS22HH sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_Get_ODR(DrvContextTypeDef *handle, float *odr)
{
  lps22hh_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  lps22hh_odr_t odr_low_level;

  if (lps22hh_data_rate_get(&ctx, &odr_low_level) == 1)
  {
    return COMPONENT_ERROR;
  }

  switch (odr_low_level)
  {
    case LPS22HH_POWER_DOWN:
    case LPS22HH_ONE_SHOOT:
      *odr = 0.0f;
      break;
    case LPS22HH_1_Hz:
    case LPS22HH_1_Hz_LOW_NOISE:
      *odr = 1.0f;
      break;
    case LPS22HH_10_Hz:
    case LPS22HH_10_Hz_LOW_NOISE:
      *odr = 10.0f;
      break;
    case LPS22HH_25_Hz:
    case LPS22HH_25_Hz_LOW_NOISE:
      *odr = 25.0f;
      break;
    case LPS22HH_50_Hz:
    case LPS22HH_50_Hz_LOW_NOISE:
      *odr = 50.0f;
      break;
    case LPS22HH_75_Hz:
    case LPS22HH_75_Hz_LOW_NOISE:
      *odr = 75.0f;
      break;
    case LPS22HH_100_Hz:
      *odr = 100.0f;
      break;
    case LPS22HH_200_Hz:
      *odr = 200.0f;
      break;
    default:
      *odr = -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LPS22HH sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @param combo the pointer to the combo shared structure
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_Set_ODR_When_Enabled(DrvContextTypeDef *handle, SensorOdr_t odr,
                                                     LPS22HH_Combo_Data_t *combo)
{
  lps22hh_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  lps22hh_odr_t odr_low_level;
  lps22hh_odr_t new_odr;

  if (lps22hh_data_rate_get(&ctx, &odr_low_level) == 1)
  {
    return COMPONENT_ERROR;
  }

  switch (odr)
  {
    case ODR_LOW:
      new_odr = LPS22HH_1_Hz;
      break;
    case ODR_MID_LOW:
      new_odr = LPS22HH_10_Hz;
      break;
    case ODR_MID:
      new_odr = LPS22HH_25_Hz;
      break;
    case ODR_MID_HIGH:
      new_odr = LPS22HH_50_Hz;
      break;
    case ODR_HIGH:
      new_odr = LPS22HH_75_Hz;
      break;
    default:
      return COMPONENT_ERROR;
  }

  /* Keep Low Noise Enable (bit 5) and One Shot (bit 4) same */
  new_odr |= (odr_low_level & 0x18);

  if (lps22hh_data_rate_set(&ctx, new_odr) == 1)
  {
    return COMPONENT_ERROR;
  }

  if (LPS22HH_Get_ODR(handle, &combo->Last_ODR) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LPS22HH sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @param combo the pointer to the combo shared structure
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_Set_ODR_When_Disabled(DrvContextTypeDef *handle, SensorOdr_t odr,
                                                      LPS22HH_Combo_Data_t *combo)
{
  switch (odr)
  {
    case ODR_LOW:
      combo->Last_ODR =  1.0f;
      break;
    case ODR_MID_LOW:
      combo->Last_ODR = 10.0f;
      break;
    case ODR_MID:
      combo->Last_ODR = 25.0f;
      break;
    case ODR_MID_HIGH:
      combo->Last_ODR = 50.0f;
      break;
    case ODR_HIGH:
      combo->Last_ODR = 75.0f;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LPS22HH sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @param combo the pointer to the combo shared structure
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_Set_ODR_Value_When_Enabled(DrvContextTypeDef *handle, float odr,
                                                           LPS22HH_Combo_Data_t *combo)
{
  lps22hh_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  lps22hh_odr_t odr_low_level;
  lps22hh_odr_t new_odr;

  if (lps22hh_data_rate_get(&ctx, &odr_low_level) == 1)
  {
    return COMPONENT_ERROR;
  }

  new_odr = (odr <=   1.0f) ? LPS22HH_1_Hz
            : (odr <=  10.0f) ? LPS22HH_10_Hz
            : (odr <=  25.0f) ? LPS22HH_25_Hz
            : (odr <=  50.0f) ? LPS22HH_50_Hz
            : (odr <=  75.0f) ? LPS22HH_75_Hz
            : (odr <= 100.0f) ? LPS22HH_100_Hz
            :                   LPS22HH_200_Hz;

  /* Keep Low Noise Enable (bit 5) and One Shot (bit 4) same */
  new_odr |= (odr_low_level & 0x18);

  if (lps22hh_data_rate_set(&ctx, new_odr) == 1)
  {
    return COMPONENT_ERROR;
  }

  if (LPS22HH_Get_ODR(handle, &combo->Last_ODR) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LPS22HH sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @param combo the pointer to the combo shared structure
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_Set_ODR_Value_When_Disabled(DrvContextTypeDef *handle, float odr,
                                                            LPS22HH_Combo_Data_t *combo)
{
  combo->Last_ODR = (odr <=   1.0f) ?   1.0f
                    : (odr <=  10.0f) ?  10.0f
                    : (odr <=  25.0f) ?  25.0f
                    : (odr <=  50.0f) ?  50.0f
                    : (odr <=  75.0f) ?  75.0f
                    : (odr <= 100.0f) ? 100.0f
                    :                   200.0f;

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
static DrvStatusTypeDef LPS22HH_Read_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t *data)
{
  lps22hh_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  if (lps22hh_read_reg(&ctx, reg, data, 1) == 1)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Power down the LPS22HH sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HH_Power_Down(DrvContextTypeDef *handle)
{
  lps22hh_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};
  lps22hh_odr_t odr_low_level;
  lps22hh_odr_t new_odr;

  if (lps22hh_data_rate_get(&ctx, &odr_low_level) == 1)
  {
    return COMPONENT_ERROR;
  }

  /* Keep Low Noise Enable (bit 5) and One Shot (bit 4) same */
  new_odr = (lps22hh_odr_t)(LPS22HH_POWER_DOWN | (odr_low_level & 0x18));

  if (lps22hh_data_rate_set(&ctx, new_odr) == 1)
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
static DrvStatusTypeDef LPS22HH_Write_Reg(DrvContextTypeDef *handle, uint8_t reg, uint8_t data)
{
  lps22hh_ctx_t ctx = {.write_reg = Sensor_IO_Write, .read_reg = Sensor_IO_Read, .handle = (void *)handle};

  if (lps22hh_write_reg(&ctx, reg, &data, 1) == 1)
  {
    return COMPONENT_ERROR;
  }

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
