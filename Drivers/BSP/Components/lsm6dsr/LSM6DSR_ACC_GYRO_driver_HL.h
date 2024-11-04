/**
 ******************************************************************************
 * @file    LSM6DSR_ACC_GYRO_driver_HL.h
 * @author  MEMS Application Team
 * @brief   This file contains definitions for the LSM6DSR_ACC_GYRO_driver_HL.c firmware driver
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LSM6DSR_ACC_GYRO_DRIVER_HL_H
#define __LSM6DSR_ACC_GYRO_DRIVER_HL_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/

#include "accelerometer.h"
#include "gyroscope.h"

/* Include accelero sensor component drivers. */
#include "LSM6DSR_ACC_GYRO_driver.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup LSM6DSR LSM6DSR
 * @{
 */

/** @addtogroup LSM6DSR_Public_Constants Public constants
 * @{
 */

#define LSM6DSR_SENSORS_MAX_NUM  1     /**< LSM6DSR max number of instances */

/** @addtogroup LSM6DSR_ACC_SENSITIVITY Accelero sensitivity values based on selected full scale
 * @{
 */

#define LSM6DSR_ACC_SENSITIVITY_FOR_FS_2G   0.061  /**< Sensitivity value for 2 g full scale [mg/LSB] */
#define LSM6DSR_ACC_SENSITIVITY_FOR_FS_4G   0.122  /**< Sensitivity value for 4 g full scale [mg/LSB] */
#define LSM6DSR_ACC_SENSITIVITY_FOR_FS_8G   0.244  /**< Sensitivity value for 8 g full scale [mg/LSB] */
#define LSM6DSR_ACC_SENSITIVITY_FOR_FS_16G  0.488  /**< Sensitivity value for 16 g full scale [mg/LSB] */

/**
 * @}
 */

/** @addtogroup LSM6DSR_GYRO_SENSITIVITY Gyro sensitivity values based on selected full scale
 * @{
 */

#define LSM6DSR_GYRO_SENSITIVITY_FOR_FS_125DPS   04.375  /**< Sensitivity value for 125 dps full scale [mdps/LSB] */
#define LSM6DSR_GYRO_SENSITIVITY_FOR_FS_250DPS   08.750  /**< Sensitivity value for 250 dps full scale [mdps/LSB] */
#define LSM6DSR_GYRO_SENSITIVITY_FOR_FS_500DPS   17.500  /**< Sensitivity value for 500 dps full scale [mdps/LSB] */
#define LSM6DSR_GYRO_SENSITIVITY_FOR_FS_1000DPS  35.000  /**< Sensitivity value for 1000 dps full scale [mdps/LSB] */
#define LSM6DSR_GYRO_SENSITIVITY_FOR_FS_2000DPS  70.000  /**< Sensitivity value for 2000 dps full scale [mdps/LSB] */

/**
 * @}
 */

/**
 * @}
 */

/** @addtogroup LSM6DSR_Public_Types LSM6DSR Public Types
 * @{
 */

/**
 * @brief LSM6DSR combo specific data internal structure definition
 */
typedef struct
{
  uint8_t isAccInitialized;
  uint8_t isGyroInitialized;
} LSM6DSR_Combo_Data_t;

/**
 * @brief LSM6DSR accelero specific data internal structure definition
 */
typedef struct
{
  LSM6DSR_Combo_Data_t *comboData; /* Combo data to manage in software SPI 3-Wire initialization of the combo sensors */
  float Previous_ODR;
} LSM6DSR_X_Data_t;

/**
 * @brief LSM6DSR gyro specific data internal structure definition
 */
typedef struct
{
  LSM6DSR_Combo_Data_t *comboData; /* Combo data to manage in software SPI 3-Wire initialization of the combo sensors */
  float Previous_ODR;
} LSM6DSR_G_Data_t;

/**
 * @}
 */

/** @addtogroup LSM6DSR_Public_Variables Public variables
 * @{
 */

extern ACCELERO_Drv_t LSM6DSR_X_Drv;
extern GYRO_Drv_t LSM6DSR_G_Drv;
extern LSM6DSR_Combo_Data_t LSM6DSR_Combo_Data[LSM6DSR_SENSORS_MAX_NUM];

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

#endif /* __LSM6DSR_ACC_GYRO_DRIVER_HL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
