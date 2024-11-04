/**
 ******************************************************************************
 * @file    IIS2DLPC_ACC_driver_HL.h
 * @author  MEMS Application Team
 * @brief   This file contains definitions for the IIS2DLPC_ACC_driver_HL.c firmware driver
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
#ifndef __IIS2DLPC_ACC_DRIVER_HL_H
#define __IIS2DLPC_ACC_DRIVER_HL_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/

#include "accelerometer.h"

/* Include accelero sensor component drivers. */
#include "IIS2DLPC_ACC_driver.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup IIS2DLPC IIS2DLPC
 * @{
 */

/** @addtogroup IIS2DLPC_Public_Constants Public constants
 * @{
 */

#define IIS2DLPC_SENSORS_MAX_NUM  1     /**< IIS2DLPC max number of instances */

/** @addtogroup IIS2DLPC_ACC_SENSITIVITY Accelero sensitivity values based on selected full scale and power mode
 * @{
 */

#define IIS2DLPC_ACC_SENSITIVITY_FOR_FS_2G_LOPOW1_MODE   0.976  /**< Sensitivity value for 2g full scale, Low-power1 mode [mg/LSB] */
#define IIS2DLPC_ACC_SENSITIVITY_FOR_FS_2G_OTHER_MODES   0.244  /**< Sensitivity value for 2g full scale, all other modes except Low-power1 [mg/LSB] */

#define IIS2DLPC_ACC_SENSITIVITY_FOR_FS_4G_LOPOW1_MODE   1.952  /**< Sensitivity value for 4g full scale, Low-power1 mode [mg/LSB] */
#define IIS2DLPC_ACC_SENSITIVITY_FOR_FS_4G_OTHER_MODES   0.488  /**< Sensitivity value for 4g full scale, all other modes except Low-power1 [mg/LSB] */

#define IIS2DLPC_ACC_SENSITIVITY_FOR_FS_8G_LOPOW1_MODE   3.904  /**< Sensitivity value for 8g full scale, Low-power1 mode [mg/LSB] */
#define IIS2DLPC_ACC_SENSITIVITY_FOR_FS_8G_OTHER_MODES   0.976  /**< Sensitivity value for 8g full scale, all other modes except Low-power1 [mg/LSB] */

#define IIS2DLPC_ACC_SENSITIVITY_FOR_FS_16G_LOPOW1_MODE  7.808  /**< Sensitivity value for 16g full scale, Low-power1 mode [mg/LSB] */
#define IIS2DLPC_ACC_SENSITIVITY_FOR_FS_16G_OTHER_MODES  1.952  /**< Sensitivity value for 16g full scale, all other modes except Low-power1 [mg/LSB] */

/**
 * @}
 */

/** @addtogroup IIS2DLPC_ACT_THRESHOLD Sleep-to-wake, return-to-sleep activation threshold in low-power mode
 * @{
 */

#define IIS2DLPC_ACT_THRESHOLD_LOW       0x01  /**< Lowest  value of threshold */
#define IIS2DLPC_ACT_THRESHOLD_MID_LOW   0x0F
#define IIS2DLPC_ACT_THRESHOLD_MID       0x1F
#define IIS2DLPC_ACT_THRESHOLD_MID_HIGH  0x2F
#define IIS2DLPC_ACT_THRESHOLD_HIGH      0x3F  /**< Highest value of threshold */

/**
 * @}
 */

/** @addtogroup IIS2DLPC_ACT_DURATION Sleep-to-wake, return-to-sleep duration
 * @{
 */

#define IIS2DLPC_ACT_DURATION_LOW       0x01  /**< Lowest  value of threshold */
#define IIS2DLPC_ACT_DURATION_MID_LOW   0x0F
#define IIS2DLPC_ACT_DURATION_MID       0x1F
#define IIS2DLPC_ACT_DURATION_MID_HIGH  0x2F
#define IIS2DLPC_ACT_DURATION_HIGH      0x3F  /**< Highest value of threshold */

/**
 * @}
 */

/**
 * @}
 */

/** @addtogroup IIS2DLPC_Public_Types IIS2DLPC Public Types
 * @{
 */

#if 0 /* _NOTE_: Remove this line if you plan to use this structure below */
/**
 * @brief IIS2DLPC accelero extended features driver internal structure definition
 */
typedef struct
{
  /* _NOTE_: Possible to add the functions for extended features of the sensor here */
} IIS2DLPC_ExtDrv_t;
#endif /* _NOTE_: Remove this line if you plan to use this structure above */

/**
 * @brief IIS2DLPC specific data internal structure definition
 */
typedef struct
{
  float Previous_ODR;
} IIS2DLPC_Data_t;

/**
 * @}
 */

/** @addtogroup IIS2DLPC_Public_Variables Public variables
 * @{
 */

extern ACCELERO_Drv_t IIS2DLPC_Drv;
#if 0 /* _NOTE_: Remove this line if you plan to use this structure below */
extern IIS2DLPC_ExtDrv_t IIS2DLPC_ExtDrv;
#endif /* _NOTE_: Remove this line if you plan to use this structure above */

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

#endif /* __IIS2DLPC_ACC_DRIVER_HL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
