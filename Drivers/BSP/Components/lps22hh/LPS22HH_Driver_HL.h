/**
 ******************************************************************************
 * @file    LPS22HH_Driver_HL.h
 * @author  MEMS Application Team
 * @brief   This file contains definitions for the LPS22HH_Driver_HL.c firmware driver
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
#ifndef __LPS22HH_DRIVER_HL_H
#define __LPS22HH_DRIVER_HL_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "pressure.h"
#include "temperature.h"

/* Include pressure sensor component drivers. */
#include "LPS22HH_Driver.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup LPS22HH LPS22HH
 * @{
 */

/** @addtogroup LPS22HH_Public_Constants Public constants
 * @{
 */

#define LPS22HH_SENSORS_MAX_NUM  1     /**< LPS22HH max number of instances */

/**
 * @}
 */

/** @addtogroup LPS22HH_Public_Types LPS22HH Public Types
 * @{
 */

/**
 * @brief LPS22HH combo specific data internal structure definition
 */
typedef struct
{
  uint8_t isPressInitialized;
  uint8_t isTempInitialized;
  uint8_t isPressEnabled;
  uint8_t isTempEnabled;
  float Last_ODR;
} LPS22HH_Combo_Data_t;

/**
 * @brief LPS22HH pressure specific data internal structure definition
 */
typedef struct
{
  LPS22HH_Combo_Data_t *comboData;       /* Combo data to manage in software enable/disable of the combo sensors */
} LPS22HH_P_Data_t;

/**
 * @brief LPS22HH temperature specific data internal structure definition
 */
typedef struct
{
  LPS22HH_Combo_Data_t *comboData;       /* Combo data to manage in software enable/disable of the combo sensors */
} LPS22HH_T_Data_t;

/**
 * @}
 */

/** @addtogroup LPS22HH_Public_Variables Public variables
 * @{
 */

extern PRESSURE_Drv_t LPS22HH_P_Drv;
extern TEMPERATURE_Drv_t LPS22HH_T_Drv;
extern LPS22HH_Combo_Data_t LPS22HH_Combo_Data[LPS22HH_SENSORS_MAX_NUM];

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

#endif /* __LPS22HH_DRIVER_HL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
