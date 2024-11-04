/**
 ******************************************************************************
 * @file    LPS33HW_Driver_HL.h
 * @author  MEMS Application Team
 * @brief   This file contains definitions for the LPS33HW_Driver_HL.c firmware driver
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
#ifndef __LPS33HW_DRIVER_HL_H
#define __LPS33HW_DRIVER_HL_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "pressure.h"
#include "temperature.h"

/* Include pressure sensor component drivers. */
#include "LPS33HW_Driver.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup LPS33HW LPS33HW
 * @{
 */

/** @addtogroup LPS33HW_Public_Constants Public constants
 * @{
 */

#define LPS33HW_SENSORS_MAX_NUM  1     /**< LPS33HW max number of instances */

/**
 * @}
 */

/** @addtogroup LPS33HW_Public_Types LPS33HW Public Types
 * @{
 */

/**
 * @brief LPS33HW combo specific data internal structure definition
 */
typedef struct
{
  uint8_t isPressInitialized;
  uint8_t isTempInitialized;
  uint8_t isPressEnabled;
  uint8_t isTempEnabled;
  float Last_ODR;
} LPS33HW_Combo_Data_t;

/**
 * @brief LPS33HW pressure specific data internal structure definition
 */
typedef struct
{
  LPS33HW_Combo_Data_t *comboData;       /* Combo data to manage in software enable/disable of the combo sensors */
} LPS33HW_P_Data_t;

/**
 * @brief LPS33HW temperature specific data internal structure definition
 */
typedef struct
{
  LPS33HW_Combo_Data_t *comboData;       /* Combo data to manage in software enable/disable of the combo sensors */
} LPS33HW_T_Data_t;

/**
 * @}
 */

/** @addtogroup LPS33HW_Public_Variables Public variables
 * @{
 */

extern PRESSURE_Drv_t LPS33HW_P_Drv;
extern TEMPERATURE_Drv_t LPS33HW_T_Drv;
extern LPS33HW_Combo_Data_t LPS33HW_Combo_Data[LPS33HW_SENSORS_MAX_NUM];

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

#endif /* __LPS33HW_DRIVER_HL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
