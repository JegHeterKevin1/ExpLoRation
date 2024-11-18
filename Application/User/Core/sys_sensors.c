/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sys_sensors.c
  * @author  MCD Application Team
  * @brief   Manages the sensors on the application
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "sys_conf.h"
#include "sys_sensors.h"
#include "sensor.h"
#include "main.h"
#include "tinygps.h"


/* External variables ---------------------------------------------------------*/
extern void		*HUMIDITY_handle;
extern void		*TEMPERATURE_handle;
extern void		*PRESSURE_handle;
extern void 	*ACCELERO_handle;
extern void 	*GYRO_handle;
extern void 	*MAGNETO_handle;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */
#define STSOP_LATTITUDE           ((float) 43.618622 )  /*!< default latitude position */
#define STSOP_LONGITUDE           ((float) 7.051415  )  /*!< default longitude position */
#define MAX_GPS_POS               ((int32_t) 8388607 )  /*!< 2^23 - 1 */
#define HUMIDITY_DEFAULT_VAL      50.0f                 /*!< default humidity */
#define TEMPERATURE_DEFAULT_VAL   18.0f                 /*!< default temperature */
#define PRESSURE_DEFAULT_VAL      1000.0f               /*!< default pressure */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/


/* Exported functions --------------------------------------------------------*/
int32_t  EnvSensors_Read(sensor_t *sensor_data)
{
	float 					flat, flon;
	unsigned long 			age;
	float 					HUMIDITY_Value = HUMIDITY_DEFAULT_VAL;
	float 					TEMPERATURE_Value = TEMPERATURE_DEFAULT_VAL;
	float					PRESSURE_Value = PRESSURE_DEFAULT_VAL;
	displayFloatToInt_t		lat_out_value;
	displayFloatToInt_t		lon_out_value;

	BSP_HUMIDITY_Get_Hum(HUMIDITY_handle, &HUMIDITY_Value);
	BSP_TEMPERATURE_Get_Temp(TEMPERATURE_handle, &TEMPERATURE_Value);
	BSP_PRESSURE_Get_Press(PRESSURE_handle, &PRESSURE_Value);

	sensor_data->pressure    = PRESSURE_Value;
	sensor_data->temperature = TEMPERATURE_Value;
	sensor_data->humidity    = HUMIDITY_Value;

	gps_f_get_position(&flat, &flon, &age);

	FloatToInt(flat, &lat_out_value, 5);

	// Latitude is negative
	if (0 != lat_out_value.sign)
	{
		sensor_data->latitude  = -(int32_t)(lat_out_value.out_int * 100000 + lat_out_value.out_dec);
	}
	// Latitude is positive
	else
	{
		sensor_data->latitude  = (int32_t)(lat_out_value.out_int * 100000 + lat_out_value.out_dec);
	}

	FloatToInt(flon, &lon_out_value, 5);

	// Longitude is negative
	if (0 != lon_out_value.sign)
	{
		sensor_data->longitude = -(int32_t)(lon_out_value.out_int * 100000 + lon_out_value.out_dec);
	}
	// Longitude is positive
	else
	{
		sensor_data->longitude = (int32_t)(lon_out_value.out_int * 100000 + lon_out_value.out_dec);
	}

	return 0;
}


int32_t  EnvSensors_Init(void)
{
	/* Init */
	BSP_ACCELERO_Init(LSM6DS0_X_0, &ACCELERO_handle);
	BSP_GYRO_Init(LSM6DS0_G_0, &GYRO_handle);
	BSP_MAGNETO_Init(LIS3MDL_0, &MAGNETO_handle);
	BSP_HUMIDITY_Init(HTS221_H_0, &HUMIDITY_handle);
	BSP_TEMPERATURE_Init(HTS221_H_0, &TEMPERATURE_handle);
	BSP_PRESSURE_Init(LPS25HB_P_0, &PRESSURE_handle);

	/* Enable */
	BSP_ACCELERO_Sensor_Enable(ACCELERO_handle);
	BSP_GYRO_Sensor_Enable(GYRO_handle);
	BSP_MAGNETO_Sensor_Enable(MAGNETO_handle);
	BSP_HUMIDITY_Sensor_Enable(HUMIDITY_handle);
	BSP_TEMPERATURE_Sensor_Enable(TEMPERATURE_handle);
	BSP_PRESSURE_Sensor_Enable(PRESSURE_handle);

	return 0;
}


/* Private Functions Definition -----------------------------------------------*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
