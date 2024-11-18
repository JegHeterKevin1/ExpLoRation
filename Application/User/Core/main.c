/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h> /* strlen */
#include <stdio.h>  /* snprintf */
#include <math.h>   /* trunc */
#include "app_lorawan.h"
#include "sensor.h"
#include "serial_protocol.h"
#include "sys_sensors.h"

/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/

/* Extern variables ----------------------------------------------------------*/
extern volatile uint8_t DataLoggerActive; /*!< DataLogger Flag */
extern UART_HandleTypeDef huart2;     /*!< UART HANDLE */

/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/

#define MAX_BUF_SIZE 				256


/* Private variables ---------------------------------------------------------*/

char 				dataOut[MAX_BUF_SIZE];      /*!< DataOut Frame */
RTC_HandleTypeDef 	RtcHandle;             		/*!< RTC HANDLE */
volatile uint32_t 	DataTxPeriod 		= 1;    /*!< TX DATA Period */
volatile uint8_t 	AutoInit 			= 1;    /*!< Auto Init */
SensorAxes_t 		ACC_Value;                  /*!< Acceleration Value */
SensorAxes_t 		GYR_Value;                  /*!< Gyroscope Value */
SensorAxes_t 		MAG_Value;                  /*!< Magnetometer Value */
float 				PRESSURE_Value;             /*!< Pressure Value */
float 				HUMIDITY_Value;             /*!< Humidity Value */
float 				TEMPERATURE_Value;          /*!< Temperature Value */
volatile uint32_t 	Int_Current_Time1 	= 0; 	/*!< Int_Current_Time1 Value */
volatile uint32_t 	Int_Current_Time2 	= 0; 	/*!< Int_Current_Time2 Value */
void 				*ACCELERO_handle 	= NULL;
void 				*GYRO_handle 		= NULL;
void 				*MAGNETO_handle 	= NULL;
void 				*HUMIDITY_handle 	= NULL;
void 				*TEMPERATURE_handle = NULL;
void 				*PRESSURE_handle 	= NULL;
int 				RTC_SYNCH_PREDIV;

uint8_t new_data = 0;
uint8_t new_data_flags = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

#if defined(IKS01A1_ACTIVATED)
static void Accelero_Sensor_Handler(TMsg *Msg);
static void Gyro_Sensor_Handler(TMsg *Msg);
static void Magneto_Sensor_Handler(TMsg *Msg);
static void Pressure_Sensor_Handler(TMsg *Msg);
static void Humidity_Sensor_Handler(TMsg *Msg);
static void Temperature_Sensor_Handler(TMsg *Msg);
#endif // IKS01A1_ACTIVATED


/* Private user code ---------------------------------------------------------*/


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
#if defined(IKS01A1_ACTIVATED)
	TMsg			MsgDat;
#endif // IKS01A1_ACTIVATED

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	BSP_LED_Init(LED_BLUE);
	BSP_LED_Init(LED_GREEN);
	BSP_LED_Init(LED_RED);

	/* Send every time button is pushed */
	BSP_PB_Init(BUTTON_SW1, BUTTON_MODE_EXTI);
	BSP_PB_Init(BUTTON_SW2, BUTTON_MODE_EXTI);
	BSP_PB_Init(BUTTON_SW3, BUTTON_MODE_EXTI);

#if defined(LORAWAN_ACTIVATED)
	/* Initialize all configured peripherals */
	MX_LoRaWAN_Init();
#endif	// LORAWAN_ACTIVATED

	/* Init IKS01A1 */
#if defined(IKS01A1_ACTIVATED)

	/*Initialize the Sensors */
	EnvSensors_Init();

#endif // IKS01A1_ACTIVATED

	/* Infinite loop */
	while (1)
	{

#if defined(LORAWAN_ACTIVATED)
		MX_LoRaWAN_Process();
#endif	// LORAWAN_ACTIVATED

#if defined(IKS01A1_ACTIVATED)

		/* IKS01A1 Sensors handling */
		AutoInit			= 1;

		Accelero_Sensor_Handler(&MsgDat);
		Gyro_Sensor_Handler(&MsgDat);
		Magneto_Sensor_Handler(&MsgDat);
		Humidity_Sensor_Handler(&MsgDat);
		Temperature_Sensor_Handler(&MsgDat);
		Pressure_Sensor_Handler(&MsgDat);

		HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 5000);

		HAL_Delay(1000);

#endif // IKS01A1_ACTIVATED

	/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}


#if defined(IKS01A1_ACTIVATED)

/**
 * @brief  Handles the ACCELERO axes data getting/sending
 * @param  Msg the ACCELERO part of the stream
 * @retval None
 */
static void Accelero_Sensor_Handler(TMsg *Msg)
{
	int32_t data[3];
	uint8_t status = 0;
	uint8_t drdy = 0;

	if (BSP_ACCELERO_IsInitialized(ACCELERO_handle, &status) == COMPONENT_OK && status == 1)
	{
		BSP_ACCELERO_Get_DRDY_Status(ACCELERO_handle, &drdy);

		if (drdy != 0)
		{
			new_data++;
			new_data_flags |= 1;

			BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACC_Value);

			if (DataLoggerActive)
			{
				Serialize_s32(&Msg->Data[19], ACC_Value.AXIS_X, 4);
				Serialize_s32(&Msg->Data[23], ACC_Value.AXIS_Y, 4);
				Serialize_s32(&Msg->Data[27], ACC_Value.AXIS_Z, 4);
			}
			else if (AutoInit)
			{
				data[0] = ACC_Value.AXIS_X;
				data[1] = ACC_Value.AXIS_Y;
				data[2] = ACC_Value.AXIS_Z;

				snprintf(dataOut, MAX_BUF_SIZE, "\tACC_X: %d, ACC_Y: %d, ACC_Z: %d\r\n", (int)data[0], (int)data[1], (int)data[2]);
				HAL_UART_Transmit(&huart2, (uint8_t *)dataOut, strlen(dataOut), 5000);
			}
		}
	}
}

/**
 * @brief  Handles the GYRO axes data getting/sending
 * @param  Msg the GYRO part of the stream
 * @retval None
 */
static void Gyro_Sensor_Handler(TMsg *Msg)
{
	int32_t data[3];
	uint8_t status = 0;
	uint8_t drdy = 0;

	if (BSP_GYRO_IsInitialized(GYRO_handle, &status) == COMPONENT_OK && status == 1)
	{
		BSP_GYRO_Get_DRDY_Status(GYRO_handle, &drdy);

		if (drdy != 0)
		{
			new_data++;
			new_data_flags |= 2;

			BSP_GYRO_Get_Axes(GYRO_handle, &GYR_Value);

			if (DataLoggerActive)
			{
				Serialize_s32(&Msg->Data[31], GYR_Value.AXIS_X, 4);
				Serialize_s32(&Msg->Data[35], GYR_Value.AXIS_Y, 4);
				Serialize_s32(&Msg->Data[39], GYR_Value.AXIS_Z, 4);
			}
			else if (AutoInit)
			{
				data[0] = GYR_Value.AXIS_X;
				data[1] = GYR_Value.AXIS_Y;
				data[2] = GYR_Value.AXIS_Z;

				snprintf(dataOut, MAX_BUF_SIZE, "\tGYR_X: %d, GYR_Y: %d, GYR_Z: %d\r\n", (int)data[0], (int)data[1], (int)data[2]);
				HAL_UART_Transmit(&huart2, (uint8_t *)dataOut, strlen(dataOut), 5000);
			}
		}
	}
}

/**
 * @brief  Handles the MAGNETO axes data getting/sending
 * @param  Msg the MAGNETO part of the stream
 * @retval None
 */
static void Magneto_Sensor_Handler(TMsg *Msg)
{
	int32_t data[3];
	uint8_t status = 0;
	uint8_t drdy = 0;

	if (BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status) == COMPONENT_OK && status == 1)
	{
		BSP_MAGNETO_Get_DRDY_Status(MAGNETO_handle, &drdy);

		if (drdy != 0)
		{
			new_data++;
			new_data_flags |= 4;

			BSP_MAGNETO_Get_Axes(MAGNETO_handle, &MAG_Value);

			if (DataLoggerActive)
			{
				Serialize_s32(&Msg->Data[43], (int32_t)MAG_Value.AXIS_X, 4);
				Serialize_s32(&Msg->Data[47], (int32_t)MAG_Value.AXIS_Y, 4);
				Serialize_s32(&Msg->Data[51], (int32_t)MAG_Value.AXIS_Z, 4);
			}
			else if (AutoInit)
			{
				data[0] = MAG_Value.AXIS_X;
				data[1] = MAG_Value.AXIS_Y;
				data[2] = MAG_Value.AXIS_Z;

				snprintf(dataOut, MAX_BUF_SIZE, "\tMAG_X: %d, MAG_Y: %d, MAG_Z: %d\r\n", (int)data[0], (int)data[1], (int)data[2]);
				HAL_UART_Transmit(&huart2, (uint8_t *)dataOut, strlen(dataOut), 5000);
			}
		}
	}
}

/**
 * @brief  Handles the PRESSURE sensor data getting/sending
 * @param  Msg the PRESSURE part of the stream
 * @retval None
 */
static void Pressure_Sensor_Handler(TMsg *Msg)
{
	uint8_t status = 0;
	uint8_t drdy = 0;

	if (BSP_PRESSURE_IsInitialized(PRESSURE_handle, &status) == COMPONENT_OK && status == 1)
	{
		BSP_PRESSURE_Get_DRDY_Status(PRESSURE_handle, &drdy);

		if (drdy != 0)
		{
			new_data++;
			new_data_flags |= 8;

			BSP_PRESSURE_Get_Press(PRESSURE_handle, &PRESSURE_Value);

			if (DataLoggerActive)
			{
				memcpy(&Msg->Data[7], (void *)&PRESSURE_Value, sizeof(float));
			}
			else if (AutoInit)
			{
				displayFloatToInt_t out_value;
				FloatToInt(PRESSURE_Value, &out_value, 2);
				snprintf(dataOut, MAX_BUF_SIZE, "\tPRESS: %c%d.%02d\r\n", ((out_value.sign) ? '-' : '+'), (int)out_value.out_int,
						 (int)out_value.out_dec);
				HAL_UART_Transmit(&huart2, (uint8_t *)dataOut, strlen(dataOut), 5000);
			}
		}
	}
}

/**
 * @brief  Handles the HUMIDITY sensor data getting/sending
 * @param  Msg the HUMIDITY part of the stream
 * @retval None
 */
static void Humidity_Sensor_Handler(TMsg *Msg)
{
	uint8_t status = 0;
	uint8_t drdy = 0;

	if (BSP_HUMIDITY_IsInitialized(HUMIDITY_handle, &status) == COMPONENT_OK && status == 1)
	{
		BSP_HUMIDITY_Get_DRDY_Status(HUMIDITY_handle, &drdy);

		if (drdy != 0)
		{
			new_data++;
			new_data_flags |= 16;

			BSP_HUMIDITY_Get_Hum(HUMIDITY_handle, &HUMIDITY_Value);

			if (DataLoggerActive)
			{
				memcpy(&Msg->Data[15], (void *)&HUMIDITY_Value, sizeof(float));
			}
			else if (AutoInit)
			{
				displayFloatToInt_t out_value;
				FloatToInt(HUMIDITY_Value, &out_value, 2);
				snprintf(dataOut, MAX_BUF_SIZE, "\tHUM: %c%d.%02d\r\n", ((out_value.sign) ? '-' : '+'), (int)out_value.out_int,
						 (int)out_value.out_dec);
				HAL_UART_Transmit(&huart2, (uint8_t *)dataOut, strlen(dataOut), 5000);
			}
		}
	}
}

/**
 * @brief  Handles the TEMPERATURE sensor data getting/sending
 * @param  Msg the TEMPERATURE part of the stream
 * @retval None
 */
static void Temperature_Sensor_Handler(TMsg *Msg)
{
	uint8_t status = 0;
	uint8_t drdy = 0;

	if (BSP_TEMPERATURE_IsInitialized(TEMPERATURE_handle, &status) == COMPONENT_OK && status == 1)
	{
		BSP_TEMPERATURE_Get_DRDY_Status(TEMPERATURE_handle, &drdy);

		if (drdy != 0)
		{
			new_data++;
			new_data_flags |= 32;

			BSP_TEMPERATURE_Get_Temp(TEMPERATURE_handle, &TEMPERATURE_Value);

			if (DataLoggerActive)
			{
				memcpy(&Msg->Data[11], (void *)&TEMPERATURE_Value, sizeof(float));
			}
			else if (AutoInit)
			{
				displayFloatToInt_t out_value;
				FloatToInt(TEMPERATURE_Value, &out_value, 2);
				snprintf(dataOut, MAX_BUF_SIZE, "\tTEMP: %c%d.%02d\r\n", ((out_value.sign) ? '-' : '+'), (int)out_value.out_int,
						 (int)out_value.out_dec);
				HAL_UART_Transmit(&huart2, (uint8_t *)dataOut, strlen(dataOut), 5000);
			}
		}
	}
}

#endif


/**
  * @brief System Clock Configuration
  * @retval None
  */
static void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure LSE Drive Capability
	*/
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
	/** Configure the main internal regulator output voltage
	*/
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
							  |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
							  |RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief  Get the DMA Stream pending flags
 * @param  handle_dma DMA handle
 * @retval The state of FLAG (SET or RESET)
 */
uint32_t Get_DMA_Flag_Status(DMA_HandleTypeDef *handle_dma)
{
	return (__HAL_DMA_GET_FLAG(handle_dma, __HAL_DMA_GET_TE_FLAG_INDEX(handle_dma)));
}

/**
 * @brief  Returns the number of remaining data units in the current DMAy Streamx transfer
 * @param  handle_dma DMA handle
 * @retval The number of remaining data units in the current DMA Stream transfer
 */
uint32_t Get_DMA_Counter(DMA_HandleTypeDef *handle_dma)
{
	return (handle_dma->Instance->CNDTR);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
		BSP_LED_On(LED2);
		HAL_Delay(500);
		BSP_LED_Off(LED2);
		HAL_Delay(500);
	}
}


/**
 * @brief  Splits a float into two integer values.
 * @param  in the float value as input
 * @param  out_value the pointer to the sign, integer and decimal parts as output
 * @param  dec_prec the decimal precision to be used
 * @retval None
 */
void FloatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec)
{
	if (in >= 0.0f)
	{
		out_value->sign = 0;
	}
	else
	{
		out_value->sign = 1;
		in = -in;
	}

	out_value->out_int = (int32_t)in;
	in = in - (float)(out_value->out_int);
	out_value->out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
