/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"
#include "stm32wlxx_nucleo.h"
#include "stm32wlxx_hal_conf.h"
#include "stm32wlxx_hal_def.h"

#include "x_nucleo_iks01a1.h"
#include "x_nucleo_iks01a1_accelero.h"
#include "x_nucleo_iks01a1_gyro.h"
#include "x_nucleo_iks01a1_magneto.h"
#include "x_nucleo_iks01a1_humidity.h"
#include "x_nucleo_iks01a1_temperature.h"
#include "x_nucleo_iks01a1_pressure.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define USARTx_TX_AF						GPIO_AF7_USART2
#define USARTx_RX_AF						GPIO_AF7_USART2

#define RTC_ASYNCH_PREDIV_LSI				0x7F
#define RTC_SYNCH_PREDIV_LSI				0xF9

#define RTC_ASYNCH_PREDIV_LSE				0x7F
#define RTC_SYNCH_PREDIV_LSE				0x00FF
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define	LORAWAN_ACTIVATED			1
#define	IKS01A1_ACTIVATED			1
#define	GNSS_ACTIVATED				1

// Enable sensor masks
#define PRESSURE_SENSOR                         0x00000001
#define TEMPERATURE_SENSOR                      0x00000002
#define HUMIDITY_SENSOR                         0x00000004
#define UV_SENSOR                               0x00000008  // for future use
#define ACCELEROMETER_SENSOR                    0x00000010
#define GYROSCOPE_SENSOR                        0x00000020
#define MAGNETIC_SENSOR                         0x00000040
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
uint32_t Get_DMA_Flag_Status(DMA_HandleTypeDef *handle_dma);
uint32_t Get_DMA_Counter(DMA_HandleTypeDef *handle_dma);
void Config_DMA_Handler(DMA_HandleTypeDef *handle_dma);
void FloatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RTC_N_PREDIV_S 			10
#define RTC_PREDIV_S 			((1<<RTC_N_PREDIV_S)-1)
#define RTC_PREDIV_A 			((1<<(15-RTC_N_PREDIV_S))-1)
#define LED1_Pin 				GPIO_PIN_15
#define LED1_GPIO_Port 			GPIOB
#define LED2_Pin 				GPIO_PIN_9
#define LED2_GPIO_Port 			GPIOB
#define PROB3_Pin 				GPIO_PIN_14
#define PROB3_GPIO_Port 		GPIOB
#define RF_CTRL3_Pin 			GPIO_PIN_3
#define RF_CTRL3_GPIO_Port 		GPIOC
#define BUT1_Pin 				GPIO_PIN_0
#define BUT1_GPIO_Port 			GPIOA
#define BUT1_EXTI_IRQn 			EXTI0_IRQn
#define PROB2_Pin 				GPIO_PIN_13
#define PROB2_GPIO_Port 		GPIOB
#define RF_CTRL2_Pin 			GPIO_PIN_5
#define RF_CTRL2_GPIO_Port 		GPIOC
#define PROB1_Pin 				GPIO_PIN_12
#define PROB1_GPIO_Port 		GPIOB
#define RF_CTRL1_Pin 			GPIO_PIN_4
#define RF_CTRL1_GPIO_Port 		GPIOC
#define BUT3_Pin 				GPIO_PIN_6
#define BUT3_GPIO_Port 			GPIOC
#define BUT2_Pin 				GPIO_PIN_1
#define BUT2_GPIO_Port 			GPIOA
#define BUT2_EXTI_IRQn 			EXTI1_IRQn
#define LED3_Pin 				GPIO_PIN_11
#define LED3_GPIO_Port 			GPIOB
#define USARTx_RX_Pin 			GPIO_PIN_3
#define USARTx_RX_GPIO_Port 	GPIOA
#define USARTx_TX_Pin 			GPIO_PIN_2
#define USARTx_TX_GPIO_Port 	GPIOA
#define USART1_RX_Pin 			GPIO_PIN_7
#define USART1_RX_GPIO_Port 	GPIOB
#define USART1_TX_Pin 			GPIO_PIN_6
#define USART1_TX_GPIO_Port 	GPIOB
#define PROB4_Pin 				GPIO_PIN_10
#define PROB4_GPIO_Port 		GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
