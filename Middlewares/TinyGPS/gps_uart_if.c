/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gps_uart_if.c
  * @author  JPE
  * @brief   Management of the uart for GPS module communication
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gps_uart_if.h"
#include "tinygps.h"

/* External variables ---------------------------------------------------------*/
/**
  * @brief DMA handle
  */
extern DMA_HandleTypeDef hdma_usart1_rx;

/**
  * @brief UART handle
  */
extern UART_HandleTypeDef huart1;

volatile bool GPS_DataReceived = false;

/**
  * @brief buffer to receive 1 character
  */
uint8_t GPS_charRx;


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

bool gps_uart_Init(void)
{
	MX_DMA_USART1_Init();
	MX_USART1_UART_Init();

	return true;
}

bool gps_uart_DeInit(void)
{
	/* ##-1- Reset peripherals ## */
	__HAL_RCC_USART1_FORCE_RESET();
	__HAL_RCC_USART1_RELEASE_RESET();

	/* ##-2- MspDeInit ## */
	HAL_UART_MspDeInit(&huart1);

	/* ##-3- Disable the NVIC for DMA ## */
	HAL_NVIC_DisableIRQ(DMA1_Channel6_IRQn);

	return true;
}


bool gps_uart_ReceiveInit(void)
{
  UART_WakeUpTypeDef WakeUpSelection;

  /*Set wakeUp event on start bit*/
  WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;

  HAL_UARTEx_StopModeWakeUpSourceConfig(&huart1, WakeUpSelection);

  /* Make sure that no UART transfer is on-going */
  while (__HAL_UART_GET_FLAG(&huart1, USART_ISR_BUSY) == SET);

  /* Make sure that UART is ready to receive)   */
  while (__HAL_UART_GET_FLAG(&huart1, USART_ISR_REACK) == RESET);

  /* Enable USART interrupt */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_WUF);

  /*Enable wakeup from stop mode*/
  HAL_UARTEx_EnableStopMode(&huart1);

  /*Start LPUART receive on IT*/
  HAL_UART_Receive_IT(&huart1, &GPS_charRx, 1);

  return true;
}

void gps_uart_Resume(void)
{
  /*to re-enable lost UART settings*/
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

  /*to re-enable lost DMA settings*/
  if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uartHandle)
{
	if(uartHandle->Instance==USART1)
	{
		if (HAL_UART_ERROR_NONE == uartHandle->ErrorCode)
		{
			if (true == gps_encode(GPS_charRx))
			{
				GPS_DataReceived = true;
			}
		}
		HAL_UART_Receive_IT(uartHandle, &GPS_charRx, 1);
	}
	else
	{
		/* Do Nothing */
	}
}

/* Private Functions Definition -----------------------------------------------*/


/******************* END OF FILE *******************/
