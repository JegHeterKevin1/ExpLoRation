/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gps_uart_if.h
  * @author  JPE
  * @brief   Header for USART interface configuration
  ******************************************************************************
  */
/* USER CODE END Header */

#include "usart.h"
#include "dma.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPS_UART_IF_H__
#define __GPS_UART_IF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
bool gps_uart_Init(void);
bool gps_uart_DeInit(void);
bool gps_uart_ReceiveInit(void);
void gps_uart_Resume(void);


#ifdef __cplusplus
}
#endif

#endif /* __GPS_UART_IF_H__ */

/******* END OF FILE *******/
