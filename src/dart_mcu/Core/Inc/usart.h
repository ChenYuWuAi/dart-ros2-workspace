/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
#define UART1_MAX_RECEIVE_BUFFER_LENGTH 50

// 遥控器UART相关宏定义
#define RC_UART                       huart1
#define RC_UART_HANDLE                &RC_UART
#define RC_UART_RXBUFFER              uart1RecBuffer
#define RC_UART_BUFFER_LENGTH         UART1_MAX_RECEIVE_BUFFER_LENGTH

// 裁判系统UART相关宏定义
#define REFEREE_UART                  huart3
#define REFEREE_UART_HANDLE           &REFEREE_UART
#define REFEREE_UART_RXBUFFER         judge_rx_buffer
#define REFEREE_UART_BUFFER_LENGTH    UART3_MAX_RECEIVE_BUFFER_LENGTH

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
extern uint8_t uart1RecBuffer[UART1_MAX_RECEIVE_BUFFER_LENGTH];
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

