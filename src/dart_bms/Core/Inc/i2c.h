/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SMBUS_HandleTypeDef hsmbus1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_I2C1_SMBUS_Init(void);

/* USER CODE BEGIN Prototypes */
HAL_StatusTypeDef HAL_SMBUS_Master_Transmit(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint8_t *pData,
                                            uint16_t Size, uint32_t XferOptions, uint32_t Timeout);

HAL_StatusTypeDef HAL_SMBUS_Master_Receive(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint8_t *pData,
                                           uint16_t Size, uint32_t XferOptions, uint32_t Timeout);

HAL_StatusTypeDef wait_for_smbus(uint32_t Timeout);

HAL_StatusTypeDef ManufacturerBlockAccess_write(uint16_t Data);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

