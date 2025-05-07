/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
#include "rtc.h"
extern enum Shutdown_State shutdown;
extern uint8_t bq40z50_address;
static uint8_t offset_rx; // 从机被写寄存器当前偏移地址
static uint8_t offset_tx; // 从机被读寄存器当前偏移地址
static uint8_t first_byte_state = 1; // 是否收到1个字节,也就是逻辑地址，已收到为0，没有收到为1
static uint8_t is_transmitting = 0;
uint8_t DateAndTime[7] = {0};
uint8_t Periph; // 当前进行I2C通信的外设的逻辑地址
uint8_t Error[6] = {0x11, 0x45, 0x14};
/* USER CODE END 0 */

SMBUS_HandleTypeDef hsmbus1;

/* I2C1 init function */

void MX_I2C1_SMBUS_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hsmbus1.Instance = I2C1;
  hsmbus1.Init.Timing = 0x00000000;
  hsmbus1.Init.AnalogFilter = SMBUS_ANALOGFILTER_ENABLE;
  hsmbus1.Init.OwnAddress1 = 24;
  hsmbus1.Init.AddressingMode = SMBUS_ADDRESSINGMODE_7BIT;
  hsmbus1.Init.DualAddressMode = SMBUS_DUALADDRESS_DISABLE;
  hsmbus1.Init.OwnAddress2 = 0;
  hsmbus1.Init.OwnAddress2Masks = SMBUS_OA2_NOMASK;
  hsmbus1.Init.GeneralCallMode = SMBUS_GENERALCALL_DISABLE;
  hsmbus1.Init.NoStretchMode = SMBUS_NOSTRETCH_DISABLE;
  hsmbus1.Init.PacketErrorCheckMode = SMBUS_PEC_DISABLE;
  hsmbus1.Init.PeripheralMode = SMBUS_PERIPHERAL_MODE_SMBUS_HOST;
  hsmbus1.Init.SMBusTimeout = 0x0000800C;
  if (HAL_SMBUS_Init(&hsmbus1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
    // HAL_I2C_EnableListen_IT(&hi2c1);       // 使能I2C1的侦听中�????
    // HAL_SMBUS_EnableListen_IT(&hsmbus1);
  /* USER CODE END I2C1_Init 2 */

}

void HAL_SMBUS_MspInit(SMBUS_HandleTypeDef* smbusHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(smbusHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PA9     ------> I2C1_SCL
    PA10     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_I2C1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_SMBUS_MspDeInit(SMBUS_HandleTypeDef* smbusHandle)
{

  if(smbusHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PA9     ------> I2C1_SCL
    PA10     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_10);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
HAL_StatusTypeDef HAL_SMBUS_Master_Transmit(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint8_t *pData,
                                            uint16_t Size, uint32_t XferOptions, uint32_t Timeout) {
    HAL_StatusTypeDef smbus_status = HAL_SMBUS_Master_Transmit_IT(hsmbus, DevAddress, pData, Size, XferOptions);
    uint32_t smbus_start_time = HAL_GetTick();
    while (HAL_SMBUS_GetState(&hsmbus1) != HAL_SMBUS_STATE_READY) {
        if (HAL_GetTick() - smbus_start_time > Timeout) {
            // MX_I2C1_SMBUS_Init();
            return HAL_TIMEOUT;
        }
    }
    return smbus_status;
}

HAL_StatusTypeDef HAL_SMBUS_Master_Receive(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint8_t *pData,
                                           uint16_t Size, uint32_t XferOptions, uint32_t Timeout) {
    HAL_StatusTypeDef smbus_status = HAL_SMBUS_Master_Receive_IT(hsmbus, DevAddress, pData, Size, XferOptions);
    uint32_t smbus_start_time = HAL_GetTick();
    while (HAL_SMBUS_GetState(&hsmbus1) != HAL_SMBUS_STATE_READY) {
        if (HAL_GetTick() - smbus_start_time > Timeout) {
            // MX_I2C1_SMBUS_Init();
            return HAL_TIMEOUT;
        }
    }
    return smbus_status;
}

HAL_StatusTypeDef wait_for_smbus(uint32_t Timeout) {
    uint32_t smbus_start_time = HAL_GetTick();
    HAL_StatusTypeDef smbus_status = HAL_OK;
    while (HAL_SMBUS_GetState(&hsmbus1) != HAL_SMBUS_STATE_READY) {
        if (HAL_GetTick() - smbus_start_time > Timeout) {
            smbus_status = HAL_TIMEOUT;
            return smbus_status;
        }
    }
    return smbus_status;
}

HAL_StatusTypeDef ManufacturerBlockAccess_write(uint16_t Data) {
    uint16_t Safety_Alert = 0x0050;
    uint16_t Operation_Status = 0x0054;
    uint16_t Manufacturing_Status = 0x0057;
    uint8_t Txdata[4] = {0};
    Txdata[0] = 0x44;
    Txdata[1] = 0x02;
    Txdata[2] = Data & 0xff;
    Txdata[3] = (Data >> 8) & 0xff;
    HAL_StatusTypeDef smbus_status = HAL_SMBUS_Master_Transmit(&hsmbus1, bq40z50_address << 1, Txdata, 4, SMBUS_LAST_FRAME_NO_PEC,100);
    return smbus_status;
}


void RTC_LoadFromBKUP(void) {
    // 检查是否有时间保存
    if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1)) {
        RTC_TimeTypeDef sTime;
        sTime.Hours = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
        sTime.Minutes = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);
        sTime.Seconds = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4);

        // 禁用 RTC 写保护
        __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);

        // 设置 RTC 时间
        if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK) {
        } else {
        }

        // 启用 RTC 写保 ??????????
        __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
    } else {
    }
}

void sys_enter_standby_mode(void) {
    __HAL_RCC_PWR_CLK_ENABLE(); // 使能 PWR 时钟
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU); // 清除唤醒标志

    // 配置 RTC 唤醒定时器，实现 10 秒钟唤醒
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 9, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK) {
        Error_Handler();
    }

    // 进入待机模式
    HAL_PWR_EnterSTANDBYMode();
}


void HAL_SMBUS_ErrorCallback(SMBUS_HandleTypeDef *hsmbus) {
    // 检查发生的错误类型

    // 其他错误处理代码
}

void HAL_SMBUS_ListenCpltCallback(SMBUS_HandleTypeDef *hsmbus) {
    // 完成1次通信，初始化状态机和偏移地址
    first_byte_state = 1;
    Periph = 0;
    offset_rx = 0;
    offset_tx = 0;
    is_transmitting = 0;
    HAL_SMBUS_EnableListen_IT(&hsmbus1); // slave is ready again
}

// I2C设备地址回调函数（地址匹配上以后会进入该函数）
void HAL_SMBUS_AddrCallback(SMBUS_HandleTypeDef *hsmbus, uint8_t TransferDirection, uint16_t AddrMatchCode) {
    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        // 主机发送，从机接收
        if (first_byte_state) {
            // 准备接收1个字节数据
            HAL_SMBUS_Slave_Receive_IT(&hsmbus1, &Periph, 1, SMBUS_NEXT_FRAME); // 每次第个数据均为外设逻辑地址
        }
    } else {
        // 主机接收，从机发送
        // 匹配外设逻辑地址
        is_transmitting = 1;
        switch (Periph) {
            // 如果外设逻辑地址指向RTC
            case 1: {
                // 打开I2C中断发送,将DateAndTime[]中的数据依次发送
                RTC_DateTypeDef sDate;
                RTC_TimeTypeDef sTime;

                HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
                HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

                DateAndTime[0] = sDate.Year;
                DateAndTime[1] = sDate.Month;
                DateAndTime[2] = sDate.Date;
                DateAndTime[3] = sDate.WeekDay;
                DateAndTime[4] = sTime.Hours;
                DateAndTime[5] = sTime.Minutes;
                DateAndTime[6] = sTime.Seconds;

                HAL_SMBUS_Slave_Transmit_IT(&hsmbus1, &DateAndTime[offset_tx], 1, SMBUS_NEXT_FRAME);
                // 打开中断并把DateAndTime[]里面对应的数据发送给主机
                break;
            }

        case 2:
                {
                    HAL_SMBUS_Slave_Transmit_IT(&hsmbus1, &shutdown, 1, SMBUS_NEXT_FRAME);
                    break;
                }
            default:
                // 全都不匹配，发送错误码
                HAL_SMBUS_Slave_Transmit_IT(&hsmbus1, &Error[offset_tx], 1, SMBUS_NEXT_FRAME);
            // 打开中断并把Error[]里面对应的数据发送给主机
        }
    }
}

// I2C数据接收回调函数（在I2C完成 ???????后一次次接收时会关闭中断并调用该函数，因此在处理完成后需要手动重新打开中断接收
void HAL_SMBUS_SlaveRxCpltCallback(SMBUS_HandleTypeDef *hsmbus) {
    if (first_byte_state) {
        // 收到的第1个字节数据（外设地址）
        first_byte_state = 0;
    } else {
        // 收到的第N个字节数据
        offset_rx++;
    }

    if (is_transmitting == 0) {
        // 匹配外设逻辑地址
        switch (Periph) {
            // 如果外设逻辑地址指向RTC
            case 1:
                // 打开I2C中断接收,下一个收到的数据将存放到DateAndTime[offset_rx]
                if (offset_rx <= 6) {
                    HAL_SMBUS_Slave_Receive_IT(&hsmbus1, &DateAndTime[offset_rx], 1, SMBUS_NEXT_FRAME);
                    // 接收数据存到DateAndTime[]里面对应的位移
                } else if (offset_rx >= 7) {
                    RTC_TimeTypeDef sTime;
                    sTime.Hours = DateAndTime[4];
                    sTime.Minutes = DateAndTime[5];
                    sTime.Seconds = DateAndTime[6];

                    RTC_DateTypeDef sDate;
                    sDate.Year = DateAndTime[0];
                    sDate.Month = DateAndTime[1];
                    sDate.Date = DateAndTime[2];
                    sDate.WeekDay = DateAndTime[3];


                    __disable_irq();
                    // 更新RTC时钟的设置
                    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
                        Error_Handler();
                    }
                    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
                        Error_Handler();
                    }

                    __enable_irq();
                }
                break;
            default:
                // 全都不匹配，发送错误码
                HAL_SMBUS_Slave_Transmit_IT(&hsmbus1, &Error[offset_tx], 1, SMBUS_NEXT_FRAME);
        }
    }
}

// I2C数据发送回调函数（在I2C完成一次发送后会关闭中断并调用该函数，因此在处理完成后要手动重新打开中断发送
void HAL_SMBUS_SlaveTxCpltCallback(SMBUS_HandleTypeDef *hsmbus) {
    offset_tx++; // 每发送一个数据，偏移+1
    // 匹配外设逻辑地址
    switch (Periph) {
        // 如果外设逻辑地址指向RTC
        case 1:
            // 打开I2C中断发送,将DateAndTime[]中的数据依次发送
            HAL_SMBUS_Slave_Transmit_IT(&hsmbus1, &DateAndTime[offset_tx], 1, SMBUS_NEXT_FRAME);

        // 打开中断并把DateAndTime[]里面对应的数据发送给主机
            break;
        default:
            // 全都不匹配，发送错误码
            HAL_SMBUS_Slave_Transmit_IT(&hsmbus1, &Error[offset_tx], 1, SMBUS_NEXT_FRAME);
        // 打开中断并把Error[]里面对应的数据发送给主机
    }
}
/* USER CODE END 1 */
