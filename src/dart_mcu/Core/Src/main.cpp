/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "rng.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dbus.h"
#include "judge_receive.h"
#include "motor.h"
#include "sound_effect.h"
#include "velocimeter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM12_Init();
  MX_TIM6_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();
  MX_RNG_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    sFilterConfig.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_UARTEx_ReceiveToIdle_DMA(REFEREE_UART_HANDLE, REFEREE_UART_RXBUFFER[0], REFEREE_UART_BUFFER_LENGTH);
    HAL_UARTEx_ReceiveToIdle_DMA(RC_UART_HANDLE, RC_UART_RXBUFFER, RC_UART_BUFFER_LENGTH);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 全局变量用于裁判系统的双缓冲选择
static volatile uint8_t g_judge_decode_memory = MEMORY0;

/*---------------------------------function of interrupt begin------------------------------------*/
// 遥控&裁判系统解码
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == RC_UART_HANDLE && huart->RxEventType == HAL_UART_RXEVENT_IDLE) {
        // 确保数据大小正确且不超过缓冲区大小
        if (Size == RC_FRAME_LEN && Size <= RC_UART_BUFFER_LENGTH) {
            // 安全复制数据
            memcpy(RC_rx_buffer, RC_UART_RXBUFFER, Size);
            DT7_Decode();
        }

        // 清空缓冲区并重启DMA接收
        memset(RC_UART_RXBUFFER, 0, RC_UART_BUFFER_LENGTH);
        HAL_UARTEx_ReceiveToIdle_DMA(RC_UART_HANDLE, RC_UART_RXBUFFER, RC_UART_BUFFER_LENGTH);
    } else if (huart == REFEREE_UART_HANDLE &&
               huart->RxEventType == HAL_UART_RXEVENT_IDLE) {        // 确保接收的数据大小不超过缓冲区大小
        if (Size > 0 && Size <= REFEREE_UART_BUFFER_LENGTH) {
            // 使用全局变量，并在一次操作中获取当前值，避免竞态条件
            uint8_t current_buffer = g_judge_decode_memory;
            uint8_t next_buffer = (current_buffer + 1) % 2;

            // 先设置下一次接收
            HAL_UARTEx_ReceiveToIdle_DMA(REFEREE_UART_HANDLE, REFEREE_UART_RXBUFFER[next_buffer],
                                         REFEREE_UART_BUFFER_LENGTH);

            // 处理当前接收到的数据
            RefereeReceive(Size, REFEREE_UART_RXBUFFER[current_buffer]);

            // 切换缓冲区
            g_judge_decode_memory = next_buffer;
        } else {
            // 接收数据过大或无效，重置接收
            HAL_UARTEx_ReceiveToIdle_DMA(REFEREE_UART_HANDLE, REFEREE_UART_RXBUFFER[g_judge_decode_memory],
                                         REFEREE_UART_BUFFER_LENGTH);
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == RC_UART_HANDLE || huart == REFEREE_UART_HANDLE) {
        // 清除全部错误标志
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE);
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
        __HAL_UART_CLEAR_OREFLAG(huart);

        // 重置DMA
        HAL_UART_AbortReceive(huart);
        // 重新启动DMA接收
        if (huart == RC_UART_HANDLE) {
            memset(RC_UART_RXBUFFER, 0, RC_UART_BUFFER_LENGTH);
            HAL_UARTEx_ReceiveToIdle_DMA(RC_UART_HANDLE, RC_UART_RXBUFFER, RC_UART_BUFFER_LENGTH);
        } else if (huart == REFEREE_UART_HANDLE) {
            // 使用全局变量，确保与RxEventCallback使用相同的缓冲区
            HAL_UARTEx_ReceiveToIdle_DMA(REFEREE_UART_HANDLE, REFEREE_UART_RXBUFFER[g_judge_decode_memory],
                                         REFEREE_UART_BUFFER_LENGTH);
        }

        // 重新启用中断
        __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
        __HAL_UART_ENABLE_IT(huart, UART_IT_ERR);

        // 清除错误状态
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->gState = HAL_UART_STATE_READY;
        huart->RxState = HAL_UART_STATE_READY;
    }
}

/**
 * @brief 在此函数中添加自动恢复逻辑
 */

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    uint32_t err = HAL_CAN_GetError(hcan);  // 获取错误码
    // 检测到 Bus-Off 错误
    if (err & HAL_CAN_ERROR_BOF) {
        reboot_can(hcan);
    }
}


// CAN接收中断
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t aData[8];
    if (hcan == &hcan1) {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, aData);
        switch (RxHeader.StdId) {
            case 0x201: {
                motor::MotorTriggerLS.decodeCanMsg(&RxHeader, aData);
                break;
            }
            case 0x202: {
                motor::MotorLoad[0].decodeCanMsg(&RxHeader, aData);
                break;
            }
            case 0x203: {
                motor::MotorLoad[1].decodeCanMsg(&RxHeader, aData);
                break;
            }
            default: {
                break;
            }
        }
    }
    if (hcan == &hcan2) {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, aData);
        switch (RxHeader.StdId) {
            case 0x208: {
                motor::MotorYawLS.decodeCanMsg(&RxHeader, aData);
                break;
            }
            default: {
                break;
            }
        }
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    static HAL_TIM_ActiveChannel last_channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    if (htim == &htim8) {
        switch (htim->Channel) {
            // 正式测镖速时应该将此处的通道号顺序改为实际使用的通道号顺序
            case HAL_TIM_ACTIVE_CHANNEL_1:
                if (last_channel == HAL_TIM_ACTIVE_CHANNEL_CLEARED) {
                    meter::velocity_meter.onCaptureGate1Rise(__HAL_TIM_GET_COMPARE(&htim8, TIM_CHANNEL_1));
                    last_channel = HAL_TIM_ACTIVE_CHANNEL_1;
                }
                break;
            case HAL_TIM_ACTIVE_CHANNEL_3:
                if (last_channel == HAL_TIM_ACTIVE_CHANNEL_1) {
                    meter::velocity_meter.onCaptureGate2Rise(__HAL_TIM_GET_COMPARE(&htim8, TIM_CHANNEL_3));
                    last_channel = HAL_TIM_ACTIVE_CHANNEL_3;
                }
                break;
            case HAL_TIM_ACTIVE_CHANNEL_2:
                if (last_channel == HAL_TIM_ACTIVE_CHANNEL_3) {
                    meter::velocity_meter.onCaptureGate1Fall(__HAL_TIM_GET_COMPARE(&htim8, TIM_CHANNEL_2));
                    last_channel = HAL_TIM_ACTIVE_CHANNEL_2;
                }
                break;
            case HAL_TIM_ACTIVE_CHANNEL_4:
                if (last_channel == HAL_TIM_ACTIVE_CHANNEL_2) {
                    meter::velocity_meter.onCaptureGate2Fall(__HAL_TIM_GET_COMPARE(&htim8, TIM_CHANNEL_4));
                    last_channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED; // 重置状态
                }
                break;
            default:
                last_channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED; // 未知通道，重置状态
                break;
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
    else if (htim->Instance == TIM6) {
        SoundEffectManager::timer_callback(&soundEffectManager);
    } else if (htim->Instance == TIM8) {
        meter::velocity_meter.onUpdate(htim);
    }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while (1) {
        // Restart the system
    }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
