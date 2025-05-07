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


    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, judge_rx_buffer[0], UART3_MAX_RECEIVE_BUFFER_LENGTH);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1RecBuffer, UART1_MAX_RECEIVE_BUFFER_LENGTH);
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
/*---------------------------------function of interrupt begin------------------------------------*/
// 遥控&裁判系统解码
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == &huart1 && huart->RxEventType == HAL_UART_RXEVENT_IDLE
            ) {
        if (Size == 18U) {
            memcpy(RC_rx_buffer, uart1RecBuffer, Size);
            DT7_Decode();
        }

        memset(uart1RecBuffer, 0, UART1_MAX_RECEIVE_BUFFER_LENGTH);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1RecBuffer, UART1_MAX_RECEIVE_BUFFER_LENGTH);
    } else if (huart == &huart3 && huart->RxEventType == HAL_UART_RXEVENT_IDLE
            ) {
        // 双缓�?
        static uint8_t decode_memory_ = MEMORY0;

        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, judge_rx_buffer[(decode_memory_ + 1) % 2],
                                     UART3_MAX_RECEIVE_BUFFER_LENGTH);
        RefereeReceive(Size, judge_rx_buffer[decode_memory_]
        );
        decode_memory_ = (decode_memory_ + 1) % 2;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1 || huart == &huart3) {
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
        __HAL_UART_ENABLE_IT(huart, UART_IT_ERR);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->gState = HAL_UART_STATE_READY;
        huart->RxState = HAL_UART_STATE_READY;
    }
}

/**
 * @brief 在此函数中添加自动恢复逻辑
 */

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    uint32_t err = HAL_CAN_GetError(hcan);  // 获取错误码 :contentReference[oaicite:6]{index=6}

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
    static bool begin = false;
    if (htim == &htim8) {
        if (!begin && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
            meter::velocity_meter.onCaptureBegin(htim->Instance->CCR1);
            begin = true;
        } else if (begin && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
            meter::velocity_meter.onCaptureEnd(htim->Instance->CCR2);
            begin = false;
        } else {
            begin = false;
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
    __disable_irq();
    while (1) {
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
