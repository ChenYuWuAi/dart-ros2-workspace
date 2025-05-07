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
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <time.h>
#include <sys/types.h>
#include "chalie_leds.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum BMS_State
{
    BMS_STATE_INITIALIZING = 0,
    BMS_STATE_IDLE = 1,
    BMS_STATE_SLEEP = 2,
    BMS_STATE_KEY_FIRST_PRESS = 3,
    BMS_STATE_KEY_SHORT_PRESSED_ONCE = 4,
    BMS_STATE_KEY_SHORT_LONG_PRESS = 5,
    BMS_STATE_KEY_LONG_PRESS = 6
};

enum FET_State
{
    FET_STATE_OFF = 0,
    FET_STATE_ON
};

enum Light_State
{
    LIGHT_STATE_OFF = 0,
    LIGHT_STATE_BRIEF_DISPLAY,
    LIGHT_STATE_SHORT_LONG_PRESS,
    LIGHT_STATE_STEADY_DISPLAY,
    LIGHT_STATE_ERROR_DISPLAY
};

enum Shutdown_State
{
    SHUTDOWN_STATE_OFF = 0,
    SHUTDOWN_STATE_OK,
    SHUTDOWN_STATE_WAITING
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SOC_THRESHOLD_LOW    10    // 5% SOC critical low
#define VOLTAGE_THRESHOLD    7000 // 8V in mV for power on
#define KEY_LONG_PRESS_TIME  1000 // 1s for long press
#define Shutdown_Timeout     2000 // 20s for waiting for the RaspberryPi to power off
#define NO_LOAD_TIMEOUT      10000 // 10s timeout for 0 load
#define I2C_TIMEOUT          100  // I2C timeout in ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint is_shutdown = 0;
uint8_t display_active = 0; // 是否正在显示电量
uint8_t last_timer_it_state = 0;
uint8_t is_pressing = 0;
uint8_t is_status_new = 1;
HAL_StatusTypeDef smbus_status;

enum Shutdown_State shutdown = SHUTDOWN_STATE_OFF;
enum BMS_State current_bms_state = BMS_STATE_INITIALIZING;
enum FET_State current_fet_state = FET_STATE_OFF;
enum Light_State current_light_state = LIGHT_STATE_OFF;
uint8_t pres_state = 0; // 0: inactive, 1: active
uint8_t zero_load_state = 0; // 0: normal, 1: zero load
uint32_t zero_load_time = 0;
uint8_t first_time_zero_load = 0;
uint32_t key_press_start_time = 0;
uint32_t shutdown_start_time = 0;

uint8_t bq40z50_address = 0x0b;
uint8_t read = 0;
int16_t current = 0;
uint16_t voltage = 0;
uint8_t soc = 0;
uint16_t battery_status = 0;
int code = 0;

static uint8_t battery_status_BIN[16] = {0};
static uint8_t Manufacturing_Status_HEX[5] = {0};
static uint8_t Manufacturing_Status_BIN[16] = {0};
static uint8_t Safety_Alert_HEX[7] = {0};
static uint8_t Safety_Alert_BIN[32] = {0};
static uint8_t Operation_Status_HEX[7] = {0};
static uint8_t Operation_Status_BIN[32] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void BMS_StateMachine(void);

void UpdateBatteryStatus(void);

void UpdateLightState(void);

void UpdateFetState(uint8_t fet_enable, uint8_t dsg_enable, uint8_t chg_enable, uint8_t pchg_enable);

void CheckPRES(void);

void CheckZeroLoad(void);

void CheckFet(void);

void Enter_Standby_Mode(void);

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
    /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_I2C1_SMBUS_Init();
    MX_TIM21_Init();
    MX_RTC_Init();
    /* USER CODE BEGIN 2 */
    chalie_leds_init();
    HAL_SMBUS_EnableListen_IT(&hsmbus1);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        BMS_StateMachine();
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
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Configure LSE Drive Capability
    */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_RTC;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
/**
  * @brief 主状态机，管理BMS状态、FET和LED行为
  */
void BMS_StateMachine(void)
{
    // 检查按键状态
    uint8_t key_pressed = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) ? 1 : 0;

    // 存储一个临时的FET状态，作为后续的对比
    enum FET_State internal_fet_state = current_fet_state;

    // 每5000ms更新一次电池状态
    if (HAL_GetTick() % 5000 <= 2500 && !read && !key_pressed)
    {
        HAL_SMBUS_DisableListen_IT(&hsmbus1);
        UpdateBatteryStatus();
        HAL_SMBUS_EnableListen_IT(&hsmbus1);
        read = 1;
    }
    else if (HAL_GetTick() % 5000 >= 2500)
    {
        read = 0;
    }

    // 检查PRES状态和零负载是否超过10秒
    CheckPRES();
    CheckZeroLoad();

    switch (current_bms_state)
    {
    case BMS_STATE_INITIALIZING:
        // 初始化时尝试开启FET
        HAL_SMBUS_DisableListen_IT(&hsmbus1);
        UpdateFetState(0, 1, 1, 1);
        HAL_SMBUS_EnableListen_IT(&hsmbus1);
        current_bms_state = BMS_STATE_IDLE;
        current_fet_state = FET_STATE_ON;
        current_light_state = LIGHT_STATE_OFF;
        if (!pres_state)
        {
            current_fet_state = FET_STATE_ON;
        }
        break;

    case BMS_STATE_IDLE:
        current_light_state = LIGHT_STATE_OFF;
        if (current_fet_state == FET_STATE_ON)
        {
            current_light_state = LIGHT_STATE_STEADY_DISPLAY;
        }
        if (key_pressed)
        {
            current_bms_state = BMS_STATE_KEY_FIRST_PRESS;
            key_press_start_time = HAL_GetTick();
        }
    // 系统电源关闭条件
        // if (current_fet_state == FET_STATE_ON &&
        //     ((battery_status_BIN[9] && zero_load_state) || // 放电 & 0负载超过10s
        //         (battery_status_BIN[9] && soc <= SOC_THRESHOLD_LOW) || //  放电下电量低
        //         (!battery_status_BIN[9] && Safety_Alert_BIN[9])) // 充电下过充保护激活
        // )
        // {
        //     current_fet_state = FET_STATE_OFF;
        // }
        // if (current_fet_state == FET_STATE_ON && zero_load_state) //
        // {
        //     current_bms_state = BMS_STATE_SLEEP;
        // }
        break;

    case BMS_STATE_KEY_FIRST_PRESS:
        if (!key_pressed)
        {
            current_bms_state = BMS_STATE_KEY_SHORT_PRESSED_ONCE;
            key_press_start_time = HAL_GetTick();
        }
        break;

    case BMS_STATE_KEY_SHORT_PRESSED_ONCE:
        current_light_state = LIGHT_STATE_BRIEF_DISPLAY;
        if (key_pressed && (HAL_GetTick() - key_press_start_time < 2000))
        {
            key_press_start_time = HAL_GetTick();
            current_bms_state = BMS_STATE_KEY_SHORT_LONG_PRESS;
        }
        else if (!key_pressed && HAL_GetTick() - key_press_start_time >= 2000)
        {
            current_bms_state = BMS_STATE_IDLE;
        }
        break;

    case BMS_STATE_KEY_SHORT_LONG_PRESS:
        current_light_state = LIGHT_STATE_SHORT_LONG_PRESS;
        if (HAL_GetTick() - key_press_start_time >= KEY_LONG_PRESS_TIME && !key_pressed)
        {
            current_bms_state = BMS_STATE_KEY_LONG_PRESS;
        }
        else if (!key_pressed)
        {
            current_bms_state = BMS_STATE_KEY_SHORT_PRESSED_ONCE;
            key_press_start_time = HAL_GetTick();
        }
        break;

    case BMS_STATE_KEY_LONG_PRESS:
        if (!key_pressed)
        {
            current_bms_state = BMS_STATE_IDLE;
            if (current_fet_state == FET_STATE_ON)
            {
                // 若FET原本为开
                current_fet_state = FET_STATE_OFF; // 长按关闭FET
                current_light_state = LIGHT_STATE_OFF;
            }
            else
            {
                // 系统电源开启条件
                if (voltage >= VOLTAGE_THRESHOLD)
                {
                    // 如果电压大于阈值
                    current_fet_state = FET_STATE_ON;
                    current_light_state = LIGHT_STATE_STEADY_DISPLAY;
                }
            }
        }
        break;

    case BMS_STATE_SLEEP:
        shutdown_start_time = HAL_GetTick();
        if (!is_shutdown)
        {
            shutdown = SHUTDOWN_STATE_WAITING;
            is_shutdown = 1;
        }
        if (shutdown == SHUTDOWN_STATE_OK || HAL_GetTick() - shutdown_start_time >= Shutdown_Timeout)
        {
            is_shutdown = 0;
            shutdown = SHUTDOWN_STATE_OFF;
            current_fet_state = FET_STATE_OFF;
            Enter_Standby_Mode();
        }
        break;

    default:
        current_bms_state = BMS_STATE_INITIALIZING;
        break;
    }

    if (internal_fet_state != current_fet_state)
    {
        HAL_SMBUS_DisableListen_IT(&hsmbus1);
        if (!is_status_new)
        {
            UpdateBatteryStatus();
        }
        UpdateFetState(2, current_fet_state == FET_STATE_ON ? 1 : 0, current_fet_state == FET_STATE_ON ? 1 : 0, 1);
    }
    HAL_SMBUS_EnableListen_IT(&hsmbus1);
    UpdateLightState();
}

/**
  * @brief 更新电池状态
  */
void UpdateBatteryStatus(void)
{
    uint8_t Command = 0x44;
    uint16_t Safety_Alert = 0x0051;
    uint16_t Operation_Status = 0x0054;
    uint16_t Manufacturing_Status = 0x0057;
    uint8_t soc_address = 0x0d;
    uint8_t current_address = 0x0a;
    uint8_t voltage_address = 0x09;
    uint8_t battery_status_address = 0x16;

    // 更新soc,current,voltage的值
    smbus_status = HAL_SMBUS_Master_Transmit(&hsmbus1, bq40z50_address << 1, &soc_address, 1, SMBUS_FIRST_FRAME,
                                             I2C_TIMEOUT);
    if (smbus_status == HAL_TIMEOUT)
    {
        MX_I2C1_SMBUS_Init();
        HAL_SMBUS_Master_Transmit(&hsmbus1, bq40z50_address << 1, &soc_address, 1, SMBUS_FIRST_FRAME, I2C_TIMEOUT);
    }
    HAL_SMBUS_Master_Receive(&hsmbus1, bq40z50_address << 1, (uint8_t*)&soc, 1, SMBUS_LAST_FRAME_NO_PEC, I2C_TIMEOUT);
    HAL_SMBUS_Master_Transmit(&hsmbus1, bq40z50_address << 1, &current_address, 1, SMBUS_FIRST_FRAME, I2C_TIMEOUT);
    HAL_SMBUS_Master_Receive(&hsmbus1, bq40z50_address << 1, (uint8_t*)&current, 2, SMBUS_LAST_FRAME_NO_PEC,
                             I2C_TIMEOUT);
    HAL_SMBUS_Master_Transmit(&hsmbus1, bq40z50_address << 1, &voltage_address, 1, SMBUS_FIRST_FRAME, I2C_TIMEOUT);
    HAL_SMBUS_Master_Receive(&hsmbus1, bq40z50_address << 1, (uint8_t*)&voltage, 2, SMBUS_LAST_FRAME_NO_PEC,
                             I2C_TIMEOUT);
    code = (soc - 1) / 12.5;

    // 更新Battery Status
    HAL_SMBUS_Master_Transmit(&hsmbus1, bq40z50_address << 1, &battery_status_address, 1, SMBUS_FIRST_FRAME,
                              I2C_TIMEOUT);
    HAL_SMBUS_Master_Receive(&hsmbus1, bq40z50_address << 1, (uint8_t*)&battery_status, 2, SMBUS_LAST_FRAME_NO_PEC,
                             I2C_TIMEOUT);
    for (int i = 0; i < 16; i++)
    {
        battery_status_BIN[i] = (battery_status & (1 << (15 - i))) ? 1 : 0;
    }

    // 更新Manufacture Status
    ManufacturerBlockAccess_write(Manufacturing_Status);
    HAL_SMBUS_Master_Transmit(&hsmbus1, bq40z50_address << 1, &Command, 1, SMBUS_LAST_FRAME_NO_PEC, I2C_TIMEOUT);
    HAL_SMBUS_Master_Receive(&hsmbus1, bq40z50_address << 1, Manufacturing_Status_HEX, 5, SMBUS_LAST_FRAME_NO_PEC,
                             I2C_TIMEOUT);
    uint16_t value_1 = Manufacturing_Status_HEX[4] << 8 | Manufacturing_Status_HEX[3];
    for (int i = 0; i < 16; i++)
    {
        Manufacturing_Status_BIN[i] = (value_1 & (1 << (15 - i))) ? 1 : 0;
    }

    // 更新Safety Alert
    ManufacturerBlockAccess_write(Safety_Alert);
    HAL_SMBUS_Master_Transmit(&hsmbus1, bq40z50_address << 1, &Command, 1, SMBUS_LAST_FRAME_NO_PEC, I2C_TIMEOUT);
    HAL_SMBUS_Master_Receive(&hsmbus1, bq40z50_address << 1, Safety_Alert_HEX, 7, SMBUS_LAST_FRAME_NO_PEC, I2C_TIMEOUT);
    uint32_t value_2 = Safety_Alert_HEX[6] << 24 | Safety_Alert_HEX[5] << 16 | Safety_Alert_HEX[4] << 8 |
        Safety_Alert_HEX[3];
    for (int i = 0; i < 32; i++)
    {
        Safety_Alert_BIN[i] = (value_2 & (1U << (31 - i))) ? 1 : 0;
    }

    // 更新Operation Status
    ManufacturerBlockAccess_write(Operation_Status);
    HAL_SMBUS_Master_Transmit(&hsmbus1, bq40z50_address << 1, &Command, 1, SMBUS_LAST_FRAME_NO_PEC, I2C_TIMEOUT);
    HAL_SMBUS_Master_Receive(&hsmbus1, bq40z50_address << 1, Operation_Status_HEX, 7, SMBUS_LAST_FRAME_NO_PEC,
                             I2C_TIMEOUT);
    uint32_t value_3 = Operation_Status_HEX[6] << 24 | Operation_Status_HEX[5] << 16 | Operation_Status_HEX[4] << 8 |
        Operation_Status_HEX[3];
    for (int i = 0; i < 32; i++)
    {
        Operation_Status_BIN[i] = (value_3 & (1U << (31 - i))) ? 1 : 0;
    }

    CheckFet();
    is_status_new = 1;
}

/**
  * @brief 控制FET状态
  */
void UpdateFetState(uint8_t fet_enable, uint8_t dsg_enable, uint8_t chg_enable, uint8_t pchg_enable)
{
    uint16_t FET_Control = 0x0022;
    uint16_t DSG_FET_Toggle = 0x0020;
    uint16_t CHG_FET_Toggle = 0x001f;
    uint16_t PCHG_FET_Toggle = 0x001e;

    if ((Manufacturing_Status_BIN[11] == 0 && fet_enable == 1) || (
        Manufacturing_Status_BIN[11] == 1 && fet_enable == 0))
    {
        ManufacturerBlockAccess_write(FET_Control);
    }
    if ((Manufacturing_Status_BIN[13] == 0 && dsg_enable == 1) || (
        Manufacturing_Status_BIN[13] == 1 && dsg_enable == 0))
    {
        ManufacturerBlockAccess_write(DSG_FET_Toggle);
    }
    if ((Manufacturing_Status_BIN[14] == 0 && chg_enable == 1) || (
        Manufacturing_Status_BIN[14] == 1 && chg_enable == 0))
    {
        ManufacturerBlockAccess_write(CHG_FET_Toggle);
    }
    if ((Manufacturing_Status_BIN[15] == 0 && pchg_enable == 1) || (
        Manufacturing_Status_BIN[15] == 1 && pchg_enable == 0))
    {
        ManufacturerBlockAccess_write(PCHG_FET_Toggle);
    }

    is_status_new = 0;
}

/**
  * @brief 更新LED状态
  */
void UpdateLightState(void)
{
    switch (current_light_state)
    {
    case LIGHT_STATE_OFF:
        if (last_timer_it_state)
        {
            HAL_TIM_Base_Stop_IT(&htim2);
            last_timer_it_state = 0;
        }
        chalie_led_timer_update(1);
        chalie_leds_set(1, GPIO_PIN_RESET);
        break;

    case LIGHT_STATE_BRIEF_DISPLAY:
        if (!last_timer_it_state)
        {
            HAL_TIM_Base_Start_IT(&htim2);
            last_timer_it_state = 1;
            chalie_leds_set(1, GPIO_PIN_RESET);
        }
        chalie_led_code(code);
        break;

    case LIGHT_STATE_SHORT_LONG_PRESS:
        if (!last_timer_it_state)
        {
            HAL_TIM_Base_Start_IT(&htim2);
            last_timer_it_state = 1;
            chalie_leds_set(1, GPIO_PIN_RESET);
        }
        chalie_led_code((HAL_GetTick() - key_press_start_time) / 250 * 2 + 1);
        break;

    case LIGHT_STATE_STEADY_DISPLAY:
        if (!last_timer_it_state)
        {
            HAL_TIM_Base_Start_IT(&htim2);
            last_timer_it_state = 1;
        }
        chalie_led_code(code);
        break;

    case LIGHT_STATE_ERROR_DISPLAY:
        if (!last_timer_it_state)
        {
            HAL_TIM_Base_Start_IT(&htim2);
            last_timer_it_state = 1;
        }
        if (HAL_GetTick() % 1000 >= 500)
        {
            chalie_led_code(7);
        }
        else
        {
            chalie_led_code(0);
        }
        break;
    }
}

/**
  * @brief 检查PRES状态
  */
void CheckPRES(void)
{
    pres_state = Operation_Status_BIN[31];
}

/**
  * @brief 检查零负载状态
  */
void CheckZeroLoad(void)
{
    if (current > -10 && current < 10)
    {
        if (first_time_zero_load == 0)
        {
            first_time_zero_load = 1;
            zero_load_time = HAL_GetTick();
        }
        else if (HAL_GetTick() - zero_load_time > NO_LOAD_TIMEOUT)
        {
            zero_load_state = 1;
            first_time_zero_load = 0;
        }
    }
    else
    {
        zero_load_state = 0;
        first_time_zero_load = 0;
    }
}

/**
  * @brief 检查FET状态
  */
void CheckFet(void)
{
    if (Manufacturing_Status_BIN[13] == 1 && Manufacturing_Status_BIN[14] == 1)
    {
        current_fet_state = FET_STATE_ON;
    }
    else
    {
        current_fet_state = FET_STATE_OFF;
    }
}

/**
  * @brief 在RTC中断唤醒后恢复时间
  */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef* hrtc)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    // char str[200] = {0};

    // 获取时间
    if (HAL_RTC_GetTime(hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
    {
        return;
    }

    // 获取日期
    if (HAL_RTC_GetDate(hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
    {
        return;
    }

    HAL_RTCEx_BKUPWrite(hrtc, RTC_BKP_DR1, 0x01);
    HAL_RTCEx_BKUPWrite(hrtc, RTC_BKP_DR2, sTime.Hours);
    HAL_RTCEx_BKUPWrite(hrtc, RTC_BKP_DR3, sTime.Minutes);
    HAL_RTCEx_BKUPWrite(hrtc, RTC_BKP_DR4, sTime.Seconds);

    chalie_leds_set(1, GPIO_PIN_SET); // 打开LED，指示设备被唤醒
    HAL_Delay(1000); // 延迟1秒后关闭LED
    chalie_leds_set(1, GPIO_PIN_RESET);
    current_bms_state = BMS_STATE_INITIALIZING;
}

/**
  * @brief 进入待机模式
  */
void Enter_Standby_Mode(void)
{
    __HAL_RCC_PWR_CLK_ENABLE(); // 使能 PWR 时钟
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU); // 清除唤醒标志

    // 配置 RTC 唤醒定时器，实现 10 秒钟唤醒
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 9, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
    {
        Error_Handler();
    }

    // 进入待机模式
    HAL_PWR_EnterSTANDBYMode();
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
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