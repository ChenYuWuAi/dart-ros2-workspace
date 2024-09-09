/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcutils/time.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int64.h>
#include <std_msgs/msg/string.h>
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport *transport);

bool cubemx_transport_close(struct uxrCustomTransport *transport);

size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);

size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

void *microros_allocate(size_t size, void *state);

void microros_deallocate(void *pointer, void *state);

void *microros_reallocate(void *pointer, size_t size, void *state);

void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */

    bool first_connection = true;
    // micro-ROS configuration
    rmw_uros_set_custom_transport(
            true,
            (void *) NULL,
            cubemx_transport_open,
            cubemx_transport_close,
            cubemx_transport_write,
            cubemx_transport_read);
    rcl_allocator_t allocator;
    allocator = rcl_get_default_allocator();
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;
    while (1) {
        rcl_publisher_t publisher;
        std_msgs__msg__Int64 msg;
        rcl_node_t node;
        rclc_support_t support;
        if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
            printf("Error on default allocators (line %d)\n", __LINE__);
        }

        // micro-ROS app

        //create init_options
        rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
        rcl_init_options_init(&init_options, allocator);
        rcl_init_options_set_domain_id(&init_options, 7);
        rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
        rcl_init_options_fini(&init_options);

        // create node
        rcl_ret_t ret;
        ret = rclc_node_init_default(&node, "cubemx_node", "", &support);
        if (ret != RCL_RET_OK) {
            printf("Error on node init (line %d)\n", __LINE__);
            HAL_NVIC_SystemReset();
        }


        // create publisher
        rclc_publisher_init_default(
                &publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
                "cubemx_publisher");

        msg.data = 0;

        for (;;) {
            rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
            if (ret != RCL_RET_OK) {
                printf("Error publishing (line %d)\n", __LINE__);
            }
            // 获取当前时间
//        rcl_time_point_value_t now;
//        rcl_clock_get_now(&support.clock, &now);
            // 同步时间�?
//            static int error_count = 0;
//            rmw_ret_t ping_ret = rmw_uros_ping_agent(100, 5);
//            if (ping_ret != RMW_RET_OK && first_connection == false) {
//                printf("Error sync session (line %d)\n", __LINE__);
//
//                // 重新连接Session
//                printf("Reconnect session (line %d)\n", __LINE__);
////                    HAL_NVIC_SystemReset();
//                // fini
//                rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
//                (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
//                rcl_publisher_fini(&publisher, &node);
//                rcl_node_fini(&node);
//                rclc_support_fini(&support);
//                break;
//            } else if (first_connection == true && ping_ret == RMW_RET_OK) {
//                first_connection = false;
//            } else
//                error_count = 0;


            msg.data = rmw_uros_epoch_millis();
            osDelay(1000);
        }
        // 重新连接Session
        printf("Reconnect session (line %d)\n", __LINE__);
    }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

