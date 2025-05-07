//
// Created by cheny on 24-9-10.
//

#ifndef DART_MCU_DARTMCU_NODE_H
#define DART_MCU_DARTMCU_NODE_H

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
#include <buzzer.h>
#include "buzzer_examples.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include <dart_msgs/msg/dart_launcher_params.h>
#include <dart_msgs/msg/dart_launcher_status.h>
#include <dart_msgs/msg/green_light.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_error));return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ \
soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_error));                   \
return false; }}

typedef struct velocity_meter_result_t {
    double velocity;
    TickType_t record_time;
} velocity_meter_result_t;

extern velocity_meter_result_t velocity_meter_result;

bool create_entities();

void destroy_entities();

void timer_logger_callback(rcl_timer_t *timer, int64_t last_call_time);

void subscription_buzzer_callback(const void *msgin);

void subscription_protocol_setting_callback(const void *msgin);

void subscription_parameter_setting_callback(const void *msgin);

void subscription_greenlight_callback(const void *msgin);

void choose_sound_effect(int index);

// --- 日志函数原型 ---
void dart_mcu_log(const char *msg);

extern rcl_allocator_t allocator;
extern rcl_publisher_t publisher_logger;
extern rcl_node_t node;
extern rclc_support_t support;
extern rcl_timer_t timer_log_update;
extern rclc_executor_t executor;
extern std_msgs__msg__Int64 msgInt64;
extern std_msgs__msg__String msgString;

extern TickType_t last_greenlight_update_time;

#endif //DART_MCU_DARTMCU_NODE_H
