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
#include <buzzer.h>
#include "buzzer_examples.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_autopilot_disconnect));}}

void microros_node_task(void);

bool create_entities();

void set_ros_transport();

void destroy_entities();

void timer_callback(rcl_timer_t *timer, int64_t last_call_time);

void subscription_buzzer_callback(const void *msgin);

void subscription_servo_callback(const void *msgin);

extern rcl_allocator_t allocator;
extern rcl_publisher_t publisher;
extern rcl_node_t node;
extern rclc_support_t support;
extern rcl_timer_t timer;
extern rclc_executor_t executor;
extern std_msgs__msg__Int64 msg;
extern bool micro_ros_init_successful;

#endif //DART_MCU_DARTMCU_NODE_H
