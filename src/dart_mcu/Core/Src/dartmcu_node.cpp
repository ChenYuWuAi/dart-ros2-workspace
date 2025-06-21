//
// Created by cheny on 24-9-10.
//
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "usb_device.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcutils/time.h>
#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>
#include <cstring>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <buzzer.h>
#include "buzzer_examples.h"
#include "dartmcu_node.h"

#include "sound_effect.h"
#include "tim.h"

#include "led.h"
#include "velocimeter.h"
#include "servo.h"

#include "state_machine.h"

#include "motor_controller.h"

#include <dart_config.h>

#include <dart_launcher_default_value.hpp>

#include <atomic>  // 用于原子操作

// 日志静态环形队列配置
#define LOG_QUEUE_SIZE 10
static char logQueueBuf[LOG_QUEUE_SIZE][LOG_BUF_LEN];
static std::atomic<uint8_t> logQueueHead{0};
static std::atomic<uint8_t> logQueueTail{0};
static std::atomic<uint8_t> logQueueCount{0};

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

// MicroROS 实体
rcl_allocator_t allocator;
rcl_subscription_t subscriber_buzzer = rcl_get_zero_initialized_subscription();
rcl_subscription_t subscriber_protocol = rcl_get_zero_initialized_subscription();
rcl_subscription_t subscriber_parameter = rcl_get_zero_initialized_subscription();
rcl_subscription_t subscriber_greenlight = rcl_get_zero_initialized_subscription();
rcl_publisher_t publisher_logger = rcl_get_zero_initialized_publisher();
rcl_publisher_t publisher_status = rcl_get_zero_initialized_publisher();
rcl_node_t node;
rclc_support_t support;
rcl_timer_t timer_log_update, timer_status_update;

rclc_executor_t executor;
std_msgs__msg__Int32 msgInt32;
std_msgs__msg__String msgString;

char msgString_buf[LOG_BUF_LEN];

velocity_meter_result_t velocity_meter_result;

TickType_t last_greenlight_update_time = 0;

dart_msgs__msg__GreenLight msgGreenLight;
dart_msgs__msg__DartLauncherParams msgDartParams;

dart_msgs__msg__DartLauncherParams msgDartProtocols;

dart_msgs__msg__DartLauncherStatus msgDartStatus = defaultDartStatus;

extern "C"
{
void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);
}

void microros_node_task(void) {
    defaultDartParams(msgDartParams);
    defaultDartProtocols(msgDartProtocols);
    msgDartStatus.params = msgDartParams;
    msgDartStatus.protocols = msgDartProtocols;

    soundEffectManager.begin(&htim2, &htim6, TIM_CHANNEL_4, HAL_RCC_GetPCLK2Freq());
    LED::led_flow.begin();
    trigger_servo[0].begin(&htim4, TIM_CHANNEL_1, HAL_RCC_GetPCLK2Freq(), 500, 2500, 0, 180, 10000, 100,
                           CONFIG_TRIGGER_SERVO_RELOAD_ANGLE_0);
    trigger_servo[1].begin(&htim5, TIM_CHANNEL_3, HAL_RCC_GetPCLK2Freq(), 500, 2500, 0, 180, 10000, 100,
                           CONFIG_TRIGGER_SERVO_RELOAD_ANGLE_1);
    trigger_servo[2].begin(&htim4, TIM_CHANNEL_3, HAL_RCC_GetPCLK2Freq(), 500, 2500, 0, 270, 10000, 100,
                           CONFIG_LOAD_SERVO_UP_ANGLE_0);
    trigger_servo[3].begin(&htim4, TIM_CHANNEL_4, HAL_RCC_GetPCLK2Freq(), 500, 2500, 0, 270, 10000, 100,
                           CONFIG_LOAD_SERVO_UP_ANGLE_0);
    trigger_servo[4].begin(&htim5, TIM_CHANNEL_1, HAL_RCC_GetPCLK2Freq(), 500, 2500, 0, 270, 10000, 100,
                           CONFIG_LOAD_SERVO_UP_ANGLE_1);
    trigger_servo[5].begin(&htim5, TIM_CHANNEL_2, HAL_RCC_GetPCLK2Freq(), 500, 2500, 0, 270, 10000, 100,
                           CONFIG_LOAD_SERVO_UP_ANGLE_1);
    trigger_servo[6].begin(&htim5, TIM_CHANNEL_4, HAL_RCC_GetPCLK2Freq(), 500, 2500, 0, 270, 10000, 100,
                           CONFIG_SLIDE_SERVO_CUT_ANGLE);


    meter::velocity_meter.begin(&htim8, TIM_CHANNEL_1, &htim8, TIM_CHANNEL_2, 65536, [=](float velocity) {
        velocity_meter_result.velocity = velocity;
        velocity_meter_result.record_time = xTaskGetTickCount();
        msgDartStatus.last_launch_speed = velocity_meter_result.velocity;
    }, 0.139, 0.0000005);

    xTaskCreate(state_machine::fsm_thread, "fsm_thread", 1024, NULL, 11, NULL);

    xTaskCreate(motor_controller::pid_control_task, "pid_control_task", 256, NULL, 25, NULL);

    set_ros_transport();
    state = WAITING_AGENT;

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_warn));
    }

    while (1) {
        switch (state) {
            case WAITING_AGENT:
                EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(50, 1)) ? AGENT_AVAILABLE
                                                                                            : WAITING_AGENT;);

                // 每10s重启一次USB
                EXECUTE_EVERY_N_MS(10000, {
                    USB_DEVICE_Stop();
                    vTaskDelay(200);
                    USB_DEVICE_Start();
                });
                break;
            case AGENT_AVAILABLE:
                state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
                if (state == WAITING_AGENT) {
                    destroy_entities();
                } else if (state == AGENT_CONNECTED) {
                    soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_plug_in), false, false);
                    LED::setLED(
                            LED::LED_GREEN, true);
                    LED::led_flow.flow_state_ = LED::LED_Flow_State::FLOW_NORMAL;
                    // 同步时间
                    rmw_uros_sync_session(1000);
                }
                break;
            case AGENT_CONNECTED:
                EXECUTE_EVERY_N_MS(3000,
                                   state = (RMW_RET_OK == rmw_uros_ping_agent(100, 5)) ? AGENT_CONNECTED
                                                                                       : AGENT_DISCONNECTED;);
                if (state == AGENT_CONNECTED) {
                    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1000));
                    // 每60s同步时间
                    EXECUTE_EVERY_N_MS(60000, {
                        rmw_uros_sync_session(1000);
                    });
                } else if (state == AGENT_DISCONNECTED) {
                    soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_remove), false, false);
                    LED::setLED(LED::LED_GREEN, false);
                    LED::led_flow.flow_state_ = LED::LED_Flow_State::FLOW_NONE;
                }
                break;
            case AGENT_DISCONNECTED:
                destroy_entities();
                state = WAITING_AGENT;
                break;
            default:
                break;
        }
        vTaskDelay(2);
    }
};

void timer_logger_callback(rcl_timer_t *timer, int64_t last_call_time) {
// 等待信号量，直到有新日志
    (void) last_call_time;
    if (!timer)
        return;

    // 从静态环形队列取出消息
    if (logQueueCount.load() > 0) {
        uint8_t head = logQueueHead.load();
        char *pMsg = logQueueBuf[head];
        if (pMsg != nullptr) {
            size_t len = strlen(pMsg);
            if (len >= msgString.data.capacity) {
                len = msgString.data.capacity - 1;
            }
            memcpy(msgString.data.data, pMsg, len);
            msgString.data.data[len] = '\0';  // 确保字符串正确终止
            msgString.data.size = len + 1;

            // 发布消息
            rcl_publish(&publisher_logger, &msgString, nullptr);
        }
        // 更新环形队列指针和计数
        logQueueHead.store((uint8_t) ((head + 1) % LOG_QUEUE_SIZE));
        logQueueCount.fetch_sub(1);
    }
}

void timer_send_status_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != nullptr) {
        // 序列化镖架状态变量发送
        // 发送velocity 填充last_launch_time
        static TickType_t last_send_tick = velocity_meter_result.record_time;
        if (last_send_tick != velocity_meter_result.record_time) {
            dart_mcu_log("velocity: %.2f", velocity_meter_result.velocity);
            last_send_tick = velocity_meter_result.record_time;
            msgDartStatus.last_launch_time = rmw_uros_epoch_millis();
        }
        msgDartStatus.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
        msgDartStatus.header.stamp.nanosec = rmw_uros_epoch_nanos() % 1000000000;
        rcl_publish(&publisher_status, &msgDartStatus, nullptr);
    }
}

bool create_entities() {

    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "dart_mcu", "", &support));

    // create publisher
    rclc_publisher_init_default(
            &publisher_logger,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/dart_launcher_mcu/log");

    rclc_publisher_init_best_effort(
            &publisher_status,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(dart_msgs, msg, DartLauncherStatus),
            "/dart_launcher_mcu/status");

    // subscribe to /buzzer/cmd_note and /buzzer/cmd_sound_effect
    // create subscriber
    RCSOFTCHECK(rclc_subscription_init_default(
            &subscriber_buzzer,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "/dart_launcher_mcu/cmd_sound_effect"))

    RCSOFTCHECK(rclc_subscription_init_default(
            &subscriber_protocol,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(dart_msgs, msg, DartLauncherParams),
            "/dart_launcher_mcu/cmd_protocols"));

    RCSOFTCHECK(rclc_subscription_init_default(
            &subscriber_parameter,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(dart_msgs, msg, DartLauncherParams),
            "/dart_launcher_mcu/cmd_params"))

    RCSOFTCHECK(rclc_subscription_init_best_effort(
            &subscriber_greenlight,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(dart_msgs, msg, GreenLight),
            "/dart_launcher_detector/results/greenlight"))

    // create timer,
    const unsigned int timer_logger_timeout = 100;
    RCCHECK(rclc_timer_init_default2(
            &timer_log_update,
            &support,
            RCL_MS_TO_NS(timer_logger_timeout),
            timer_logger_callback, true));

    const unsigned int timer2_timeout = 100;
    RCCHECK(rclc_timer_init_default2(
            &timer_status_update,
            &support,
            RCL_MS_TO_NS(timer2_timeout),
            timer_send_status_callback, true));

    // 初始化消息
    msgString.data.capacity = LOG_BUF_LEN;
    msgString.data.data = msgString_buf;
    msgString.data.size = 0;

    // create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_log_update));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_status_update));
    RCSOFTCHECK(rclc_executor_add_subscription(&executor, &subscriber_buzzer, &msgInt32,
                                               &subscription_buzzer_callback,
                                               ON_NEW_DATA));
    RCSOFTCHECK(rclc_executor_add_subscription(&executor, &subscriber_protocol, &msgDartProtocols,
                                               &subscription_protocol_setting_callback,
                                               ON_NEW_DATA));
    RCSOFTCHECK(rclc_executor_add_subscription(&executor, &subscriber_parameter, &msgDartParams,
                                               &subscription_parameter_setting_callback,
                                               ON_NEW_DATA));
    RCSOFTCHECK(rclc_executor_add_subscription(&executor, &subscriber_greenlight, &msgGreenLight,
                                               &subscription_greenlight_callback,
                                               ON_NEW_DATA));

    return true;
}

void destroy_entities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&publisher_logger, &node);
    rcl_publisher_fini(&publisher_status, &node);
    rcl_timer_fini(&timer_log_update);
    rcl_timer_fini(&timer_status_update);
    rcl_subscription_fini(&subscriber_buzzer, &node);
    rcl_subscription_fini(&subscriber_protocol, &node);
    rcl_subscription_fini(&subscriber_parameter, &node);
    rcl_subscription_fini(&subscriber_greenlight, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    // 重新初始化USB设备
    // 断联
    USB_DEVICE_Stop();
    vTaskDelay(200);
    USB_DEVICE_Start();
}


void choose_sound_effect(int index) {
    switch (index) {
        case BuzzerSound::BuzzerAutopilotDisconnect:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_autopilot_disconnect));
            break;
        case BuzzerSound::BuzzerLaunch:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_launch));
            break;
        case BuzzerSound::BuzzerHaru:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_haru));
            break;
        case BuzzerSound::BuzzerDjiStartup:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_dji_startup));
            break;
        case BuzzerSound::BuzzerWinxp:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_winxp));
            break;
        case BuzzerSound::BuzzerApproach:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_approach));
            break;
        case BuzzerSound::BuzzerLaoda:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_laoda));
            break;
        case BuzzerSound::BuzzerStartup:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_startup));
            break;
        case BuzzerSound::BuzzerPlugIn:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_plug_in));
            break;
        case BuzzerSound::BuzzerRemove:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_remove));
            break;
        case BuzzerSound::BuzzerError:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_error));
            break;
        case BuzzerSound::BuzzerWinxpLogout:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_protect));
            break;
        case BuzzerSound::BuzzerChunriying:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_chunriying));
            break;
        case BuzzerSound::BuzzerIfICouldBeAConstelletion:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_if_i_could_be_a_constelletion));
            break;
        case BuzzerSound::BuzzerGuitarLonelinessBlueEarth:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_guitar_loneliness_blue_earth));
            break;
        case BuzzerSound::BuzzerNeverForget:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_never_forget));
            break;
        case BuzzerSound::BuzzerBokuranomachi:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_bokuranomachi));
            break;
        case BuzzerSound::BuzzerBadApple:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_bad_apple));
            break;
        case BuzzerSound::BuzzerWarn:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_warn));
            break;
        case BuzzerSound::BuzzerDontSayLazy:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_dont_say_lazy));
            break;
        case BuzzerSound::BuzzerInternetOverdose:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_internet_overdose));
            break;
        case BuzzerSound::BuzzerInternetAngel:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_internet_angel));
            break;
        case BuzzerSound::BuzzerIfICouldBeAConstelletionPlus:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_if_i_could_be_a_constelletion_plus));
            break;
        case BuzzerSound::BuzzerWin10PlugIn:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_win10_plug_in));
            break;
        case BuzzerSound::BuzzerWin10Remove:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_win10_remove));
            break;
        default:
            soundEffectManager.stopCurrentSoundEffect();
            break;
    }
}

void subscription_buzzer_callback(const void *msgin) {
    const auto *msg = (const std_msgs__msg__Int32 *) msgin;
    if (msgin != NULL) {
        // 播放音效
        choose_sound_effect(msg->data);
    }
}

void subscription_protocol_setting_callback(const void *msgin) {
    const dart_msgs__msg__DartLauncherParams *msg = (const dart_msgs__msg__DartLauncherParams *) msgin;

    if (msgin != NULL) {
    }
}

void subscription_parameter_setting_callback(const void *msgin) {
    const auto *msg = (const dart_msgs__msg__DartLauncherParams *) msgin;
    if (msgin != NULL) {
    }
}

void subscription_greenlight_callback(const void *msgin) {
    if (msgin != NULL) {
        // 更新时间戳
        last_greenlight_update_time = xTaskGetTickCount();
    }
}

// 日志接口：中断中不可调用, 将格式化消息写入静态环形队列
void dart_mcu_log(const char *fmt, ...) {
    if (logQueueCount.load() >= LOG_QUEUE_SIZE) {
        return; // 队列已满丢弃
    }
    uint8_t tail = logQueueTail.load();
    char *slot = logQueueBuf[tail];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(slot, LOG_BUF_LEN, fmt, args);
    va_end(args);
    if (len < 0) {
        slot[0] = '\0';
    } else if (len >= LOG_BUF_LEN) {
        slot[LOG_BUF_LEN - 1] = '\0';
    }
    logQueueTail.store((uint8_t) ((tail + 1) % LOG_QUEUE_SIZE));
    logQueueCount.fetch_add(1);
}