//
// Created by cheny on 24-9-15.
//
// 基于Input Capture的初速度计算器

#ifndef DART_MCU_VELOCIMETER_H
#define DART_MCU_VELOCIMETER_H

#include "tim.h"
#include "stm32f4xx_hal.h"
#include <functional>

namespace meter {
    enum velocimeter_state {
        IDLE,
        CONTINOUS,
        ONESHOT,
        MEASURING
    };

    class velocimeter {
    private:
        TIM_HandleTypeDef *htim_begin;
        velocimeter_state state = IDLE;
        velocimeter_state prev_state = IDLE;
        uint32_t refresh_counter_timer;
        uint32_t update_count_begin;
        uint32_t update_count_end;

        uint32_t timer_period;

        std::function<void(float)> onVelocityUpdate;
        double distanceBetweenTwoPulse;
        double seconds_per_tick = 0.000001; // 1us/tick

        // 双光电门三平均算法相关
        // 光电门1
        uint32_t gate1_rise_tick, gate1_fall_tick;
        uint32_t gate1_rise_overflow, gate1_fall_overflow;
        // 光电门2
        uint32_t gate2_rise_tick, gate2_fall_tick;
        uint32_t gate2_rise_overflow, gate2_fall_overflow;
        // 状态
        bool gate1_rise_valid, gate1_fall_valid;
        bool gate2_rise_valid, gate2_fall_valid;

        double object_length; // 测量物体长度

    public:
        velocimeter() = default;

        void begin(
            TIM_HandleTypeDef *htim,
            uint32_t channel_begin_rise, // 光电门1上升沿通道
            uint32_t channel_begin_fall, // 光电门1下降沿通道
            uint32_t channel_end_rise, // 光电门2上升沿通道
            uint32_t channel_end_fall, // 光电门2下降沿通道
            uint32_t timer_period,
            std::function<void(float)> onVelocityUpdate,
            double distanceBetweenTwoPulse,
            double object_length = 0,
            double seconds_per_tick = 0.000001
        );

        void onUpdate(TIM_HandleTypeDef *htim);

        // 四个通道捕获处理
        void onCaptureGate1Rise(uint32_t count);
        void onCaptureGate1Fall(uint32_t count);
        void onCaptureGate2Rise(uint32_t count);
        void onCaptureGate2Fall(uint32_t count);

        void reset();

        void disable();

        // 四通道号
        uint32_t channel_gate1_rise;
        uint32_t channel_gate1_fall;
        uint32_t channel_gate2_rise;
        uint32_t channel_gate2_fall;

        void enable(bool oneshot = true);
    };

    // 全局变量
    extern velocimeter velocity_meter;
}

#endif //DART_MCU_VELOCIMETER_H
