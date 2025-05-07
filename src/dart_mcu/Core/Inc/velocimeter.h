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
        uint32_t begin_time;
        uint32_t end_time;

        uint32_t timer_period;

        std::function<void(float)> onVelocityUpdate;
        double distanceBetweenTwoPulse;
        double seconds_per_tick = 0.000001; // 1us/tick

    public:
        velocimeter() = default;

        void
        begin(TIM_HandleTypeDef *htim_begin, uint32_t channel_begin, TIM_HandleTypeDef *htim_end, uint32_t channel_end,
              uint32_t timer_period, std::function<void(float)> onVelocityUpdate, double distanceBetweenTwoPulse,
              double seconds_per_tick);

        void onUpdate(TIM_HandleTypeDef *htim);

        void onCaptureBegin(uint32_t count);

        void onCaptureEnd(uint32_t count);

        void reset();

        void disable();

        uint32_t channel_begin;
        uint32_t channel_end;

        void enable(bool oneshot = true);
    };

    // 全局变量
    extern velocimeter velocity_meter;
}

#endif //DART_MCU_VELOCIMETER_H
