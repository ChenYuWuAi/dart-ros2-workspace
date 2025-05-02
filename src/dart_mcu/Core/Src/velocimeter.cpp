//
// Created by cheny on 24-9-15.
//

#include "velocimeter.h"

namespace meter {

    velocimeter velocity_meter;

    void velocimeter::begin(TIM_HandleTypeDef *htim_begin, uint32_t channel_begin, TIM_HandleTypeDef *htim_end,
                            uint32_t channel_end,
                            uint32_t timer_period, std::function<void(float)> onVelocityUpdate,
                            double distanceBetweenTwoPulse,
                            double seconds_per_tick) {
        this->htim_begin = htim_begin;
        this->timer_period = timer_period;
        this->onVelocityUpdate = onVelocityUpdate;
        this->distanceBetweenTwoPulse = distanceBetweenTwoPulse;
        this->seconds_per_tick = seconds_per_tick;
        this->channel_begin = channel_begin;
        this->channel_end = channel_end;
        state = IDLE;
        refresh_counter_timer = 0;
        update_count_begin = 0;
        update_count_end = 0;
        begin_time = 0;
        end_time = 0;

        HAL_TIM_Base_Start_IT(htim_begin);
        HAL_TIM_IC_Start_IT(htim_begin, channel_begin);
        HAL_TIM_IC_Start_IT(htim_end, channel_end);
    }

    void velocimeter::enable(bool oneshot) {
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);
        prev_state = state;
        state = oneshot ? ONESHOT : CONTINOUS;
        HAL_TIM_IC_Start_IT(htim_begin, channel_begin);
    }

    void velocimeter::disable() {
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_RESET);
        prev_state = state;
        state = IDLE;
    }

    void velocimeter::onUpdate(TIM_HandleTypeDef *htim) {
        if (htim == htim_begin) {
            refresh_counter_timer++;
        }
    }

    void velocimeter::onCaptureBegin(uint32_t count) {
        if (state == CONTINOUS || state == ONESHOT) {
            prev_state = state;
            if (state == ONESHOT)
                HAL_TIM_IC_Stop_IT(htim_begin, velocity_meter.channel_begin);
            state = MEASURING;
            update_count_begin = refresh_counter_timer;
            begin_time = count;
        }
            // 可能漏了一个 算一下时间 如果超时就再来一次ready
        else if (state == MEASURING) {
            // 计算速度
            uint64_t ticks_diff = (refresh_counter_timer - update_count_begin) * timer_period + (count - begin_time);
            float velocity = distanceBetweenTwoPulse / (ticks_diff * seconds_per_tick);
            if (velocity < 1) {
                state = MEASURING;
                update_count_begin = refresh_counter_timer;
                begin_time = count;
            } else if (velocity > 25) {
                prev_state = state;
                state = CONTINOUS;
            }
        }
    }

    void velocimeter::onCaptureEnd(uint32_t count) {
        if (state == MEASURING) {
            update_count_end = refresh_counter_timer;
            end_time = count;
            // 计算速度
            uint64_t ticks_diff = (update_count_end - update_count_begin) * timer_period + (end_time - begin_time);
            float velocity = distanceBetweenTwoPulse / (ticks_diff * seconds_per_tick);
            onVelocityUpdate(velocity);
            state = prev_state == CONTINOUS ? CONTINOUS : IDLE;
            prev_state = MEASURING;
        }
    }

    void velocimeter::reset() {
        state = CONTINOUS;
    }
}

