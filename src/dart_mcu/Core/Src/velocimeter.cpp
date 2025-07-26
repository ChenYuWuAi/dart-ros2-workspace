//
// Created by cheny on 24-9-15.
//

#include "velocimeter.h"

#include "dartmcu_node.h"

namespace meter {
    velocimeter velocity_meter;

    void velocimeter::begin(
        TIM_HandleTypeDef* htim,
        uint32_t channel_begin_rise,
        uint32_t channel_begin_fall,
        uint32_t channel_end_rise,
        uint32_t channel_end_fall,
        uint32_t timer_period,
        std::function<void(float)> onVelocityUpdate,
        double distanceBetweenTwoPulse,
        double object_length,
        double seconds_per_tick
    ) {
        this->htim_begin         = htim;
        this->timer_period       = timer_period;
        this->onVelocityUpdate   = onVelocityUpdate;
        this->pulse_distance     = distanceBetweenTwoPulse;
        this->seconds_per_tick   = seconds_per_tick;
        this->object_length      = object_length;
        this->channel_gate1_rise = channel_begin_rise;
        this->channel_gate1_fall = channel_begin_fall;
        this->channel_gate2_rise = channel_end_rise;
        this->channel_gate2_fall = channel_end_fall;
        state                    = IDLE;
        refresh_counter_timer    = 0;

        // 初始化双光电门相关
        gate1_rise_tick     = gate1_fall_tick     = gate2_rise_tick     = gate2_fall_tick     = 0;
        gate1_rise_overflow = gate1_fall_overflow = gate2_rise_overflow = gate2_fall_overflow = 0;
        gate1_rise_valid    = gate1_fall_valid    = gate2_rise_valid    = gate2_fall_valid    = false;

        HAL_TIM_Base_Start_IT(htim);
        HAL_TIM_IC_Start_IT(htim, channel_begin_rise);
        HAL_TIM_IC_Start_IT(htim, channel_begin_fall);
        HAL_TIM_IC_Start_IT(htim, channel_end_rise);
        HAL_TIM_IC_Start_IT(htim, channel_end_fall);
    }

    void velocimeter::enable(bool oneshot) {
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);
        prev_state = state;
        state      = oneshot ? ONESHOT : CONTINOUS;
        HAL_TIM_IC_Start_IT(htim_begin, channel_gate1_rise);
        HAL_TIM_IC_Start_IT(htim_begin, channel_gate1_fall);
        HAL_TIM_IC_Start_IT(htim_begin, channel_gate2_rise);
        HAL_TIM_IC_Start_IT(htim_begin, channel_gate2_fall);
    }

    void velocimeter::disable() {
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_RESET);
        prev_state = state;
        state      = IDLE;
    }

    void velocimeter::onUpdate(TIM_HandleTypeDef* htim) {
        if (htim == htim_begin) {
            refresh_counter_timer++;
        }
    }

    // 单独处理每个通道的捕获
    void velocimeter::onCaptureGate1Rise(uint32_t count) {
        if (state == CONTINOUS || state == ONESHOT) {
            prev_state = state;
            if (state == ONESHOT) {
                HAL_TIM_IC_Stop_IT(htim_begin, channel_gate1_rise);
            }
            state               = MEASURING;
            gate1_rise_tick     = count;
            gate1_rise_overflow = refresh_counter_timer;
            gate1_rise_valid    = true;
        } else if (state == MEASURING) {
            // 可能漏了一个，判断超时或异常，重置
            uint64_t ticks_diff = (refresh_counter_timer - gate1_rise_overflow) * timer_period + (count -
                gate1_rise_tick);
            float velocity = pulse_distance / (ticks_diff * seconds_per_tick);
            if (velocity < 1) {
                state               = MEASURING;
                gate1_rise_overflow = refresh_counter_timer;
                gate1_rise_tick     = count;
            } else if (velocity > 25) {
                prev_state = state;
                state      = CONTINOUS;
            }
        }
    }

    void velocimeter::onCaptureGate1Fall(uint32_t count) {
        if (state == MEASURING) {
            gate1_fall_tick     = count;
            gate1_fall_overflow = refresh_counter_timer;
            gate1_fall_valid    = true;
        }
    }

    void velocimeter::onCaptureGate2Rise(uint32_t count) {
        if (state == MEASURING) {
            gate2_rise_tick     = count;
            gate2_rise_overflow = refresh_counter_timer;
            gate2_rise_valid    = true;
        }
    }

    void velocimeter::onCaptureGate2Fall(uint32_t count) {
        if (state == MEASURING && gate1_rise_valid && gate1_fall_valid && gate2_rise_valid) {
            gate2_fall_tick     = count;
            gate2_fall_overflow = refresh_counter_timer;
            gate2_fall_valid    = true;
            float div_count     = 0.0;

            // 计算三平均速度
            uint64_t ticks1 = (gate1_fall_overflow - gate1_rise_overflow) * timer_period + (gate1_fall_tick -
                gate1_rise_tick);
            float v1 = (object_length > 0 && ticks1 > 0) ? (object_length / (ticks1 * seconds_per_tick)) : 0;
            dart_mcu_log("v1(begin): %f", v1);
            if (v1 >= 1.0f && v1 <= 21.0f) {
                div_count++;
            } else
                v1 = 0.0f;

            uint64_t ticks2 = (gate2_fall_overflow - gate2_rise_overflow) * timer_period + (gate2_fall_tick -
                gate2_rise_tick);
            float v2 = (object_length > 0 && ticks2 > 0) ? (object_length / (ticks2 * seconds_per_tick)) : 0;
            dart_mcu_log("v2(end): %f", v2);
            if (v2 >= 1.0f && v2 <= 21.0f) {
                div_count++;
            } else
                v2 = 0.0f;

            uint64_t ticks12 = (gate2_rise_overflow - gate1_rise_overflow) * timer_period + (gate2_rise_tick -
                gate1_rise_tick);
            float v3 = (pulse_distance > 0 && ticks12 > 0) ? (pulse_distance / (ticks12 * seconds_per_tick)) : 0;
            dart_mcu_log("v3(between): %f", v3);
            if (v3 >= 1.0f && v3 <= 21.0f) {
                div_count++;
            } else
                v3 = 0.0f;

            float v_avg = (v1 + v2 + v3) / div_count;

            bool valid = v_avg >= 1.0f && v_avg <= 21.0f;

            if (valid) {
                onVelocityUpdate(v_avg);
                dart_mcu_log(" div_count: %f", div_count);
                state      = (prev_state == CONTINOUS) ? CONTINOUS : IDLE;
                prev_state = MEASURING;
            } else {
                // 无效则继续MEASURING，等待下一组

                state = MEASURING;
            }

            // 清空标志，准备下一次
            gate1_rise_valid = gate1_fall_valid = gate2_rise_valid = gate2_fall_valid = false;
        }
    }

    void velocimeter::reset() {
        state = CONTINOUS;
        // 清空双光电门标志
        gate1_rise_valid = gate1_fall_valid = gate2_rise_valid = gate2_fall_valid = false;
    }
}
