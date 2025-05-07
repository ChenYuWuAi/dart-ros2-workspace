/*
 * status_led.h
 *
 *  Created on: 19.07.2022
 *      Author: bartek
 */

#include "./buzzer.h"

Buzzer_HandleTypeDef hbuzzer;

//void Buzzer_Note(Buzzer_HandleTypeDef *handle, uint32_t noteFreq) {
////	printf("note %d\r\n", noteFreq);
//    if (noteFreq > 0) {
//        handle->Init.timer->Instance->ARR =
//                handle->Init.timerClockFreqHz / (handle->Init.timer->Init.Prescaler + 1) / noteFreq;
//        __HAL_TIM_SET_COMPARE(handle->Init.timer, handle->Init.channel, handle->Init.timer->Instance->ARR / 2);
//    } else
//        __HAL_TIM_SET_COMPARE(handle->Init.timer, handle->Init.channel, 0);
//}

void Buzzer_Note(Buzzer_HandleTypeDef *handle, uint32_t noteFreq) {
    if (noteFreq == 0) {
        __HAL_TIM_SET_COMPARE(handle->Init.timer, handle->Init.channel, 0);
        return;
    }
    uint32_t clk = handle->Init.timerClockFreqHz;
    uint32_t maxArr = 0xFFFF;
    // 1）先按当前 PSC 计算周期
    uint32_t base = handle->Init.timer->Instance->PSC + 1;
    uint32_t ticks = clk / base / noteFreq;
    if (ticks > maxArr) {
        // 2）需要更大 PSC：使得 ticks/psc <= maxArr
        uint32_t needed = (clk / noteFreq + maxArr - 1) / maxArr; // 向上取整
        // PSC 寄存器 值 = needed-1
        handle->Init.timer->Instance->PSC = needed - 1;
        ticks = clk / needed / noteFreq;
    }
    // 3）设置 ARR 和 CCR（一半占空比）
    handle->Init.timer->Instance->ARR = ticks - 1;
    __HAL_TIM_SET_COMPARE(handle->Init.timer, handle->Init.channel, (ticks - 1) / 2);
}

void Buzzer_NoNote(Buzzer_HandleTypeDef *handle) {
    Buzzer_Note(handle, 0);
}

void Buzzer_Init(Buzzer_HandleTypeDef *handle, Buzzer_InitTypeDef *config) {
    handle->Init = *config;
}

void Buzzer_Start(Buzzer_HandleTypeDef *handle) {
    Buzzer_NoNote(handle);
    HAL_TIM_PWM_Start(handle->Init.timer, handle->Init.channel);
}