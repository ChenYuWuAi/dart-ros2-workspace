//
// Created by cheny on 24-9-10.
//

#ifndef DART_MCU_SOUND_EFFECT_H
#define DART_MCU_SOUND_EFFECT_H

#include "main.h"
#include "buzzer.h"
#include "buzzer_tones.h"
#include <vector>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcutils/time.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"

#define BUZZER_NOTE(x) x, (sizeof(x) / sizeof(x[0]))

// 智能指针
#include <memory>

// 可以触发音效事件/进入警报模式
enum SoundEffectState {
    READY,
    READY_FOR_CIRCULATING,
    PLAYING,
    CIRCULATING,
    PLAYED
};

typedef struct soundEffect_t {
    // 音效数组
    note_t *notes;
    // 音效数组大小
    size_t notes_size;
    // 播放进度
    size_t progress;
    // 行为
    uint8_t state;
    bool add_rest;
} soundEffect_t;


//HAL_RCC_GetPCLK2Freq();
class SoundEffectManager {
private:
    Buzzer_HandleTypeDef hbuzzer;
    std::vector<std::shared_ptr<soundEffect_t>> soundEffects_queue;
    std::shared_ptr<soundEffect_t> currentSoundEffect;
    TIM_HandleTypeDef *timer_beep;

public:
    SoundEffectManager() = default;

    // Timer_Beep必须配成10000Hz
    void begin(TIM_HandleTypeDef *timer_pwm,
               TIM_HandleTypeDef *timer_beep_,
               __IO uint32_t pwm_channel,
               uint32_t timerClockFreqHz);

    std::shared_ptr<soundEffect_t>
    addSoundEffect(note_t *notes_, size_t notes_size_, bool emergency = false, bool circulating = false,  bool add_rest = true);

    // 开始播放音效
    void Start_SoundEffect();

    // Usage: 在定时器溢出中断中使用
    // SoundEffectManager::timer_callback(&soundEffectManager);
    static void timer_callback(void *pvParameters);

    void stopCurrentSoundEffect() ;

    void clearSoundEffects();
};

extern SoundEffectManager soundEffectManager;


#endif //DART_MCU_SOUND_EFFECT_H
