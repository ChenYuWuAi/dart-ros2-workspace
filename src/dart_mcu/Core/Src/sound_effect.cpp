// sound_effect.cpp
// 完整版：每个音符后自动插入 5ms 休止符
#include "sound_effect.h"
#include "buzzer.h"
#include "tim.h"
#include <memory>
#include <vector>

SoundEffectManager soundEffectManager;

using namespace std;

void SoundEffectManager::begin(
        TIM_HandleTypeDef *timer_pwm,
        TIM_HandleTypeDef *timer_beep_,
        __IO uint32_t pwm_channel,
        uint32_t timerClockFreqHz) {
    Buzzer_InitTypeDef cfg;
    cfg.channel = pwm_channel;
    cfg.timer = timer_pwm;
    cfg.timerClockFreqHz = timerClockFreqHz;
    timer_beep = timer_beep_;
    Buzzer_Init(&hbuzzer, &cfg);
    Buzzer_Start(&hbuzzer);
}

shared_ptr<soundEffect_t>
SoundEffectManager::addSoundEffect(
        note_t *notes_, size_t notes_size_, bool emergency, bool circulating, bool add_rest) {
    auto se = make_shared<soundEffect_t>();
    se->notes = notes_;
    se->notes_size = notes_size_;
    se->progress = 0;
    se->state = circulating ? SoundEffectState::READY_FOR_CIRCULATING
                            : SoundEffectState::READY;
    se->add_rest = add_rest; // 新增参数
    if (emergency) {
        soundEffects_queue.insert(soundEffects_queue.begin(), se);
    } else {
        soundEffects_queue.push_back(se);
    }
    if (!currentSoundEffect) {
        Start_SoundEffect();
    }
    return se;
}

void SoundEffectManager::Start_SoundEffect() {
    if (soundEffects_queue.empty()) {
        HAL_TIM_Base_Stop(timer_beep);
        currentSoundEffect = nullptr;
        return;
    }
    currentSoundEffect = soundEffects_queue.front();
    currentSoundEffect->state =
            (currentSoundEffect->state == SoundEffectState::READY_FOR_CIRCULATING)
            ? SoundEffectState::CIRCULATING
            : SoundEffectState::PLAYING;

    // 立即触发回调以播放第一个阶段
            __HAL_TIM_SetCounter(timer_beep, 0);
    __HAL_TIM_SET_AUTORELOAD(timer_beep, 1);
    HAL_TIM_Base_Start_IT(timer_beep);
}

void SoundEffectManager::timer_callback(void *pvParameters) {
    auto *mgr = static_cast<SoundEffectManager *>(pvParameters);
    auto se = mgr->currentSoundEffect;
    if (!se) return;

    // 根据是否添加休止符计算总阶段数
    uint32_t total = se->add_rest ? se->notes_size * 2 : se->notes_size;
    if (se->progress < total &&
        (se->state == SoundEffectState::PLAYING ||
         se->state == SoundEffectState::CIRCULATING)) {
        bool rest = se->add_rest && (se->progress & 1) != 0; // 判断是否为休止符阶段
        uint32_t idx = se->add_rest ? (se->progress >> 1) : se->progress;
        uint32_t dur_ms;
        if (rest)
            dur_ms = 10;
        else if (se->notes[idx].duration <= 10)
            dur_ms = se->notes[idx].duration;
        else
            dur_ms = se->notes[idx].duration - 10;

        if (rest) {
            Buzzer_NoNote(&mgr->hbuzzer);    // 静音阶段
        } else {
            Buzzer_Note(&mgr->hbuzzer,
                        se->notes[idx].pitch); // 发声阶段
        }
        se->progress++;

        // 重载定时器：duration (ms) → timer ticks (0.1 ms per tick)
                __HAL_TIM_SetCounter(mgr->timer_beep, 0);
        __HAL_TIM_SET_AUTORELOAD(
                mgr->timer_beep,
                dur_ms * 10 - 1);
    } else {
        // 播放完毕或循环处理
        if (se->state == SoundEffectState::CIRCULATING) {
            se->progress = 0;
        } else {
            se->state = SoundEffectState::PLAYED;
            Buzzer_NoNote(&mgr->hbuzzer);
            // 移除当前并启动下一个
            mgr->soundEffects_queue.erase(
                    mgr->soundEffects_queue.begin());
            mgr->currentSoundEffect = nullptr;
            mgr->Start_SoundEffect();
        }
    }
}

void SoundEffectManager::stopCurrentSoundEffect() {
    if (currentSoundEffect) {
        currentSoundEffect->state = SoundEffectState::PLAYED;
    }
}

void SoundEffectManager::clearSoundEffects() {
    soundEffects_queue.clear();
    currentSoundEffect = nullptr;
    Buzzer_NoNote(&hbuzzer);
    HAL_TIM_Base_Stop(timer_beep);
}
