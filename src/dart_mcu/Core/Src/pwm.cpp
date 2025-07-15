#include "pwm.h"

void PWM::configure(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t timerClock, uint32_t period, uint32_t frequency) {
    htim_ = htim;
    channel_ = channel;
    timerClock_ = timerClock;

    __HAL_TIM_SET_AUTORELOAD(htim_, period - 1);
    // 根据频率重新计算分频器
    __HAL_TIM_SET_PRESCALER(htim_, (timerClock_ / (frequency * period)) - 1);
    updateTimer();
}

void PWM::setDutyCycle(float dutyCyclePercent) {
    if (dutyCyclePercent < 0.0f) dutyCyclePercent = 0.0f;
    if (dutyCyclePercent > 100.0f) dutyCyclePercent = 100.0f;

    setCompareValue(static_cast<uint32_t>((dutyCyclePercent / 100.0f) * getAutoReload()));
}

void PWM::setCompareValue(uint32_t compareValue) {
    __HAL_TIM_SET_COMPARE(htim_, channel_, compareValue);
}

//void PWM::setTimerClock(uint32_t timerClock) {
//    // 定时器时钟源频率变化，重新配置预分频系数
//    uint32_t formerCompareValue = getCompareValue();
//    uint32_t formerAutoReload = getAutoReload();
//    uint32_t formerFrequency = getFrequency();
//    timerClock_ = timerClock;
//    setCompareValue(formerCompareValue);
//    configure(htim_, channel_, timerClock_, formerAutoReload, formerFrequency);
//    updateTimer();
//}

uint32_t PWM::getFrequency() const {
    return timerClock_ / (getAutoReload() * (__HAL_TIM_GET_ICPRESCALER(htim_, channel_) + 1));
}

uint32_t PWM::getAutoReload() const {
    return __HAL_TIM_GET_AUTORELOAD(htim_);
}

uint32_t PWM::getCompareValue() const {
    return __HAL_TIM_GET_COMPARE(htim_, channel_);
}

void PWM::start() {
    HAL_TIM_PWM_Start(htim_, channel_);
}

void PWM::stop() {
    HAL_TIM_PWM_Stop(htim_, channel_);
}

void PWM::updateTimer() {
    // 使设置生效
    __HAL_TIM_SET_COUNTER(htim_, 0);  // 复位计数器
    HAL_TIM_GenerateEvent(htim_, TIM_EVENTSOURCE_UPDATE);
}

