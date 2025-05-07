//
// Created by cheny on 24-9-11.
//

#include "servo.h"

servo trigger_servo[7];

void servo::begin(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t timerClock, uint32_t minPulseWidth, uint32_t maxPulseWidth, uint16_t minAngle, uint16_t maxAngle, uint32_t pwm_period, uint32_t pwm_frequency, uint16_t initialAngle) {
    pwm_.configure(htim, channel, timerClock, pwm_period, pwm_frequency);

    minPulseWidth_ = minPulseWidth;
    maxPulseWidth_ = maxPulseWidth;
    minAngle_ = minAngle;
    maxAngle_ = maxAngle;

    pwm_.setCompareValue(calCompareValue(initialAngle));
}

void servo::setAngle(uint16_t angle) {
    if (angle < minAngle_) angle = minAngle_;
    if (angle > maxAngle_) angle = maxAngle_;

    pwm_.setCompareValue(calCompareValue(angle));
}

void servo::enable() {
    pwm_.start();
}

void servo::disable() {
    pwm_.stop();
}

inline uint32_t servo::calCompareValue(uint16_t angle) {
    return (angle - minAngle_) * (maxPulseWidth_ - minPulseWidth_) / (maxAngle_ - minAngle_) + minPulseWidth_;
}