//
// Created by cheny on 24-9-11.
//

#ifndef DART_MCU_SERVO_H
#define DART_MCU_SERVO_H

#include "pwm.h"
class servo {
private:
    PWM pwm_;
public:
    uint32_t minPulseWidth_;
    uint32_t maxPulseWidth_;
    uint16_t minAngle_; // 单位：度
    uint16_t maxAngle_; // 单位：度

    servo() = default;

    void begin(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t timerClock, uint32_t minPulseWidth,
               uint32_t maxPulseWidth, uint16_t minAngle, uint16_t maxAngle, uint32_t pwm_period,
               uint32_t pwm_frequency, uint16_t initialAngle = 0);

    void setAngle(uint16_t angle);

    void enable();

    void disable();

private:
    inline uint32_t calCompareValue(uint16_t angle);
};

extern servo trigger_servo[7];


#endif //DART_MCU_SERVO_H
