//
// Created by cheny on 24-6-29.
//
// Motor Interface
#include "motor.h"
#include "can.h"

namespace motor {
    motor_dji MotorLS(&hcan1, 10000);
    motor_dji MotorY(&hcan1, 10000, true);
    motor_dji MotorDM(&hcan1, 16384);

    motor_vesc MotorFW[4] = {
            motor::motor_vesc(&hcan2, 54000, 7, 1, true),
            motor::motor_vesc(&hcan2, 54000, 7, 2, true),
            motor::motor_vesc(&hcan2, 54000, 7, 3, true),
            motor::motor_vesc(&hcan2, 54000, 7, 4, true)
    };

    void update_can_array(uint8_t *aData, uint8_t id, int16_t output) {
        aData[id * 2] = (uint8_t) (output >> 8);
        aData[id * 2 + 1] = (uint8_t) (output);
    }

    void int32_to_can_frame(int32_t RPM_value_, int poles, unsigned char *data) {
        int32_t value = RPM_value_ * poles;
        data[0] = (value >> 24) & 0xFF;
        data[1] = (value >> 16) & 0xFF;
        data[2] = (value >> 8) & 0xFF;
        data[3] = value & 0xFF;
    }
}