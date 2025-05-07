//
// Created by cheny on 24-9-11.
//

#ifndef DART_MCU_MOTOR_H
#define DART_MCU_MOTOR_H

#include "inttypes.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"

namespace motor {

    enum E_MotorState {
        IDLE,
        RUNNING,
        DISCONNECTED,
        UNDEFINED
    };

    enum E_MotorType {
        M2006,
        M3508,
        GM6020
    };

    void update_can_array(uint8_t *aData, uint8_t id, int16_t output);

    class motor_rm {
    public:
        // Current
        int16_t target_current_;   // 0-10000

        // Velocity
        int16_t current_velocity_; // RPM
        int16_t max_current_;      // 0-10000

        // Round & Position
        int current_round_;
        int current_angle_;

        // Update Time and Disconnect check
        TickType_t last_update_time_;

        // Type
        E_MotorType motor_type_;

        // Can Interface
        uint8_t motor_id_;

        // Motor State
        E_MotorState motor_state_ = DISCONNECTED;
    private:
        int current_angle_last_;
        // Motor Angle Reverse Flag
        bool angle_reverse_;
        bool first_decode_ = false;

        // Motor State Flag
        E_MotorState motor_state_next_;

    public:
        void create(int16_t max_current, E_MotorType motor_type, uint8_t motor_id,
                    bool angle_reserve = false) {
            max_current_ = max_current;
            angle_reverse_ = angle_reserve;
            target_current_ = 0;
            current_velocity_ = 0;
            current_round_ = 0;
            last_update_time_ = xTaskGetTickCount();
            motor_state_ = DISCONNECTED;
            motor_type_ = motor_type;
            motor_id_ = motor_id;
        }

        // Interface
        void decodeCanMsg(CAN_RxHeaderTypeDef *rxHeader, const uint8_t *rxData);

        bool setNextState(E_MotorState state);

        void resetRound();

        bool setCurrent(int16_t target_current);

        int16_t updateCurrent();
    };

    extern motor_rm MotorTriggerLS;  // 扳机丝杆电机
    extern motor_rm MotorYawLS;      // 偏航丝杆电机
    extern motor_rm MotorLoad[2];    // 装填电机
};
#endif //DART_MCU_MOTOR_H
