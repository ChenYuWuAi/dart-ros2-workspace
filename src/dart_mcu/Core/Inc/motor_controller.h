//
// Created by cheny on 24-6-29.
//

#ifndef DART_25_CONTROLLER_H
#define DART_25_CONTROLLER_H

#include "motor.h"

#define LIMIT_MIN_MAX(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

namespace motor_controller {
    enum E_PID_Velocity_Angle_Controller_State {
        VELOCITY_CONTROL,
        ANGLE_CONTROL,
        OPEN_LOOP
    };

    template<typename T>
    struct pid_controller {
        T kp, ki, kd;
        T cur_error, last_error, sum_error;
        T sum_error_max;
        T p_max, i_max, output_max;
        T output;
        T target;

        pid_controller(T kp, T ki, T kd, T sum_error_max, T p_max, T i_max, T output_max);

        T update(T current);

        void reset();
    };

    template<typename T>
    struct pid_angle_velocity_controller {
        pid_controller<T> pid_velocity_;
        pid_controller<T> pid_angle_;

        int32_t target_angle_with_rounds_; // 0-8192 + rounds * 8192
        int32_t current_angle_with_rounds_;

        int16_t target_velocity_{};
        int16_t current_velocity_{};

        int16_t target_openloop_{};

        motor::motor_rm *motor_;

        E_PID_Velocity_Angle_Controller_State state_;

        pid_angle_velocity_controller(pid_controller<T> pid_velocity, pid_controller<T> pid_angle,
                                      motor::motor_rm *motor,
                                      E_PID_Velocity_Angle_Controller_State state);

        T update();

        void set_state(E_PID_Velocity_Angle_Controller_State state) {
            if (state_ != state) {
                state_ = state;
                reset();
            }
        }

        void reset();
    };

    extern double motor_load_sync_offset;

    extern pid_angle_velocity_controller<double> MotorTriggerLSController;
    extern pid_angle_velocity_controller<double> MotorYawLSController;
    extern pid_angle_velocity_controller<double> MotorLoadController[2];
    extern pid_angle_velocity_controller<double> AutoSightController;

    extern pid_controller<double> MotorLoadSyncController;

    [[noreturn]] void pid_control_task(void *pvParameter);
}
#endif //DART_25_CONTROLLER_H
