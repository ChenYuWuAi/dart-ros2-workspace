//
// Created by cheny on 24-6-29.
//

#ifndef DART_24_CONTROLLER_H
#define DART_24_CONTROLLER_H

#include "motor.h"

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

        pid_controller(T kp, T ki, T kd, T sum_error_max, T p_max, T i_max, T output_max)
                :
                kp(kp), ki(ki), kd(kd), sum_error_max(sum_error_max), p_max(p_max), i_max(i_max),
                output_max(output_max) {
            cur_error = 0;
            last_error = 0;
            sum_error = 0;
            output = 0;
        }

        T update(T current) {
            last_error = cur_error;
            cur_error = target - current;
            sum_error = LIMIT_MIN_MAX(sum_error + cur_error,
                                      -sum_error_max,
                                      sum_error_max);

            output = LIMIT_MIN_MAX((kp * cur_error), -p_max, p_max) +
                     LIMIT_MIN_MAX((ki * sum_error), -i_max, i_max) +
                     kd * (cur_error - last_error);

            output = LIMIT_MIN_MAX(output, -output_max, output_max);

            return output;
        }

        void reset() {
            cur_error = 0;
            last_error = 0;
            sum_error = 0;
            output = 0;
        }
    };

    template<typename T>
    struct pid_angle_velocity_controller {
        pid_controller<T> pid_velocity_;
        pid_controller<T> pid_angle_;

        int32_t target_angle_with_rounds_; // 0-8192 + rounds * 8192
        int32_t current_angle_with_rounds_;

        int16_t target_velocity_;
        int16_t current_velocity_;

        int16_t target_openloop_;

        motor::motor_dji *motor_;

        E_PID_Velocity_Angle_Controller_State state_;

        pid_angle_velocity_controller(pid_controller<T> pid_velocity, pid_controller<T> pid_angle,
                                      motor::motor_dji *motor,
                                      E_PID_Velocity_Angle_Controller_State state
        ) : pid_velocity_(pid_velocity), pid_angle_(pid_angle), motor_(motor), state_(state) {
            target_angle_with_rounds_ = 0;
            current_angle_with_rounds_ = motor_->current_round_ * 8192 + motor_->current_angle_;
        }

        T update() {
            if (state_ == VELOCITY_CONTROL) {
                current_angle_with_rounds_ = motor_->current_round_ * 8192 + motor_->current_angle_;
                current_velocity_ = motor_->current_velocity_ * 0.6 + current_velocity_ * 0.4;
                pid_velocity_.target = target_velocity_;
                return pid_velocity_.update(current_velocity_);
            } else if (state_ == ANGLE_CONTROL) {
                // оч╥Ы
                LIMIT_MIN_MAX(target_angle_with_rounds_, 0, 370000);
                current_angle_with_rounds_ = motor_->current_round_ * 8192 + motor_->current_angle_;
                current_velocity_ = motor_->current_velocity_ * 0.6 + current_velocity_ * 0.4;
                pid_angle_.target = target_angle_with_rounds_;
                pid_velocity_.target = -pid_angle_.update(current_angle_with_rounds_);
                return pid_velocity_.update(current_velocity_);
            } else {
                return target_openloop_;
            }
        }

        void set_state(E_PID_Velocity_Angle_Controller_State state) {
            if (state_ != state) {
                state_ = state;
                reset();
            }
        }

        void reset() {
            pid_velocity_.reset();
            pid_angle_.reset();
        }
    };

    void control_node_spawner(void *argument);

    extern pid_angle_velocity_controller<double> pid_velocity_MotorLS;
    extern pid_angle_velocity_controller<double> pid_velocity_MotorDM;
    extern pid_angle_velocity_controller<double> pid_angle_velocity_MotorY;

}
#endif //DART_24_CONTROLLER_H
