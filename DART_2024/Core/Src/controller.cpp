//
// Created by cheny on 24-6-29.
//
#include "controller.h"
#include "motor.h"
#include "can.h"
#include "dart_param.h"
#include <cstring>

namespace motor_controller {
    pid_angle_velocity_controller<double> pid_velocity_MotorLS(
            pid_controller<double>(PID_MOTORLS_KP, PID_MOTORLS_KI, PID_MOTORLS_KD,
                                   PID_MOTORLS_SUM_ERROR_MAX, PID_MOTORLS_P_MAX, PID_MOTORLS_I_MAX,
                                   PID_MOTORLS_I_MAX), pid_controller<double>(0, 0, 0, 0, 0, 0, 0),
            &motor::MotorLS,
            E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);

    pid_angle_velocity_controller<double> pid_velocity_MotorDM(
            pid_controller<double>(PID_MOTORDM_KP, PID_MOTORDM_KI, PID_MOTORDM_KD,
                                   PID_MOTORDM_SUM_ERROR_MAX, PID_MOTORDM_P_MAX, PID_MOTORDM_I_MAX,
                                   PID_MOTORDM_I_MAX), pid_controller<double>(0, 0, 0, 0, 0, 0, 0),
            &motor::MotorDM,
            E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);

    pid_angle_velocity_controller<double>
            pid_angle_velocity_MotorY(pid_controller<double>(PID_MOTORY_KP, PID_MOTORY_KI, PID_MOTORY_KD,
                                                             PID_MOTORY_SUM_ERROR_MAX, PID_MOTORY_P_MAX,
                                                             PID_MOTORY_I_MAX,
                                                             PID_MOTORY_OUTPUT_MAX),
                                      pid_controller<double>(PID_MOTORY_ANGLE_KP, PID_MOTORY_ANGLE_KI,
                                                             PID_MOTORY_ANGLE_KD,
                                                             PID_MOTORY_ANGLE_SUM_ERROR_MAX, PID_MOTORY_ANGLE_P_MAX,
                                                             PID_MOTORY_ANGLE_I_MAX,
                                                             PID_MOTORY_ANGLE_OUTPUT_MAX),
                                      &motor::MotorY,
                                      E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);

    [[noreturn]] void control_node_fw(void *pvParameters) {
        // 20Hz
        TickType_t xLastWakeTime = xTaskGetTickCount();
        while (true) {
            // Update Motor State for MotorFW
            for (auto &i: motor::MotorFW) {
                i.update(); // Will detect Motor State Automatically
                vTaskDelayUntil(&xLastWakeTime, 100);
            }
        }
        vTaskDelete(nullptr);
    }

    [[noreturn]] void control_node_pid(void *pvParameters) {
        // 1KHz
        // Initialize PID Controller
        TickType_t xLastWakeTime = xTaskGetTickCount();
        static uint32_t tx_mailbox;
        // Can Frame
        CAN_TxHeaderTypeDef tx_header;
        tx_header.StdId = 0x200;
        tx_header.IDE = CAN_ID_STD;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = 8;
        tx_header.TransmitGlobalTime = DISABLE;

        uint8_t can_array[8];
        while (true) {
            // Update PID Controller
            // Check Disconnect
            if (xTaskGetTickCount() - motor::MotorLS.last_update_time_ > 1000) {
                motor::MotorLS.motor_state_ = motor::DISCONNECTED;
            } else if (motor::MotorLS.motor_state_ == motor::DISCONNECTED) {
                motor::MotorLS.motor_state_ = motor::IDLE;
            }

            if (xTaskGetTickCount() - motor::MotorDM.last_update_time_ > 1000) {
                motor::MotorDM.motor_state_ = motor::DISCONNECTED;
            } else if (motor::MotorDM.motor_state_ == motor::DISCONNECTED) {
                motor::MotorDM.motor_state_ = motor::IDLE;
            }

            // Update Motor State
            memset(can_array, 0, 8);
            if (motor::MotorLS.motor_state_ == motor::RUNNING) {
                motor::MotorLS.set_current(pid_velocity_MotorLS.update());
                motor::update_can_array(can_array, 0, motor::MotorLS.target_current_);
            } else {
                motor::update_can_array(can_array, 0, 0);
                pid_velocity_MotorLS.reset();
            }
            if (motor::MotorDM.motor_state_ == motor::RUNNING) {
                motor::MotorDM.set_current(pid_velocity_MotorDM.update());
                motor::update_can_array(can_array, 1, motor::MotorDM.target_current_);
            } else {
                motor::update_can_array(can_array, 1, 0);
                pid_velocity_MotorDM.reset();
            }

            if (!(motor::MotorDM.motor_state_ == motor::DISCONNECTED &&
                  motor::MotorLS.motor_state_ == motor::DISCONNECTED))
                HAL_CAN_AddTxMessage(motor::MotorLS.hcan_, &tx_header, can_array, &tx_mailbox);

            vTaskDelayUntil(&xLastWakeTime, 1);

            // Update Next State
            if (motor::MotorLS.motor_state_next_ != motor::UNDEFINED &&
                motor::MotorLS.motor_state_ != motor::DISCONNECTED) {
                motor::MotorLS.motor_state_ = motor::MotorLS.motor_state_next_;
                motor::MotorLS.motor_state_next_ = motor::UNDEFINED;
            } else if (motor::MotorLS.motor_state_next_ != motor::UNDEFINED &&
                       motor::MotorLS.motor_state_ == motor::DISCONNECTED) {
                motor::MotorLS.motor_state_next_ = motor::UNDEFINED;
            }

            if (motor::MotorDM.motor_state_next_ != motor::UNDEFINED &&
                motor::MotorDM.motor_state_ != motor::DISCONNECTED) {
                motor::MotorDM.motor_state_ = motor::MotorDM.motor_state_next_;
                motor::MotorDM.motor_state_next_ = motor::UNDEFINED;
            } else if (motor::MotorDM.motor_state_next_ != motor::UNDEFINED &&
                       motor::MotorDM.motor_state_ == motor::DISCONNECTED) {
                motor::MotorDM.motor_state_next_ = motor::UNDEFINED;
            }

            // Update Angle Controller
            if (motor::MotorY.motor_state_ == motor::RUNNING || motor::MotorY.motor_state_next_ == motor::RUNNING) {
                motor::MotorY.set_current(pid_angle_velocity_MotorY.update());
            } else {
                pid_angle_velocity_MotorY.reset();
            }
            motor::MotorY.update(); // Will detect Motor Next State Automatically

            vTaskDelayUntil(&xLastWakeTime, 1);
        }
        vTaskDelete(nullptr);
    }

    void control_node_spawner(void *pvParameters) {
        xTaskCreate(control_node_fw, "control_node_fw", 512, nullptr, 6, nullptr);
        xTaskCreate(control_node_pid, "control_node_pid", 512, nullptr, 6, nullptr);
        vTaskDelete(nullptr);
    }
}