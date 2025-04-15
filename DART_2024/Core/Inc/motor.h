//
// Created by cheny on 24-6-29.
//

#ifndef DART_24_MOTOR_H
#define DART_24_MOTOR_H

#include "main.h"

#include "freeRTOS.h"
#include "task.h"

namespace motor {

    enum E_MotorState {
        IDLE,
        RUNNING,
        DISCONNECTED,
        UNDEFINED
    };

    void update_can_array(uint8_t *aData, uint8_t id, int16_t output);

    void int32_to_can_frame(int32_t RPM_value_, int poles, unsigned char *data);

    struct motor_vesc {
        int32_t target_velocity_; // RPM
        int32_t current_velocity_; // RPM
        int32_t max_velocity_; // ABS RPM
        uint8_t poles_;
        TickType_t last_update_time_;
        // CAN Interface
        CAN_HandleTypeDef *hcan_;
        uint8_t motor_id_;


        // Motor State
        E_MotorState motor_state_;
        E_MotorState motor_next_state_;

        bool direction;

        // Constructor
        motor_vesc(CAN_HandleTypeDef
                   *hcan,
                   int32_t
                   max_velocity, uint8_t
                   poles,
                   uint8_t
                   motor_id, bool
                   direction = true
        ) :

                hcan_(hcan), max_velocity_(max_velocity), poles_(poles) {
            target_velocity_ = 0;
            current_velocity_ = 0;
            last_update_time_ = xTaskGetTickCount();
            motor_state_ = E_MotorState::DISCONNECTED;
            motor_id_ = motor_id;
        }

        bool set_next_state(E_MotorState state) {
            if (motor_state_ != DISCONNECTED) {
                motor_next_state_ = state;
                return true;
            } else return false;
        }

        bool set_velocity(int32_t target_velocity) {
            if (target_velocity > max_velocity_)
                target_velocity = max_velocity_;
            else if (target_velocity < -max_velocity_)
                target_velocity = -max_velocity_;

            if (motor_state_ == IDLE)
                motor_state_ = (RUNNING);
            else if (motor_state_ == DISCONNECTED)
                return false;

            target_velocity_ = target_velocity;
            return true;
        }

        /**
         * @brief 解析CAN数据，先判断ID是该电机之后才可以解析数据
         * @param rxHeader
         * @param rxData
         */
        void decode(CAN_RxHeaderTypeDef *rxHeader, const uint8_t *rxData) {
            // 扩展帧
            if (rxHeader->IDE == CAN_ID_EXT)
                // 如果ID在0x9xx范围内，说明是反馈速度，将速度(ERPM)保存到current_velocity_中
                if (rxHeader->ExtId >= 0x901 && rxHeader->ExtId <= 0x9FF)
                    current_velocity_ = (rxData[0] << 24) | (rxData[1] << 16) | (rxData[2] << 8) | rxData[3];
            // 根据速度方向转化
            current_velocity_ = direction ? current_velocity_ : -current_velocity_;
            // 除去poles
            current_velocity_ /= poles_;
            last_update_time_ = xTaskGetTickCount();
        }

        bool update() {
            // Check Disconnect
            if (xTaskGetTickCount() - last_update_time_ > 1000) {
                motor_state_ = (DISCONNECTED);
            } else if (motor_state_ == DISCONNECTED) {
                motor_state_ = (IDLE);
            }

            // Update Motor State
            if (motor_state_ == RUNNING) {
                // Write CAN Data to hcan
                uint8_t data[8] = {0};
                static uint32_t txFifo = 0;
                int32_to_can_frame(target_velocity_, poles_, data);
                // 扩展帧 0x300 + 电机ID
                CAN_TxHeaderTypeDef txHeader;
                txHeader.DLC = 8;
                txHeader.ExtId = 0x300 + motor_id_;
                txHeader.IDE = CAN_ID_EXT;
                txHeader.RTR = CAN_RTR_DATA;
                txHeader.TransmitGlobalTime = DISABLE;
                HAL_CAN_AddTxMessage(hcan_, &txHeader, data, &txFifo);
            }

            // update next_state
            if (motor_next_state_ != UNDEFINED && motor_state_ != DISCONNECTED) {
                motor_state_ = motor_next_state_;
                motor_next_state_ = UNDEFINED;
            } else if (motor_next_state_ != UNDEFINED && motor_state_ == DISCONNECTED) {
                motor_next_state_ = UNDEFINED;
            }
            return true;
        }
    };

    struct motor_dji {
        int16_t target_current_;   // 0-10000
        int16_t current_velocity_; // RPM
        int16_t max_current_;      // 0-10000

        int current_round_;
        int current_round_last_;

        bool first_decode_ = false;

        uint16_t current_angle_; // 0-8191
        uint16_t current_angle_last_;

        TickType_t last_update_time_;

        // CAN Interface
        CAN_HandleTypeDef *hcan_;
        uint8_t motor_id_;

        // Motor Angle Reverse
        bool angle_reverse_;

        // Motor State
        E_MotorState motor_state_;
        E_MotorState motor_state_next_;

        // Constructor
        motor_dji(CAN_HandleTypeDef
                  *hcan,
                  int16_t
                  max_current, bool angle_reserve = false
        ) :

                hcan_(hcan), max_current_(max_current) {
            target_current_ = 0;
            current_velocity_ = 0;
            current_round_ = 0;
            current_round_last_ = 0;
            current_angle_ = 0;
            current_angle_last_ = 0;
            last_update_time_ = xTaskGetTickCount();
            motor_state_ = DISCONNECTED;
            angle_reverse_ = angle_reserve;
        }

        bool set_current(int16_t target_current) {
            if (target_current > max_current_)
                target_current = max_current_;
            else if (target_current < -max_current_)
                target_current = -max_current_;

            if (motor_state_ == IDLE)
                motor_state_ = (RUNNING);
            else if (motor_state_ == DISCONNECTED)
                return false;

            target_current_ = target_current;
            return true;
        }

        bool set_next_state(E_MotorState state) {
            if (motor_state_ != DISCONNECTED) {
                motor_state_next_ = state;
                return true;
            } else return false;
        }

        void reset_round() {
            current_round_ = 0;
            current_round_last_ = 0;
        }

        void decode(CAN_RxHeaderTypeDef *rxHeader, const uint8_t *rxData) {
            // 标准帧
            if (rxHeader->IDE == CAN_ID_STD) {
                // 更新last
                current_angle_last_ = current_angle_;
                current_round_last_ = current_round_;

                if (angle_reverse_)
                    current_angle_ = 8191 - ((rxData[0] << 8) | rxData[1]);
                else
                    current_angle_ = (rxData[0] << 8) | rxData[1];

                current_velocity_ = (rxData[2] << 8) | rxData[3];

                last_update_time_ = xTaskGetTickCount();
                if (motor_state_ == DISCONNECTED && xTaskGetTickCount() - last_update_time_ < 1000) {
                    motor_state_ = (IDLE);
                }

                if (!first_decode_) {
                    first_decode_ = true;
                    return;
                }

                // 判断是否过零
                if ((current_angle_ < 2048) && (current_angle_last_ > 6144))
                    current_round_ += 1;
                else if ((current_angle_ > 6144) && (current_angle_last_ < 2048))
                    current_round_ -= 1;

            }
        }

        /**
         * @brief 更新电机状态，只有6020可用！2006电机需要生成电流控制帧
         * @return
         */
        bool update() {
            // Check Disconnect
            if (xTaskGetTickCount() - last_update_time_ > 1000) {
                motor_state_ = (DISCONNECTED);
            } else if (motor_state_ == DISCONNECTED) {
                motor_state_ = (IDLE);
            }

            // Update Motor State
            if (motor_state_ == RUNNING || motor_state_ == IDLE) {
                // Write CAN Data to hcan
                uint8_t data[8];
                static uint32_t txFifo = 0;
                // Write to Data[0], [1]
                update_can_array(data, 0, motor_state_ == IDLE ? 0 : target_current_);

                // 标准帧 0x2ff
                CAN_TxHeaderTypeDef txHeader;
                txHeader.DLC = 8;
                txHeader.StdId = 0x2fe;
                txHeader.IDE = CAN_ID_STD;
                txHeader.RTR = CAN_RTR_DATA;
                txHeader.TransmitGlobalTime = DISABLE;
                HAL_CAN_AddTxMessage(hcan_, &txHeader, data, &txFifo);
            }

            // update next_state
            if (motor_state_next_ != UNDEFINED && motor_state_ != DISCONNECTED) {
                motor_state_ = motor_state_next_;
                motor_state_next_ = UNDEFINED;
            } else if (motor_state_next_ != UNDEFINED && motor_state_ == DISCONNECTED) {
                motor_state_next_ = UNDEFINED;
            }
            return true;
        }
    };

    extern motor_dji MotorLS;
    extern motor_dji MotorY;
    extern motor_dji MotorDM;
    extern motor_vesc MotorFW[4];
}

#endif //DART_24_MOTOR_H