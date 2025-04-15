//
// Created by cheny on 24-6-30.
//
#include "dart_param.h"
#include "state_machine.h"

// Variables for debug mode params
double target_velocity_ratio = TARGET_VELOCITY_RATIO;
int32_t target_velocity_fw = TARGET_VELOCITY_FW;
int32_t target_velocity_fw_offset = TARGET_VELOCITY_FW_OFFSET;
uint32_t target_yaw_angle_with_rounds = TARGET_YAW_ANGLE_WITH_ROUNDS;
int32_t target_yaw_angle_with_rounds_offset = TARGET_YAW_ANGLE_WITH_ROUNDS_OFFSET;

// Variables for launch params
launch_params_t launch_params[LAUNCH_PARAMS_NUM] = {
        {LAUNCH_PARAMS_DEFAULT_TARGET_VELOCITY_FW_OFFSET_0,
                LAUNCH_PARAMS_DEFAULT_TARGET_YAW_ANGLE_WITH_ROUNDS_OFFSET_0},
        {LAUNCH_PARAMS_DEFAULT_TARGET_VELOCITY_FW_OFFSET_1,
                LAUNCH_PARAMS_DEFAULT_TARGET_YAW_ANGLE_WITH_ROUNDS_OFFSET_1},
        {LAUNCH_PARAMS_DEFAULT_TARGET_VELOCITY_FW_OFFSET_2,
                LAUNCH_PARAMS_DEFAULT_TARGET_YAW_ANGLE_WITH_ROUNDS_OFFSET_2},
        {LAUNCH_PARAMS_DEFAULT_TARGET_VELOCITY_FW_OFFSET_3,
                LAUNCH_PARAMS_DEFAULT_TARGET_YAW_ANGLE_WITH_ROUNDS_OFFSET_3}
};

void can_param_decode(CAN_RxHeaderTypeDef *rxHeader, const uint8_t *rxData) {
    // -0x711 参数设置 【待设置参数类型2字节+参数数据4字节+空2字节】
    uint16_t param_type = (rxData[0] << 8) | rxData[1];

    // 将param_type转成E_Dart_Can_Param_Index
    E_Dart_Can_Param_Index param_index = static_cast<E_Dart_Can_Param_Index>(param_type);
    switch (param_index) {
        case VELOCITY_RATIO: {
            // int32_t 除以 10000
            target_velocity_ratio = ((rxData[2] << 24) | (rxData[3] << 16) | (rxData[4] << 8) | rxData[5]) / 10000.0;
            break;
        }
        case VELOCITY_FW: {
            target_velocity_fw = (rxData[2] << 24) | (rxData[3] << 16) | (rxData[4] << 8) | rxData[5];
            break;
        }
        case VELOCITY_FW_OFFSET: {
            target_velocity_fw_offset = (rxData[2] << 24) | (rxData[3] << 16) | (rxData[4] << 8) | rxData[5];
            break;
        }
        case YAW_ANGLE_WITH_ROUNDS: {
            target_yaw_angle_with_rounds = (rxData[2] << 24) | (rxData[3] << 16) | (rxData[4] << 8) | rxData[5];
//            if (state_machine::dart_fsm.openFSM_.focusEState() != state_machine::Boot)
            motor_controller::pid_angle_velocity_MotorY.target_angle_with_rounds_ =
                    target_yaw_angle_with_rounds + target_yaw_angle_with_rounds_offset;
            break;
        }
        case YAW_ANGLE_WITH_ROUNDS_OFFSET: {
            target_yaw_angle_with_rounds_offset = (rxData[2] << 24) | (rxData[3] << 16) | (rxData[4] << 8) | rxData[5];
            motor_controller::pid_angle_velocity_MotorY.target_angle_with_rounds_ =
                    target_yaw_angle_with_rounds + target_yaw_angle_with_rounds_offset;
            break;
        }
        case LAUNCH_PARAMS_TARGET_VELOCITY_FW_OFFSET_0: {
            launch_params[0].target_velocity_fw_offset =
                    (rxData[2] << 24) | (rxData[3] << 16) | (rxData[4] << 8) | rxData[5];
            break;
        }
        case LAUNCH_PARAMS_TARGET_VELOCITY_FW_OFFSET_1: {
            launch_params[1].target_velocity_fw_offset =
                    (rxData[2] << 24) | (rxData[3] << 16) | (rxData[4] << 8) | rxData[5];
            break;
        }
        case LAUNCH_PARAMS_TARGET_VELOCITY_FW_OFFSET_2: {
            launch_params[2].target_velocity_fw_offset =
                    (rxData[2] << 24) | (rxData[3] << 16) | (rxData[4] << 8) | rxData[5];
            break;
        }
        case LAUNCH_PARAMS_TARGET_VELOCITY_FW_OFFSET_3: {
            launch_params[3].target_velocity_fw_offset =
                    (rxData[2] << 24) | (rxData[3] << 16) | (rxData[4] << 8) | rxData[5];
            break;
        }
        case LAUNCH_PARAMS_TARGET_YAW_ANGLE_WITH_ROUNDS_OFFSET_0: {
            launch_params[0].target_yaw_angle_with_rounds_offset =
                    (rxData[2] << 24) | (rxData[3] << 16) | (rxData[4] << 8) | rxData[5];
            break;
        }
        case LAUNCH_PARAMS_TARGET_YAW_ANGLE_WITH_ROUNDS_OFFSET_1: {
            launch_params[1].target_yaw_angle_with_rounds_offset =
                    (rxData[2] << 24) | (rxData[3] << 16) | (rxData[4] << 8) | rxData[5];
            break;
        }
        case LAUNCH_PARAMS_TARGET_YAW_ANGLE_WITH_ROUNDS_OFFSET_2: {
            launch_params[2].target_yaw_angle_with_rounds_offset =
                    (rxData[2] << 24) | (rxData[3] << 16) | (rxData[4] << 8) | rxData[5];
            break;
        }
        case LAUNCH_PARAMS_TARGET_YAW_ANGLE_WITH_ROUNDS_OFFSET_3: {
            launch_params[3].target_yaw_angle_with_rounds_offset =
                    (rxData[2] << 24) | (rxData[3] << 16) | (rxData[4] << 8) | rxData[5];
            break;
        }
        default: {
            break;
        }
    }
}