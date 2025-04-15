//
// Created by cheny on 24-6-30.
//
#include "state_machine.h"
#include "motor.h"
#include "controller.h"
#include "dart_param.h"
#include "dbus.h"
#include "judge_receive.h"
#include "tim.h"
#include "can.h"
#include "cstring"

namespace state_machine {
    void dart_informational_update();

    // Flag区域
    bool buzzer_error_on_ = false; // true表示任意状态下都激活蜂鸣器警告，表示电机运行异常
    uint8_t buzzer_error_mode_ = 0; // true表示任意状态下都激活蜂鸣器警告，表示电机运行异常
    bool buzzer_info_on_ = false; // true表示激活一次蜂鸣器提示，表示某一动作完成，蜂鸣完成后自动取消标志
    E_Lead_Screw_Switch_State lead_scew_switch_down_value_; // 丝杆开关状态
    bool move_motor_to_bottom_running_flag_yaw; // 电机移动到底限位，正在运行标志位，有两个槽位可用，一般来说用第一个，比赛模式需要使用两个
    bool move_motor_to_bottom_running_flag_dm; // 电机移动到底限位，正在运行标志位
    bool move_motor_to_bottom_running_flag_lead_screw; // 电机移动到底限位，正在运行标志位
    bool moved_to_bottom_flag_1; // 电机移动到底限位，完成标志位，用于状态转换，有两个槽位可用，一般来说用第一个，比赛模式需要使用两个
    bool moved_to_bottom_flag_2; // 电机移动到底限位，完成标志位，用于状态转换
    uint8_t state_match_launch_counter_ = 0;
    E_Match_Actions state_match_action_ = E_Match_Actions::Undefined;
    TickType_t rc_action_last_time_; // 遥控器数据在Action内按频率读取时间戳
    bool waiting_for_fw_stable_flag_ = false; // 等待摩擦轮稳定标志
    TickType_t waiting_for_fw_stable_last_time_; // 等待摩擦轮稳定时间戳

    TickType_t buzzer_period_last_on_time_; // 蜂鸣器提示持续时间戳

    bool actionRemote_fw_on_flag_ = false; // 遥控器调试模式摩擦轮启动标志
    uint8_t last_left_switch_status_ = 0; // 上一次左侧拨杆状态

    // 裁判系统判定Flag
    uint8_t last_dart_gate_opening_status_ = 0; // 上一次发射状态
    uint16_t last_dart_launch_time_ = 0;    // 上一次发射指令下达时间
    bool match_flag_ = 0; // 上场比赛判断 若已经上场则执行最严格的安全措施

    // 初次启动位置
    int32_t first_boot_angle_with_round;


    void Dart_FSM::start() {
        openFSM_.setCustom(this);
        openFSM_.setStates({E_Dart_State::Boot, E_Dart_State::Protect, E_Dart_State::Remote, E_Dart_State::Match});

        openFSM_.enterState(E_Dart_State::Boot);
    }

    void Dart_FSM::update() {
        dart_informational_update();
        // 状态机更新
        openFSM_.update();
    }

    Dart_FSM dart_fsm;

    // 非阻塞电机移动到底限位，返回值表示是否执行完成
    template<typename TypeTarget, typename TypeGate, typename TypeController>
    bool move_motor_to_bottom_limit(motor_controller::pid_angle_velocity_controller<TypeController> &controller_,
                                    TypeTarget operation_target_, TypeGate gate_velocity_,
                                    TickType_t timeout_, bool &running_flag_,
                                    bool openloop_ = false) {

        static TickType_t last_time;
        if (!running_flag_) { // 开始运行
            last_time = xTaskGetTickCount();
            if (openloop_) {
                controller_.
                        set_state(motor_controller::E_PID_Velocity_Angle_Controller_State::OPEN_LOOP);
                controller_.
                        target_openloop_ = operation_target_;
            } else {
                controller_.
                        set_state(motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
                controller_.
                        target_velocity_ = operation_target_;
            }
            running_flag_ = true;
            return false; // 未完成
        } else { // 运行中
            if (abs(controller_.motor_->current_velocity_) <= abs(gate_velocity_)) {
                if (xTaskGetTickCount() - last_time > timeout_) {
                    // 停止电机，复原状态
                    if (openloop_)
                        controller_.target_openloop_ = 0;
                    else
                        controller_.target_velocity_ = 0;
                    controller_.motor_->set_next_state(motor::E_MotorState::IDLE);
                    running_flag_ = false;
                    return true; // 完成
                }
            } else {
                controller_.
                        target_velocity_ = operation_target_;
                last_time = xTaskGetTickCount();
            }
            return false; // 未完成
        }
    }

    void little_to_big_endian(uint8_t *data, uint8_t size) {
        for (uint8_t i = 0; i < size / 2; i++) {
            uint8_t temp = data[i];
            data[i] = data[size - i - 1];
            data[size - i - 1] = temp;
        }
    }

    void dart_informational_update() {
        // 负责与上位机通信、更新硬件开关状态、发布蜂鸣器警告
        // 读取限位开关值

//        lead_scew_switch_down_value_ =
//                HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_SET ? E_Lead_Screw_Switch_State::Untriggered
//                                                                     : E_Lead_Screw_Switch_State::Triggered;
        // 消抖读取限位开关值
        static uint8_t lead_screw_switch_down_value_counter = 0;
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_SET) {
            lead_screw_switch_down_value_counter = 0;
            lead_scew_switch_down_value_ = E_Lead_Screw_Switch_State::Untriggered;
        } else {
            if (lead_screw_switch_down_value_counter < 10)
                lead_screw_switch_down_value_counter++;
            else
                lead_scew_switch_down_value_ = E_Lead_Screw_Switch_State::Triggered;
        }


        // 非比赛模式/启动模式下，如果遥控器断联，即刻进入保护
        if (xTaskGetTickCount() - RC_Data.last_update_time > pdMS_TO_TICKS(1000)) {
            if (dart_fsm.openFSM_.focusEState() != E_Dart_State::Match &&
                dart_fsm.openFSM_.focusEState() != E_Dart_State::Boot &&
                dart_fsm.openFSM_.focusEState() != E_Dart_State::Protect) {
                dart_fsm.openFSM_.nextState(E_Dart_State::Protect);
            }
            buzzer_error_on_ = true;
        }

        // 遥控看门狗
        static TickType_t last_reset_tick = xTaskGetTickCount();
        if (xTaskGetTickCount() - RC_Data.last_update_time > pdMS_TO_TICKS(5) &&
            xTaskGetTickCount() - last_reset_tick > pdMS_TO_TICKS(200)) {
            DT7_Reset();
            last_reset_tick = xTaskGetTickCount();
        }

        static TickType_t last_reset_tick_judge_judge = xTaskGetTickCount();
        if (xTaskGetTickCount() - ext_judge_last_receive_time > pdMS_TO_TICKS(5) &&
            xTaskGetTickCount() - last_reset_tick_judge_judge > pdMS_TO_TICKS(200)) {
            judge_Reset();
            last_reset_tick_judge_judge = xTaskGetTickCount();
        }


        // TODO:使用buzzer类状态机重构
        // Buzzer Warning
        if (buzzer_error_on_) {
            if (buzzer_error_mode_ == 0)
                if (xTaskGetTickCount() % pdMS_TO_TICKS(4000) < pdMS_TO_TICKS(500)) {
                    // 响
                    __HAL_TIM_SET_AUTORELOAD(&htim4, BUZZER_WARNING_TIM_RELOAD);
                    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, BUZZER_WARNING_VOLUME);
                } else {
                    // 占空比停止
                    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
                }
            else if (buzzer_error_mode_ == 1)
                if (xTaskGetTickCount() % pdMS_TO_TICKS(1000) < pdMS_TO_TICKS(100)) {
                    // 响
                    __HAL_TIM_SET_AUTORELOAD(&htim4, BUZZER_WARNING_TIM_RELOAD);
                    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, BUZZER_WARNING_VOLUME);
                } else {
                    // 占空比停止
                    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
                }

        } else
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);

        if (buzzer_info_on_ || buzzer_period_last_on_time_ != 0) {
            if (buzzer_info_on_) {
                buzzer_period_last_on_time_ = xTaskGetTickCount();
                buzzer_info_on_ = false;
            }
            if (xTaskGetTickCount() - buzzer_period_last_on_time_ < pdMS_TO_TICKS(500)) {
                // 响
                __HAL_TIM_SET_AUTORELOAD(&htim4, BUZZER_INFO_TIM_RELOAD);
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, BUZZER_INFO_VOLUME);
            } else {
                // 占空比停止
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
                buzzer_period_last_on_time_ = 0;
            }
        }

        // State Broadcast
        //  一 0X700 系统状态1 &发射参数 1 【电机和遥控器在线位 8位 [1字节]+
        //  模式位 1字节+堵转过程状态1字节+发射过程状态1字节+摩擦轮Target4字节】
        //  一 0X701 发射参数2【摩擦轮 TargetOffset4 字节 +Yaw角度 4字节】
        //  一 0X702 发射参数3 【Yaw 轴角度 Offset4 字节+ Yaw轴真实角度 4字节】
        //  一 0X703 发射参数4 【第一槽位 Yaw 角度Offset + 第一槽位摩擦轮Offset】
        //  一 0X704 发射参数5 【第二槽位 Yaw 角度Offset + 第二槽位摩擦轮Offset】
        //  一 0X705 发射参数6 【第三槽位 Yaw 角度Offset + 第三槽位摩擦轮Offset】
        //  一 0X706 发射参数7 【第四槽位 Yaw 角度Offset + 第四槽位摩擦轮Offset】
        //  一 0X707 发射参数8 【摩擦轮速度比】 double*10000
        //  一 0X708 裁判日志【ext_dart_client_cmd.dart_launch_opening_status +
        //  ext_game_status.game_progress +
        //  ext_dart_info.dart_remaining_time +
        //  ext_dart_client_cmd.latest_launch_cmd_time +
        //  ext_game_status.stage_remain_time + 在线判断位】
        static uint8_t status_send_prescaler = 0; // 状态发送预分频器，降低频率到50Hz
        static uint8_t status_send_state = 0; // 状态发送状态机
        if (++status_send_prescaler >= 50) {
            status_send_prescaler = 0;

            // Can Send
            static uint32_t tx_mailbox;
            // Can Frame
            CAN_TxHeaderTypeDef tx_header;
            tx_header.StdId = 0x700 + status_send_state;
            tx_header.IDE = CAN_ID_STD;
            tx_header.RTR = CAN_RTR_DATA;
            tx_header.DLC = 8;
            tx_header.TransmitGlobalTime = DISABLE;

            uint8_t can_array[8];
            switch (status_send_state) {
                case 0:
                    // 系统状态1 & 发射参数 1
                    // 电机和遥控器在线位 8位 [1字节] + 模式位 1字节 + 堵转过程状态 1字节 + 发射过程状态 1字节 + 摩擦轮 Target 4 字节
                    can_array[0] = (motor::MotorLS.motor_state_ != motor::DISCONNECTED) << 0 |
                                   (motor::MotorY.motor_state_ != motor::DISCONNECTED) << 1 |
                                   (motor::MotorDM.motor_state_ != motor::DISCONNECTED) << 2 |
                                   (motor::MotorFW[0].motor_state_ != motor::DISCONNECTED) << 3 |
                                   (motor::MotorFW[1].motor_state_ != motor::DISCONNECTED) << 4 |
                                   (motor::MotorFW[2].motor_state_ != motor::DISCONNECTED) << 5 |
                                   (motor::MotorFW[3].motor_state_ != motor::DISCONNECTED) << 6 |
                                   (xTaskGetTickCount() - RC_Data.last_update_time < pdMS_TO_TICKS(500)) << 7;
                    can_array[1] = dart_fsm.openFSM_.focusEState();
                    if (can_array[1] == E_Dart_State::Match)
                        can_array[1] += state_match_action_;
                    can_array[2] = move_motor_to_bottom_running_flag_yaw << 0 |
                                   move_motor_to_bottom_running_flag_dm << 1 |
                                   move_motor_to_bottom_running_flag_lead_screw << 2;
                    can_array[3] = state_match_launch_counter_;
                    memcpy(&can_array[4], &target_velocity_fw, sizeof(target_velocity_fw));
                    little_to_big_endian(can_array + 4, sizeof(target_velocity_fw));
                    break;
                case 1:
                    // 发射参数2
                    // 摩擦轮 TargetOffset4 字节 +Yaw 角度 4 字节
                    memcpy(&can_array[0], &target_velocity_fw_offset, 4);
                    little_to_big_endian(can_array, 4);
                    memcpy(&can_array[4], &target_yaw_angle_with_rounds, 4);
                    little_to_big_endian(can_array + 4, 4);
                    break;
                case 2: {
                    int32_t yaw_angle_with_rounds_offset =
                            motor::MotorY.current_round_ * 8192 + motor::MotorY.current_angle_;
                    // 发射参数3
                    // Yaw 轴角度 0ffset4 字节+ Yaw 轴真实角度 4 字节
                    memcpy(&can_array[0], &target_yaw_angle_with_rounds_offset, 4);
                    little_to_big_endian(can_array, 4);
                    memcpy(&can_array[4], &(yaw_angle_with_rounds_offset), 4);
                    little_to_big_endian(can_array + 4, 4);
                    break;
                }
                case 3:
                    // 发射参数4
                    // 第一槽位 Yaw 角度 0ffset+ 第一槽位摩擦轮 0ffset
                    memcpy(&can_array[0], &launch_params[0].target_yaw_angle_with_rounds_offset, 4);
                    little_to_big_endian(can_array, 4);
                    memcpy(&can_array[4], &launch_params[0].target_velocity_fw_offset, 4);
                    little_to_big_endian(can_array + 4, 4);
                    break;
                case 4:
                    // 发射参数5
                    // 第二槽位 Yaw 角度 0ffset +第二槽位摩擦轮 0ffset
                    memcpy(&can_array[0], &launch_params[1].target_yaw_angle_with_rounds_offset, 4);
                    little_to_big_endian(can_array, 4);
                    memcpy(&can_array[4], &launch_params[1].target_velocity_fw_offset, 4);
                    little_to_big_endian(can_array + 4, 4);
                    break;
                case 5:
                    // 发射参数6
                    // 第三槽位 Yaw 角度 0ffset +第三槽位摩擦轮 0ffset
                    memcpy(&can_array[0], &launch_params[2].target_yaw_angle_with_rounds_offset, 4);
                    little_to_big_endian(can_array, 4);
                    memcpy(&can_array[4], &launch_params[2].target_velocity_fw_offset, 4);
                    little_to_big_endian(can_array + 4, 4);
                    break;
                case 6:
                    // 发射参数7
                    // 第四槽位 Yaw 角度 0ffset +第四槽位摩擦轮 0ffset
                    memcpy(&can_array[0], &launch_params[3].target_yaw_angle_with_rounds_offset, 4);
                    little_to_big_endian(can_array, 4);
                    memcpy(&can_array[4], &launch_params[3].target_velocity_fw_offset, 4);
                    little_to_big_endian(can_array + 4, 4);
                    break;
                case 7:
                    // 发射参数8
                    // 摩擦轮速度比
                {
                    int32_t can_target_velocity_ratio = int32_t(target_velocity_ratio * 10000);
                    can_array[0] = can_target_velocity_ratio >> 24 & 0xFF;
                    can_array[1] = can_target_velocity_ratio >> 16 & 0xFF;
                    can_array[2] = can_target_velocity_ratio >> 8 & 0xFF;
                    can_array[3] = can_target_velocity_ratio & 0xFF;
                    memset(&can_array[4], 0, 4); // 剩下四位填充0
                    break;
                }
                case 8:
                    // 裁判日志
                    // ext dart client cmd. dart launch opening_status+ ext_game status. game_progres S + ext dart info. dart remaining_time + ext dart client cmd. latest launch cmd time+ ext_game_status. stage_remain time +断联判断位
                    can_array[0] = ext_dart_client_cmd.dart_launch_opening_status;
                    can_array[1] = ext_game_status.game_progress;
                    can_array[2] = ext_dart_info.dart_remaining_time;
                    memcpy(&can_array[3], &ext_dart_client_cmd.latest_launch_cmd_time,
                           sizeof(ext_dart_client_cmd.latest_launch_cmd_time)); // 2字节
                    little_to_big_endian(can_array + 3, sizeof(ext_dart_client_cmd.latest_launch_cmd_time));
                    memcpy(&can_array[5], &ext_game_status.stage_remain_time,
                           sizeof(ext_game_status.stage_remain_time)); // 2字节
                    little_to_big_endian(can_array + 5, sizeof(ext_game_status.stage_remain_time));
                    can_array[7] = (xTaskGetTickCount() - ext_judge_last_receive_time) < pdMS_TO_TICKS(500);
                    break;
                default:
                    break;
            }
            status_send_state = (status_send_state + 1) % 9;
            HAL_CAN_AddTxMessage(&hcan2, &tx_header, can_array, &tx_mailbox);
        }

        // 比赛上场判断
        if (ext_game_status.game_progress != 0)
            match_flag_ = 1;
    }

    void
    set_motor_next_state_with_disconnect_detection(motor::E_MotorState state_LS, motor::E_MotorState state_Y,
                                                   motor::E_MotorState state_DM, motor::E_MotorState state_FW) {
        buzzer_error_on_ = false;
        buzzer_error_on_ |= !motor::MotorLS.set_next_state(state_LS);
        buzzer_error_on_ |= !motor::MotorY.set_next_state(state_Y);
        buzzer_error_on_ |= !motor::MotorDM.set_next_state(state_DM);
        for (auto &i: motor::MotorFW)
            buzzer_error_on_ |= !i.set_next_state(state_FW);
    }

    void set_next_state_by_RC() {
        // 通过遥控器设置状态机状态
        E_Dart_State next_state = E_Dart_State::Protect;
        // 遥控器数据有效
        if (xTaskGetTickCount() - RC_Data.last_update_time < pdMS_TO_TICKS(500)) {
            if (RC_Data.Switch_Right == RC_SW_MID) {
                next_state = E_Dart_State::Remote;
            } else if (RC_Data.Switch_Right == RC_SW_DOWN) {
                next_state = E_Dart_State::Match;
            }
        } else if (dart_fsm.openFSM_.focusEState() == E_Dart_State::Match)
            next_state = E_Dart_State::Match;

        if (next_state != dart_fsm.openFSM_.focusEState()) {
            dart_fsm.openFSM_.nextState(next_state);
        }
    }

    class ActionWaitForAllMotorOnline : public OpenFSMAction {
    public:
        void update(OpenFSM &fsm) const override {
            // 等待电机全部上线
            if (motor::MotorLS.motor_state_ == motor::E_MotorState::DISCONNECTED ||
                motor::MotorY.motor_state_ == motor::E_MotorState::DISCONNECTED ||
                motor::MotorDM.motor_state_ == motor::E_MotorState::DISCONNECTED ||
                motor::MotorFW[0].motor_state_ == motor::E_MotorState::DISCONNECTED ||
                motor::MotorFW[1].motor_state_ == motor::E_MotorState::DISCONNECTED ||
                motor::MotorFW[2].motor_state_ == motor::E_MotorState::DISCONNECTED ||
                motor::MotorFW[3].motor_state_ == motor::E_MotorState::DISCONNECTED) {
                buzzer_error_on_ = true;
                buzzer_error_mode_ = 1; // 短鸣
            } else
                fsm.nextAction();
        }

        void exit(OpenFSM &fsm) const override {
            // Set Motor State: MotorLS -> IDLE, MotorY -> IDLE, MotorDM -> IDLE, MotorFW -> IDLE
            set_motor_next_state_with_disconnect_detection(motor::E_MotorState::IDLE, motor::E_MotorState::IDLE,
                                                           motor::E_MotorState::IDLE, motor::E_MotorState::IDLE);
            buzzer_error_on_ = false;
            buzzer_error_mode_ = 0;
            buzzer_info_on_ = true;
        }
    };

    class ActionResetLSandYaw : public OpenFSMAction {
    public:
        void enter(OpenFSM &fsm) const override {
            // Set Motor State: MotorLS -> RUNNING, MotorY -> RUNNING, MotorDM -> IDLE, MotorFW -> IDLE
            set_motor_next_state_with_disconnect_detection(motor::E_MotorState::RUNNING,
                                                           motor::E_MotorState::RUNNING,
                                                           motor::E_MotorState::IDLE,
                                                           motor::E_MotorState::IDLE);

            moved_to_bottom_flag_1 = false;
//            first_boot_angle_with_round = motor::MotorY.current_round_ * 8192 + motor::MotorY.current_angle_;
        }

        void update(OpenFSM &fsm) const override {

            // Reset Yaw & Lead Screw
            if (!moved_to_bottom_flag_1)
                if (move_motor_to_bottom_limit<>(motor_controller::pid_angle_velocity_MotorY,
                                                 YAW_RIGHT_VELOCITY,
                                                 YAW_RIGHT_GATE_VELOCITY, YAW_RIGHT_BLOCK_TIMEOUT,
                                                 move_motor_to_bottom_running_flag_yaw, false)) { // flag结束后自动归零
                    moved_to_bottom_flag_1 = true;
                    buzzer_info_on_ = true;
                    // 切角度闭环
//                    int32_t current_angle_with_rounds =
//                            motor::MotorY.current_round_ * 8192 + motor::MotorY.current_angle_;
                    motor::MotorY.reset_round();

                    motor::MotorY.set_next_state(motor::E_MotorState::RUNNING);
                    motor_controller::pid_angle_velocity_MotorY.target_angle_with_rounds_ = // 回到初始位置
                            target_yaw_angle_with_rounds + target_yaw_angle_with_rounds_offset;
                    motor_controller::pid_angle_velocity_MotorY.set_state(
                            motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);

                }
            if (reset_LS() && moved_to_bottom_flag_1 &&
                abs(motor::MotorY.current_round_ * 8192 + motor::MotorY.current_angle_ -
                    motor_controller::pid_angle_velocity_MotorY.target_angle_with_rounds_) < 10)
                fsm.nextAction();

        }

        static bool reset_LS() {
            if (lead_scew_switch_down_value_ == E_Lead_Screw_Switch_State::Untriggered) {
                // 丝杆电机下降
                motor_controller::pid_velocity_MotorLS.target_velocity_ = LEAD_SCREW_DOWN_VELOCITY; // RPM
                return false;
            } else {
                // 丝杆电机停止并归零
                motor_controller::pid_velocity_MotorLS.target_velocity_ = 0;
                motor::MotorLS.reset_round();
                motor::MotorLS.set_next_state(motor::E_MotorState::IDLE);
                return true;
            }
        }
    };

    class ActionResetDM : public OpenFSMAction {
    public:
        void enter(OpenFSM &fsm) const override {
            // Set Motor State: MotorLS -> IDLE, MotorY -> IDLE, MotorDM -> RUNNING, MotorFW -> IDLE
            set_motor_next_state_with_disconnect_detection(motor::E_MotorState::IDLE, motor::E_MotorState::IDLE,
                                                           motor::E_MotorState::RUNNING,
                                                           motor::E_MotorState::IDLE);
            move_motor_to_bottom_running_flag_dm = false; // 重置标志位
        }

        void update(OpenFSM &fsm) const override {
            // 向右侧归零
            if (move_motor_to_bottom_limit<>(motor_controller::pid_velocity_MotorDM,
                                             DRUM_MAGAZINE_RIGHT_VELOCITY,
                                             DRUM_MAGAZINE_RIGHT_GATE_VELOCITY, DRUM_MAGAZINE_BLOCK_TIMEOUT,
                                             move_motor_to_bottom_running_flag_dm, false))
                fsm.nextAction(); // 闭环归零，完成后进入下一个Action
        }

        void exit(OpenFSM &fsm) const override {
            // Set Motor State: MotorDM -> IDLE
            move_motor_to_bottom_running_flag_dm = false; // 重置标志位，以防动作未完成

            // GPIO Interface
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // 激光开关
        }
    };

// 保护模式Action
    class ActionProtect : public OpenFSMAction {
    public:
        void enter(OpenFSM &fsm) const override {
            // 关闭激光器
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
        }

        void update(OpenFSM &fsm) const override {
            // 保护模式下，所有电机停止
//            for (auto &i: motor::MotorFW)
//                i.target_velocity_ = 0;
            set_motor_next_state_with_disconnect_detection(motor::E_MotorState::IDLE, motor::E_MotorState::IDLE,
                                                           motor::E_MotorState::IDLE,
                                                           motor::E_MotorState::IDLE);
            set_next_state_by_RC();
        }

        void exit(OpenFSM &fsm) const override {
            // 重启激光器
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
        }
    };

// 调试模式Action
    class ActionRemote : public OpenFSMAction {
    public:
        virtual void enter(OpenFSM &fsm) {
            // Set Motor State: MotorLS -> RUNNING, MotorY -> RUNNING, MotorDM -> RUNNING, MotorFW -> RUNNING
            set_motor_next_state_with_disconnect_detection(motor::E_MotorState::RUNNING,
                                                           motor::E_MotorState::RUNNING,
                                                           motor::E_MotorState::RUNNING,
                                                           motor::E_MotorState::RUNNING);

            // Set Control Mode
            motor_controller::pid_velocity_MotorLS.set_state(
                    motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
            motor_controller::pid_angle_velocity_MotorY.set_state(
                    motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);
            motor_controller::pid_velocity_MotorDM.set_state(
                    motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);

            rc_action_last_time_ = xTaskGetTickCount();

            // 堵转状态
            move_motor_to_bottom_running_flag_yaw = false; // Yaw轴使用
            move_motor_to_bottom_running_flag_dm = false; // 弹鼓电机使用
            move_motor_to_bottom_running_flag_lead_screw = false; // 丝杆电机使用

            // 摩擦轮启动标志
            actionRemote_fw_on_flag_ = false;
            last_left_switch_status_ = RC_Data.Switch_Left;
        }

        void update(OpenFSM &fsm) const override {
            // Set Motor State: MotorLS -> RUNNING, MotorY -> RUNNING, MotorDM -> RUNNING, MotorFW -> RUNNING
            set_motor_next_state_with_disconnect_detection(motor::E_MotorState::RUNNING,
                                                           motor::E_MotorState::RUNNING,
                                                           motor::E_MotorState::RUNNING,
                                                           motor::E_MotorState::RUNNING);

            // 根据RC数据设置电机速度、是否进入堵转模式等
            // 此模式下，RC拨轮控制摩擦轮速度增减，RC右摇杆Y方向控制丝杆电机运转，X方向控制弹鼓电机运转，RC左摇杆X方向控制YAW轴横移
            if (actionRemote_fw_on_flag_) {   // 时间域：10Hz
                if (xTaskGetTickCount() - rc_action_last_time_ > pdMS_TO_TICKS(100)) {
                    rc_action_last_time_ = xTaskGetTickCount();
                    // 处理摩擦轮速度增减
                    // <--- 514 --- 939 --- 中点 --- 1121 --- 1622 --->
                    // <+20     +1                        -1      -20>
                    if (RC_Data.ch4_wheel <= 514)
                        target_velocity_fw += 20;
                    else if (RC_Data.ch4_wheel > 514 && RC_Data.ch4_wheel <= 939)
                        target_velocity_fw += 1;
                    else if (RC_Data.ch4_wheel >= 1121 && RC_Data.ch4_wheel < 1622)
                        target_velocity_fw -= 1;
                    else if (RC_Data.ch4_wheel >= 1622)
                        target_velocity_fw -= 20;

                    // 摩擦轮速度限幅
                    if (target_velocity_fw > 7072)
                        target_velocity_fw = 7072;
                    else if (target_velocity_fw < 0)
                        target_velocity_fw = 0;

                    // 更新摩擦轮速度
                    for (int i = 0; i < 2; i++)
                        motor::MotorFW[i].target_velocity_ = target_velocity_fw + target_velocity_fw_offset;
                    for (int i = 2; i < 4; i++)
                        motor::MotorFW[i].target_velocity_ = int32_t(
                                double(target_velocity_fw + target_velocity_fw_offset) * target_velocity_ratio);


                }
            } else {
                for (auto &i: motor::MotorFW)
                    i.target_velocity_ = 0;
            }

            // 摩擦轮开关
            if (last_left_switch_status_ != RC_Data.Switch_Left) {
                last_left_switch_status_ = RC_Data.Switch_Left;
                if (RC_Data.Switch_Left == RC_SW_DOWN) {
                    actionRemote_fw_on_flag_ = !actionRemote_fw_on_flag_;
                }
            }

            // 摩擦轮开关门控
            if (ext_game_status.game_progress == 1 || ext_game_status.game_progress == 2 ||
                ext_game_status.game_progress == 3 || ext_game_status.game_progress == 5 || match_flag_)
                actionRemote_fw_on_flag_ = false;

            // 时间域:1000Hz
            // 处理丝杆电机速度
            // < --- 950 --- 中点 --- 1400 --- >
            // <正速度                     负速度>
            if (!move_motor_to_bottom_running_flag_lead_screw) {
                if (RC_Data.ch3 > 366 && RC_Data.ch3 <= 950 &&
                    lead_scew_switch_down_value_ == E_Lead_Screw_Switch_State::Untriggered)
                    motor_controller::pid_velocity_MotorLS.
                            target_velocity_ = -8000;
                else if (RC_Data.ch3 >= 1400 && motor::MotorLS.current_round_ <= SECOND_LAUNCH_POSITION_ROUNDS)
                    motor_controller::pid_velocity_MotorLS.
                            target_velocity_ = 10000;
                else if (RC_Data.ch3 <= 366)
                    move_motor_to_bottom_running_flag_lead_screw = true;
                else
                    motor_controller::pid_velocity_MotorLS.
                            target_velocity_ = 0;
            } else {
                // 丝杆重置
                if (ActionResetLSandYaw::reset_LS() || RC_Data.ch3 >= 1600) // 摇杆向上取消堵转
                    move_motor_to_bottom_running_flag_lead_screw = false;
            }

            // 处理弹鼓电机速度，丝杆运动时屏蔽弹鼓电机
            // < --- 700 --- 中点 --- 1400 --- >
            // <正速度                     负速度>
            if (RC_Data.ch2 <= 700 &&
                motor_controller::pid_velocity_MotorLS.target_velocity_ == 0) {
                motor_controller::pid_velocity_MotorDM.
                        target_velocity_ = 3000;
            } else if (RC_Data.ch2 >= 1400 && motor_controller::pid_velocity_MotorLS.target_velocity_ == 0) {
                motor_controller::pid_velocity_MotorDM.
                        target_velocity_ = -3000;
            } else {
                motor_controller::pid_velocity_MotorDM.
                        target_velocity_ = 0;
            }


//            // 堵转管理器
//            if (RC_Data.Switch_Left == RC_SW_UP && !moved_to_bottom_flag_1) {
//                // 电机移动到底限位
//                // 设置电机为速度控制模式
//                if (!move_motor_to_bottom_running_flag_yaw)
//                    // 相当于堵转状态enter
//                    motor_controller::pid_velocity_MotorDM.set_state(
//                            motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
//                // 相当于update
//                if (move_motor_to_bottom_limit<>(motor_controller::pid_angle_velocity_MotorY,
//                                                 YAW_RIGHT_VELOCITY,
//                                                 YAW_RIGHT_GATE_VELOCITY,
//                                                 YAW_RIGHT_BLOCK_TIMEOUT, move_motor_to_bottom_running_flag_yaw,
//                                                 false)) {
//
//                    motor::MotorY.reset_round();
//                    // 恢复为角度控制模式
//                    motor_controller::pid_angle_velocity_MotorY.target_angle_with_rounds_ =
//                            target_yaw_angle_with_rounds + target_yaw_angle_with_rounds_offset;
//
//                    motor_controller::pid_angle_velocity_MotorY.set_state(
//                            motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);
//                    // 相当于exit
//                    moved_to_bottom_flag_1 = true;
//                    // 蜂鸣一次
//                    buzzer_info_on_ = true;
//                }
//            } else if (RC_Data.Switch_Left != RC_SW_UP) {
//                moved_to_bottom_flag_1 = false; // 取消移动，复位标志位并不允许继续运行
//                if (move_motor_to_bottom_running_flag_yaw) {
//                    move_motor_to_bottom_running_flag_yaw = false; // 重置标志位
//                    // 角度控制
//                    motor_controller::pid_angle_velocity_MotorY.set_state(
//                            motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);
//                }
//            }

            // 处理Yaw轴横移，如果在堵转模式下，不允许横移
            // <--- 700 --- 900 --- 中点 --- 1100 --- 1310 --->
            // <+
            // 100    +10                      -10      -100>
            // 如果有速度，则转为速度控制模式，否则转为位置控制模式
//            if (!move_motor_to_bottom_running_flag_yaw) {
            if (RC_Data.ch0 > 900 && RC_Data.ch0 < 1100) {
                if (motor_controller::pid_angle_velocity_MotorY.state_ !=
                    motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL) {
                    motor_controller::pid_angle_velocity_MotorY.set_state(
                            motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);
                    motor_controller::pid_angle_velocity_MotorY.target_angle_with_rounds_ =
                            motor::MotorY.current_round_ * 8192 + motor::MotorY.current_angle_;
                    target_yaw_angle_with_rounds = motor_controller::pid_angle_velocity_MotorY.target_angle_with_rounds_;
                }
            } else {
                if (motor_controller::pid_angle_velocity_MotorY.state_ !=
                    motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL)
                    motor_controller::pid_angle_velocity_MotorY.set_state(
                            motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
                if (RC_Data.ch0 <= 700)
                    motor_controller::pid_angle_velocity_MotorY.target_velocity_ = -200;
                else if (RC_Data.ch0 > 700 && RC_Data.ch0 <= 900)
                    motor_controller::pid_angle_velocity_MotorY.target_velocity_ = -10;
                else if (RC_Data.ch0 >= 1100 && RC_Data.ch0 < 1310)
                    motor_controller::pid_angle_velocity_MotorY.target_velocity_ = 10;
                else if (RC_Data.ch0 >= 1310)
                    motor_controller::pid_angle_velocity_MotorY.target_velocity_ = 200;
            }
//            }

            // 刷新遥控模式
            set_next_state_by_RC();
        }

        void exit(OpenFSM &fsm) const

        override {
            // 重置标志位
            move_motor_to_bottom_running_flag_yaw = false;
            moved_to_bottom_flag_1 = false;
        }
    };

    class ActionMatch_Enter : public OpenFSMAction {
    public:
        void enter(OpenFSM &fsm) const override {
            // 跟上电复位差不多，但是要先复位丝杆、再移动弹鼓电机，同时执行yaw轴堵转，等待电机复位完成
            // Set Motor State: MotorLS -> IDLE, MotorY -> IDLE, MotorDM -> IDLE, MotorFW -> IDLE
            set_motor_next_state_with_disconnect_detection(motor::E_MotorState::RUNNING,
                                                           motor::E_MotorState::RUNNING,
                                                           motor::E_MotorState::RUNNING,
                                                           motor::E_MotorState::IDLE);

            // 速度控制
            motor_controller::pid_angle_velocity_MotorY.set_state(
                    motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);

            move_motor_to_bottom_running_flag_yaw = false; // 重置标志位
            moved_to_bottom_flag_1 = true; // 重置标志位

            move_motor_to_bottom_running_flag_dm = false; // 重置标志位
            moved_to_bottom_flag_2 = false; // 重置标志位

            // 重置发射计数
            state_match_launch_counter_ = 0;

            // 设置Action计数
            state_match_action_ = E_Match_Actions::Enter;
        }

        void update(OpenFSM &fsm) const override {
            // Yaw电机归零
//            if (!moved_to_bottom_flag_1) {
//                if (move_motor_to_bottom_limit<>(motor_controller::pid_angle_velocity_MotorY,
//                                                 YAW_RIGHT_VELOCITY,
//                                                 YAW_RIGHT_GATE_VELOCITY, YAW_RIGHT_BLOCK_TIMEOUT,
//                                                 move_motor_to_bottom_running_flag_yaw, false)) { // flag结束后自动归零
//                    moved_to_bottom_flag_1 = true;
//                    buzzer_info_on_ = true;
//                    // 切角度闭环
//                    motor::MotorY.reset_round();
//                    motor::MotorY.set_next_state(motor::E_MotorState::RUNNING);
//                    motor_controller::pid_angle_velocity_MotorY.target_angle_with_rounds_ =
//                            target_yaw_angle_with_rounds + target_yaw_angle_with_rounds_offset;
//                    motor_controller::pid_angle_velocity_MotorY.set_state(
//                            motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);
//
//                }
//            }

            reset_LS_and_DM(moved_to_bottom_flag_2, move_motor_to_bottom_running_flag_dm,
                            DRUM_MAGAZINE_RIGHT_VELOCITY);

            if (moved_to_bottom_flag_1 && moved_to_bottom_flag_2 &&
                lead_scew_switch_down_value_ == E_Lead_Screw_Switch_State::Triggered) {
                fsm.nextAction();
            }

            set_next_state_by_RC();
        }

        static void reset_LS_and_DM(bool &DM_complete_flag_, bool &DM_running_flag_, int16_t dm_velocity) {
            // 丝杆电机归零，可以直接copy ActionResetLS
            ActionResetLSandYaw::reset_LS();

            // 弹鼓电机归零，必须在丝杆电机归零后执行
            if (!DM_complete_flag_ && lead_scew_switch_down_value_ == E_Lead_Screw_Switch_State::Triggered) {
                motor_controller::pid_velocity_MotorDM.target_velocity_ = dm_velocity;

                if (move_motor_to_bottom_limit<>(motor_controller::pid_velocity_MotorDM,
                                                 dm_velocity,
                                                 DRUM_MAGAZINE_RIGHT_GATE_VELOCITY, DRUM_MAGAZINE_BLOCK_TIMEOUT,
                                                 DM_running_flag_, false)) {
                    DM_complete_flag_ = true;
                }
            }
        }

        void exit(OpenFSM &fsm) const override {
            // 重置标志位
            move_motor_to_bottom_running_flag_yaw = false;
            moved_to_bottom_flag_1 = false;
            move_motor_to_bottom_running_flag_dm = false;
            moved_to_bottom_flag_2 = false;

            // 恢复角度控制
            motor_controller::pid_angle_velocity_MotorY.set_state(
                    motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);
        }
    };

    enum E_Gate_State {
        OPENED,
        CLOSED,
        OPERATING
    };

    class ActionMatch_Wait : public OpenFSMAction {
    public:
        void enter(OpenFSM &fsm) const override {
            // Set Motor State: MotorLS -> IDLE, MotorY -> RUNNING, MotorDM -> IDLE, MotorFW -> RUNNING
            set_motor_next_state_with_disconnect_detection(motor::E_MotorState::IDLE,
                                                           motor::E_MotorState::RUNNING,
                                                           motor::E_MotorState::IDLE,
                                                           motor::E_MotorState::RUNNING);
            for (auto &i: motor::MotorFW)
                i.target_velocity_ = 0;

            // Angle Control for Motor Y
            motor_controller::pid_angle_velocity_MotorY.set_state(
                    motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);

            last_dart_gate_opening_status_ = E_Gate_State::CLOSED;
            last_dart_launch_time_ = 0;

            // 进入时蜂鸣一次
            buzzer_info_on_ = true;

            // 设置Action计数
            state_match_action_ = E_Match_Actions::Wait;
        }

        void update(OpenFSM &fsm) const override {
            // Load Yaw Default Angle
            motor_controller::pid_angle_velocity_MotorY.target_angle_with_rounds_ = target_yaw_angle_with_rounds;

            set_motor_next_state_with_disconnect_detection(motor::E_MotorState::IDLE,
                                                           motor::E_MotorState::RUNNING,
                                                           motor::E_MotorState::IDLE,
                                                           motor::E_MotorState::RUNNING);

#if CONFIG_FORCE_WAIT_FOR_GAME_PROGRESS == 1
            if (ext_game_status.game_progress != 4) {
                // 等待比赛开始，未开始则不允许发射
                return;
            }
#endif

            // 读取裁判系统变量，线程安全
            uint8_t dart_launch_opening_status = ext_dart_client_cmd.dart_launch_opening_status;
            uint8_t game_progress = ext_game_status.game_progress;
            uint8_t dart_remaining_time = ext_dart_info.dart_remaining_time;
            uint16_t latest_launch_cmd_time = ext_dart_client_cmd.latest_launch_cmd_time;

            // 等待发射信号
            bool launch_grant_ = false;

            // 信号一：裁判系统飞镖闸门从“正在开启”达到“完全开启”信号，同时比赛正常进行中
            launch_grant_ |= (last_dart_gate_opening_status_ == E_Gate_State::OPERATING &&
                              dart_launch_opening_status == E_Gate_State::OPENED &&
                              game_progress == 4);

            // 信号二：遥控器信号 外八字，不要求比赛进行中 要求不在上场模式
            launch_grant_ |= (RC_Data.ch0 > 1400 && RC_Data.ch2 < 400) && (!match_flag_);
            launch_grant_ |= (RC_Data.ch0 > 1400 && RC_Data.ch2 < 400) && game_progress == 4 && (match_flag_);

            // 信号三：飞镖发射剩余时间变化，时间落在15s内，而且比赛进行中
            launch_grant_ |= (dart_remaining_time > 0 &&
                              dart_remaining_time <= 15 &&
                              game_progress == 4);

            // 信号四：选手端手动发送触发
            // 比赛状态确认
            if (latest_launch_cmd_time != 0 &&
                latest_launch_cmd_time != last_dart_launch_time_ &&
                game_progress == 4) {
                last_dart_launch_time_ = latest_launch_cmd_time;
                launch_grant_ = true;
            }

            // 门控 比赛时间不足\准备阶段\自检时拒绝发射
            if ((ext_game_status.stage_remain_time < 10 && game_progress == 4) || game_progress == 1 ||
                game_progress == 2 || game_progress == 3 || game_progress == 5)
                launch_grant_ = false;

            // 摩擦轮预启动
            bool preload_fw_grant_ = false;

            // 信号一：裁判系统飞镖闸门从“完全关闭”达到“正在开启”信号，同时比赛正常进行中
            preload_fw_grant_ |= (last_dart_gate_opening_status_ == E_Gate_State::CLOSED &&
                                  dart_launch_opening_status == E_Gate_State::OPERATING &&
                                  game_progress == 4);

            // 信号二：遥控器信号 拨轮拨动，不要求比赛进行中
//            preload_fw_grant_ |= (RC_Data.ch4_wheel < 939 || RC_Data.ch4_wheel > 1121);

            // 信号三：飞镖发射剩余时间变化，时间落在15s内，而且比赛进行中
            preload_fw_grant_ |= (dart_remaining_time > 0 &&
                                  dart_remaining_time <= 15 &&
                                  game_progress == 4);

            // 信号四：选手端手动发送触发
            // 比赛状态确认
            if (latest_launch_cmd_time != 0 &&
                latest_launch_cmd_time != last_dart_launch_time_ &&
                game_progress == 4) {
                last_dart_launch_time_ = latest_launch_cmd_time;
                preload_fw_grant_ = true;
            }

            // 信号五：遥控器内八字开关
            if ((RC_Data.ch2 > 1400 && RC_Data.ch0 < 400 && (!match_flag_)) ||
                (RC_Data.ch2 > 1400 && RC_Data.ch0 < 400 && match_flag_ && game_progress == 4))
                preload_fw_grant_ = true;

//            // 信号六：比赛进程开始
//            if (game_progress == 4)
//                preload_fw_grant_ = true;

            // 门控 时间不足时禁止发射
            if ((ext_game_status.stage_remain_time < 10 && game_progress == 4) || game_progress == 1 ||
                game_progress == 2 || game_progress == 3 || game_progress == 5) {
                preload_fw_grant_ = false;
                // 摩擦轮归零
                for (int i = 0; i < 4; i++)
                    motor::MotorFW[i].target_velocity_ = 0;
            }

            // 摩擦轮预启动
            if (preload_fw_grant_) {
//                buzzer_info_on_ = true;
// 全速预启动
                for (int i = 0; i < 2; i++)
                    motor::MotorFW[i].target_velocity_ =
                            target_velocity_fw + launch_params[state_match_launch_counter_].target_velocity_fw_offset;
                for (int i = 2; i < 4; i++)
                    motor::MotorFW[i].target_velocity_ = int32_t(double(target_velocity_fw +
                                                                        launch_params[state_match_launch_counter_].target_velocity_fw_offset) *
                                                                 target_velocity_ratio);
// 怠速预启动
//                for (int i = 0; i < 2; i++)
//                    motor::MotorFW[i].target_velocity_ =
//                            1000;
//                for (int i = 2; i < 4; i++)
//                    motor::MotorFW[i].target_velocity_ = int32_t(double(1000) *
//                                                                 target_velocity_ratio);
            }

            if (launch_grant_)
                fsm.nextAction();

            last_dart_gate_opening_status_ = dart_launch_opening_status;
            // 允许退出比赛模式
//            if (RC_Data.ch0 > 1400 && RC_Data.ch2 < 400)
            set_next_state_by_RC();
        }
    };

    class ActionMatch_Launch : public OpenFSMAction { // 启动摩擦轮，并一次性把丝杆推完
        void enter(OpenFSM &fsm) const override {
            // Set Motor State: MotorLS -> IDLE, MotorY -> RUNNING, MotorDM -> IDLE, MotorFW -> RUNNING
            set_motor_next_state_with_disconnect_detection(motor::E_MotorState::IDLE,
                                                           motor::E_MotorState::RUNNING,
                                                           motor::E_MotorState::IDLE,
                                                           motor::E_MotorState::RUNNING);

            waiting_for_fw_stable_last_time_ = xTaskGetTickCount();
            waiting_for_fw_stable_flag_ = true;

            // 设置Action计数
            state_match_action_ = E_Match_Actions::Launch;
        }

        static bool push_for_launch(uint16_t launch_position_rounds) {
            if (motor_controller::pid_velocity_MotorLS.current_angle_with_rounds_ <
                launch_position_rounds * 8192) {
                motor::MotorLS.set_next_state(motor::E_MotorState::RUNNING);
                motor_controller::pid_velocity_MotorLS.target_velocity_ = LEAD_SCREW_UP_VELOCITY;
                return false;
            } else {
                return true;
            }
        }

        void update(OpenFSM &fsm) const override {
            // Buzzer 比赛特殊音效
            if ((xTaskGetTickCount() % 1000 < 200 && xTaskGetTickCount() % 1000 > 0) ||
                (xTaskGetTickCount() % 1000 > 300 && xTaskGetTickCount() % 1000 < 500)) {// 响
                __HAL_TIM_SET_AUTORELOAD(&htim4, 6000); // 频率最高
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 5000); // 音量最高
            } else
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); // 占空比停止


            // 摩擦轮启动
            for (int i = 0; i < 2; i++)
                motor::MotorFW[i].target_velocity_ =
                        target_velocity_fw +
                        launch_params[state_match_launch_counter_].target_velocity_fw_offset;
            for (int i = 2; i < 4; i++)
                motor::MotorFW[i].target_velocity_ = int32_t(
                        double(target_velocity_fw +
                               launch_params[state_match_launch_counter_].target_velocity_fw_offset) *
                        target_velocity_ratio);

            // Yaw轴位置调整
            motor_controller::pid_angle_velocity_MotorY.target_angle_with_rounds_ =
                    target_yaw_angle_with_rounds +
                    launch_params[state_match_launch_counter_].target_yaw_angle_with_rounds_offset;

            // 丝杆电机推到第一发射位置，等待摩擦轮速度恢复，然后推到第二发射位置
            if (waiting_for_fw_stable_flag_) {
                if (xTaskGetTickCount() - waiting_for_fw_stable_last_time_ >
                    pdMS_TO_TICKS(WAIT_TIME_BEFORE_LAUNCH)) {
                    waiting_for_fw_stable_flag_ = false;
                    waiting_for_fw_stable_last_time_ = xTaskGetTickCount();
                    // 二次检查waiting_for_fw_stable_flag_，用作安全锁定，最后十秒钟拒绝飞镖发射
                    if ((ext_game_status.stage_remain_time < 10 && ext_game_status.game_progress == 4) ||
                        ext_game_status.game_progress == 1 ||
                        ext_game_status.game_progress == 2 ||
                        ext_game_status.game_progress == 3 ||
                        ext_game_status.game_progress == 5) {
                        waiting_for_fw_stable_flag_ = true;
                        fsm.nextAction(); // 直接进入ActionMatch_Reload
                    }
                }
            }


            if (!waiting_for_fw_stable_flag_ &&
                (state_match_launch_counter_ == 0 || state_match_launch_counter_ == 2)) {
                // 非阻塞推到第一发射位置
                if (push_for_launch(FIRST_LAUNCH_POSITION_ROUNDS)) {
                    // 推出完成
                    motor_controller::pid_velocity_MotorLS.target_velocity_ = 0;
                    state_match_launch_counter_++;
                    waiting_for_fw_stable_flag_ = true;
                    waiting_for_fw_stable_last_time_ = xTaskGetTickCount();
                }
            }
            if (!waiting_for_fw_stable_flag_ &&
                (state_match_launch_counter_ == 1 || state_match_launch_counter_ == 3)) {
                // 非阻塞推到第二发射位置
                if (push_for_launch(SECOND_LAUNCH_POSITION_ROUNDS)) {
                    // 推出完成
                    motor_controller::pid_velocity_MotorLS.target_velocity_ = 0;
                    state_match_launch_counter_++;
                    motor::MotorLS.set_next_state(motor::E_MotorState::IDLE);
                    fsm.nextAction();
                }
            }
            // 可以退出比赛模式
//            if (RC_Data.ch0 > 1400 && RC_Data.ch2 < 400)
            set_next_state_by_RC();
        }
    };

    class ActionMatch_Reload : public OpenFSMAction { // 复位丝杆和弹鼓，但是不动yaw轴
        void enter(OpenFSM &fsm) const override {
            // Set Motor State: MotorLS -> RUNNING, MotorY -> RUNNING, MotorDM -> RUNNING, MotorFW -> IDLE
            set_motor_next_state_with_disconnect_detection(motor::E_MotorState::RUNNING,
                                                           motor::E_MotorState::RUNNING,
                                                           motor::E_MotorState::RUNNING,
                                                           motor::E_MotorState::IDLE);

            move_motor_to_bottom_running_flag_dm = false; // 重置标志位
            moved_to_bottom_flag_2 = false; // 重置标志位

            // 设置Action计数
            state_match_action_ = E_Match_Actions::Reload;
        }

        void update(OpenFSM &fsm) const override {


            ActionMatch_Enter::reset_LS_and_DM(moved_to_bottom_flag_2, move_motor_to_bottom_running_flag_dm,
                                               state_match_launch_counter_ == 2 ? DRUM_MAGAZINE_LEFT_VELOCITY
                                                                                : DRUM_MAGAZINE_RIGHT_VELOCITY);

            if (moved_to_bottom_flag_2 &&
                lead_scew_switch_down_value_ == E_Lead_Screw_Switch_State::Triggered) {
                fsm.nextAction();
            }
            // 可以退出比赛模式
//            if (RC_Data.ch0 > 1400 && RC_Data.ch2 < 400)
            set_next_state_by_RC();
        }
    };

    void fsm_thread(void *parameters) {
        TickType_t last_time;
        OpenFSM::RegisterAction<ActionWaitForAllMotorOnline>("ActionWaitForAllMotorOnline");
        OpenFSM::RegisterAction<ActionResetLSandYaw>("ActionResetLSandYaw");
        OpenFSM::RegisterAction<ActionResetDM>("ActionResetDM");
        OpenFSM::RegisterAction<ActionProtect>("ActionProtect");
        OpenFSM::RegisterAction<ActionRemote>("ActionRemote");
        OpenFSM::RegisterAction<ActionMatch_Enter>("ActionMatch_Enter");
        OpenFSM::RegisterAction<ActionMatch_Wait>("ActionMatch_Wait");
        OpenFSM::RegisterAction<ActionMatch_Launch>("ActionMatch_Launch");
        OpenFSM::RegisterAction<ActionMatch_Reload>("ActionMatch_Reload");

        OpenFSM::RegisterState("StateBoot", {"ActionWaitForAllMotorOnline", "ActionResetLSandYaw", "ActionResetDM"},
                               E_Dart_State::Boot);
        OpenFSM::RegisterState("StateProtect", {"ActionProtect"}, E_Dart_State::Protect);
        OpenFSM::RegisterState("StateRemote", {"ActionRemote"}, E_Dart_State::Remote);
        OpenFSM::RegisterState("StateMatch", {"ActionMatch_Enter", "ActionMatch_Wait", "ActionMatch_Launch",
                                              "ActionMatch_Reload", "ActionMatch_Wait", "ActionMatch_Launch",
                                              "ActionMatch_Reload"},
                               E_Dart_State::Match);

        OpenFSM::RegisterRelation("StateBoot", {"StateProtect"});
        OpenFSM::RegisterRelation("StateProtect", {"StateRemote", "StateMatch"});
        OpenFSM::RegisterRelation("StateRemote", {"StateProtect", "StateMatch"});
        OpenFSM::RegisterRelation("StateMatch", {"StateProtect", "StateRemote"});

        dart_fsm.start();

        while (true) {
            dart_fsm.update();
            vTaskDelayUntil(&last_time, pdMS_TO_TICKS(1)); // 1000Hz
        }
        vTaskDelete(nullptr);
    }

} // namespace state_machine