//
// Created by cheny on 24-9-18.
//

// TODO: 准备阶段进行自瞄，随后将自瞄结果从offset存到primary，在launch时测算移动靶偏移量
// TODO: 比赛模式第三、四发之间添加等待时间，防止抖落

#include "state_machine.h"

#include <cmath>

#include "openfsm.h"
#include "motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sound_effect.h"
#include "buzzer_examples.h"
#include "motor_controller.h"
#include "micro_switch.h"
#include "dart_config.h"
#include "servo.h"
#include "dbus.h"
#include "judge_receive.h"
#include "velocimeter.h"

namespace state_machine
{
#define anyMotorDisconnected \
    (motor::MotorYawLS.motor_state_ == motor::E_MotorState::DISCONNECTED || \
     motor::MotorLoad[0].motor_state_ == motor::E_MotorState::DISCONNECTED || \
     motor::MotorLoad[1].motor_state_ == motor::E_MotorState::DISCONNECTED || \
     motor::MotorTriggerLS.motor_state_ == motor::E_MotorState::DISCONNECTED)

#define enterProtectModeIfMotorDisconnected() \
do { \
if (dart_fsm.openFSM_.focusEState() != E_Dart_State::Protect && anyMotorDisconnected){ \
dart_fsm.openFSM_.nextState(E_Dart_State::Protect);                                   \
dart_mcu_log("Enter Protect: Motor Disconnected");                                              \
} \
} while (0)

#define restartCANIfMotorDisconnected() \
do {                                    \
static TickType_t last_reboot_can_time = xTaskGetTickCount();                                        \
if ((xTaskGetTickCount() - last_reboot_can_time > 1000) && (       \
motor::MotorYawLS.motor_state_ == motor::E_MotorState::DISCONNECTED || \
motor::MotorLoad[0].motor_state_ == motor::E_MotorState::DISCONNECTED || \
motor::MotorLoad[1].motor_state_ == motor::E_MotorState::DISCONNECTED || \
motor::MotorTriggerLS.motor_state_ == motor::E_MotorState::DISCONNECTED)){ \
reboot_can(&hcan1); \
reboot_can(&hcan2);                     \
last_reboot_can_time = xTaskGetTickCount();                   \
}                                       \
} while (0)

#define enableTriggerServo() \
do{ \
trigger_servo[0].enable(); \
trigger_servo[1].enable(); \
}while(0)

#define setTriggerServotoTrigger() \
do{ \
trigger_servo[0].setAngle(CONFIG_TRIGGER_SERVO_TRIGGER_ANGLE_0); \
trigger_servo[1].setAngle(CONFIG_TRIGGER_SERVO_TRIGGER_ANGLE_1); \
}while(0)

#define setTriggerServotoReload() \
do{ \
trigger_servo[0].setAngle(CONFIG_TRIGGER_SERVO_RELOAD_ANGLE_0); \
trigger_servo[1].setAngle(CONFIG_TRIGGER_SERVO_RELOAD_ANGLE_1); \
}while(0)

#define disableTriggerServo() \
do{ \
trigger_servo[0].disable(); \
trigger_servo[1].disable(); \
}while(0)

#define enableLoadServo() \
do{                       \
trigger_servo[2].enable(); \
trigger_servo[3].enable();\
trigger_servo[4].enable(); \
trigger_servo[5].enable();\
}while(0)

#define disableLoadServo() \
do{                       \
trigger_servo[2].disable(); \
trigger_servo[3].disable();\
trigger_servo[4].disable(); \
trigger_servo[5].disable();\
}while(0)

#define setLoadServotoUP() \
do{                        \
    trigger_servo[2].setAngle(CONFIG_LOAD_SERVO_UP_ANGLE_0); \
    trigger_servo[3].setAngle(CONFIG_LOAD_SERVO_UP_ANGLE_0); \
    trigger_servo[4].setAngle(CONFIG_LOAD_SERVO_UP_ANGLE_1); \
    trigger_servo[5].setAngle(CONFIG_LOAD_SERVO_UP_ANGLE_1); \
}while(0)

#define setLoadServotoDOWN() \
do{                        \
    trigger_servo[2].setAngle(CONFIG_LOAD_SERVO_DOWN_ANGLE_0); \
    trigger_servo[3].setAngle(CONFIG_LOAD_SERVO_DOWN_ANGLE_0); \
    trigger_servo[4].setAngle(CONFIG_LOAD_SERVO_DOWN_ANGLE_1); \
    trigger_servo[5].setAngle(CONFIG_LOAD_SERVO_DOWN_ANGLE_1); \
}while(0)

#define enableSlidedownServo() \
do{                        \
    trigger_servo[6].enable(); \
}while(0)

#define disableSlidedownServo() \
do{                        \
    trigger_servo[6].disable(); \
}while(0)

#define setSlidedownServotoSlide() \
do{                        \
    trigger_servo[6].setAngle(CONFIG_SLIDE_SERVO_SLIDE_ANGLE); \
}while(0)

#define setSlidedownServotoCut() \
do{                        \
    trigger_servo[6].setAngle(CONFIG_SLIDE_SERVO_CUT_ANGLE); \
}while(0)

#define disableLaser() HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET)
#define enableLaser() HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET)

    // 裁判系统判定Flag
    uint8_t last_dart_gate_opening_status_ = 0; // 上一次发射状态
    uint16_t last_dart_launch_time_ = 0; // 上一次发射指令下达时间
    bool match_flag_ = 0; // 上场比赛判断 若已经上场则执行最严格的安全措施

    bool isRemoteOnline(TickType_t current_tick)
    {
        return (current_tick < RC_Data.last_update_time) || (current_tick - RC_Data.last_update_time) < 1000;
    }

    void setNextStateByRemote(bool enterProtectIfDisconnected = true, bool inMatch = false)
    {
        // 通过遥控器设置状态机状态
        E_Dart_State next_state = E_Dart_State::Protect;

        TickType_t current_tick = xTaskGetTickCount();

        if (inMatch)
        {
            if (isRemoteOnline(current_tick))
            {
                if (RC_Data.Switch_Right == RC_SW_UP)
                {
                    next_state = E_Dart_State::Protect;
                }
                else if (RC_Data.Switch_Right == RC_SW_DOWN || RC_Data.Switch_Right == RC_SW_MID)
                {
                    next_state = E_Dart_State::Match;
                }
            }
        }
        else
        {
            if (isRemoteOnline(current_tick))
            {
                if (RC_Data.Switch_Right == RC_SW_UP)
                {
                    next_state = E_Dart_State::Protect;
                }
                else if (RC_Data.Switch_Right == RC_SW_DOWN)
                {
                    next_state = E_Dart_State::Match;
                }
                else if (RC_Data.Switch_Right == RC_SW_MID)
                {
                    next_state = E_Dart_State::Remote;
                }
            }
            else if (dart_fsm.openFSM_.focusEState() != Protect &&
                enterProtectIfDisconnected)
            {
                next_state = E_Dart_State::Protect;
                dart_mcu_log("Enter Protect: Remote Disconnected");
            }
            enterProtectModeIfMotorDisconnected();
        }

        if (next_state != dart_fsm.openFSM_.focusEState() && !anyMotorDisconnected)
        {
            if (!isRemoteOnline(current_tick))
                dart_mcu_log("Enter Protect: Remote Disconnected");
            dart_fsm.openFSM_.nextState(next_state);
        }
    }

    void FSM::update()
    {
        // 状态机更新
        openFSM_.update();
        micro_switch_read();

        // 遥控看门狗
        static TickType_t last_reset_tick = xTaskGetTickCount();
        if (xTaskGetTickCount() - RC_Data.last_update_time > pdMS_TO_TICKS(1000) &&
            xTaskGetTickCount() - last_reset_tick > pdMS_TO_TICKS(200))
        {
            DT7_Reset();
            last_reset_tick = xTaskGetTickCount();
        }

        static TickType_t last_reset_tick_judge_judge = xTaskGetTickCount();
        if (xTaskGetTickCount() - ext_judge_last_receive_time > pdMS_TO_TICKS(1000) &&
            xTaskGetTickCount() - last_reset_tick_judge_judge > pdMS_TO_TICKS(200))
        {
            judge_Reset();
            last_reset_tick_judge_judge = xTaskGetTickCount();
        }

        // CAN看门狗
        restartCANIfMotorDisconnected();

        // 系统状态更新 写入dart_launcher_status
        msgDartStatus.motor_yaw_online = motor::MotorYawLS.motor_state_ != motor::E_MotorState::DISCONNECTED;
        msgDartStatus.motor_loader_online[0] =
            motor::MotorLoad[0].motor_state_ != motor::E_MotorState::DISCONNECTED;
        msgDartStatus.motor_loader_online[1] =
            motor::MotorLoad[1].motor_state_ != motor::E_MotorState::DISCONNECTED;
        msgDartStatus.motor_trigger_online =
            motor::MotorTriggerLS.motor_state_ != motor::E_MotorState::DISCONNECTED;
        msgDartStatus.judge_online = !(xTaskGetTickCount() - ext_judge_last_receive_time > pdMS_TO_TICKS(1000));
        msgDartStatus.rc_online = isRemoteOnline(xTaskGetTickCount());
        msgDartStatus.dart_state = openFSM_.focusEState();
        msgDartStatus.motor_yaw_angle = motor_controller::MotorYawLSController.current_angle_with_rounds_;
        msgDartStatus.motor_trigger_angle =
            motor::MotorTriggerLS.current_round_ * 8192 + motor::MotorTriggerLS.current_angle_;
        msgDartStatus.motor_loader_angle[0] =
            motor::MotorLoad[0].current_round_ * 8192 + motor::MotorLoad[0].current_angle_;
        msgDartStatus.motor_loader_angle[1] =
            motor::MotorLoad[1].current_round_ * 8192 + motor::MotorLoad[1].current_angle_;
        msgDartStatus.motor_loader_current[0] =
            motor::MotorLoad[0].target_current_;
        msgDartStatus.motor_loader_current[1] =
            motor::MotorLoad[1].target_current_;
        msgDartStatus.params = msgDartParams;
        msgDartStatus.protocols = msgDartProtocols;
        msgDartStatus.game_progress = ext_game_status.game_progress;
        msgDartStatus.stage_remain_time = ext_game_status.stage_remain_time;
        msgDartStatus.dart_remaining_time = ext_dart_info.dart_remaining_time;
        msgDartStatus.dart_launch_opening_status = ext_dart_client_cmd.dart_launch_opening_status;;
        msgDartStatus.latest_launch_cmd_time = ext_dart_client_cmd.latest_launch_cmd_time;

        // 比赛上场判断
        if (ext_game_status.game_progress != 0)
            match_flag_ = 1;
    }

    Dart_FSM dart_fsm;

    class ActionWaitForAllMotorOnline : public OpenFSMAction
    {
    public:
        void enter(OpenFSM& fsm) const override
        {
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_dont_say_lazy));
            enableLoadServo();
            setLoadServotoUP();
            setTriggerServotoReload();
            enableTriggerServo();
            enableSlidedownServo();

            msgDartStatus.dart_state = dart_fsm.openFSM_.focusEState();
        }

        void update(OpenFSM& fsm) const override
        {
            // 电机上电后默认状态为IDLE
            if (motor::MotorYawLS.motor_state_ != motor::E_MotorState::DISCONNECTED &&
                motor::MotorLoad[0].motor_state_ != motor::E_MotorState::DISCONNECTED &&
                motor::MotorLoad[1].motor_state_ != motor::E_MotorState::DISCONNECTED &&
                motor::MotorTriggerLS.motor_state_ != motor::E_MotorState::DISCONNECTED)
            {
                dart_mcu_log("Resetting zero point...");
                fsm.nextAction();
            }
        }

        void exit(OpenFSM& fsm) const override
        {
            soundEffectManager.clearSoundEffects();
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_approach));
        }
    };

    /**
     * @brief 非阻塞电机移动直到限位开关被触发，返回值表示是否执行完成
     * @tparam T
     * @param fsm 状态机对象
     * @param pid PID控制器对象
     * @param triggerFlag 限位开关状态
     * @param target_reset_velocity 目标速度
     * @return
     */
    template <typename T>
    E_ResetActionReturnState
    actionResetLSUntilTrigger(motor_controller::pid_angle_velocity_controller<T>& pid,
                              E_Lead_Screw_Switch_State& triggerFlag,
                              int16_t target_reset_velocity)
    {
        if (pid.motor_->motor_state_ == motor::E_MotorState::DISCONNECTED)
        {
            return E_ResetActionReturnState::Failed;
        }
        else if (pid.motor_->motor_state_ == motor::E_MotorState::RUNNING || pid.motor_->motor_state_ ==
            motor::E_MotorState::IDLE)
        {
            if (triggerFlag == E_Lead_Screw_Switch_State::Untriggered)
            {
                pid.target_velocity_ = target_reset_velocity;
                pid.set_state(motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
                return E_ResetActionReturnState::Operating;
            }
            else if (triggerFlag == E_Lead_Screw_Switch_State::Triggered)
            {
                pid.motor_->setNextState(motor::E_MotorState::IDLE);
                pid.motor_->resetRound();
                return E_ResetActionReturnState::Finished;
            }
        }
        else if (triggerFlag == E_Lead_Screw_Switch_State::Untriggered)
        {
            pid.target_velocity_ = target_reset_velocity;
            pid.set_state(motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
            return E_ResetActionReturnState::Operating;
        }
        return E_ResetActionReturnState::Operating;
    }

    /**
     * @brief 非阻塞电机移动到底限位，返回值表示是否执行完成
     * @tparam TypeTarget
     * @tparam TypeGate
     * @tparam TypeController
     * @param controller_ PID控制器对象
     * @param operation_target_ 目标速度
     * @param gate_velocity_ 限位速度
     * @param gate_current_ 限位电流
     * @param timeout_ 超时时间
     * @param running_flag_ 运行标志
     * @param openloop_ 是否开环
     * @return 是否执行完成
     */
    E_ResetActionReturnState
    actionResetMotorUntilBlocked(motor_controller::pid_angle_velocity_controller<double>& controller_,
                                 int operation_target_, int gate_velocity_, int gate_current_,
                                 TickType_t timeout_, TickType_t& last_time, uint8_t& running_flag_,
                                 bool openloop_ = false)
    {
        if (running_flag_ == 0)
        {
            // 开始运行
            last_time = xTaskGetTickCount();
            if (openloop_)
            {
                controller_.
                    set_state(motor_controller::E_PID_Velocity_Angle_Controller_State::OPEN_LOOP);
                controller_.
                    target_openloop_ = operation_target_;
            }
            else
            {
                controller_.
                    set_state(motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
                controller_.
                    target_velocity_ = operation_target_;
            }
            running_flag_ = 1;
            return E_ResetActionReturnState::Operating; // 未完成
        }
        else if (running_flag_ == 1)
        {
            // 运行中
            if ((openloop_ == false &&
                    abs(controller_.motor_->target_current_) >= abs(gate_current_)) ||
                (openloop_ == true &&
                    abs(controller_.motor_->current_velocity_) <= abs(gate_velocity_))
            )
            {
                if (xTaskGetTickCount() - last_time > timeout_)
                {
                    // 停止电机，复原状态
                    if (!openloop_)
                        controller_.target_velocity_ = 0;
                    // controller_.motor_->setNextState(motor::E_MotorState::IDLE);
                    controller_.motor_->resetRound();
                    running_flag_ = 2;
                    return E_ResetActionReturnState::Finished; // 完成
                }
            }
            else
            {
                controller_.
                    target_velocity_ = operation_target_;
                last_time = xTaskGetTickCount();
            }
            return E_ResetActionReturnState::Operating; // 未完成
        }
        else if (running_flag_ == 2)
            return E_ResetActionReturnState::Finished; // 完成

        else
        {
            running_flag_ = 0;
            return E_ResetActionReturnState::Failed; // 失败
        }
    }

    /**
     * @brief 上电归零操作，将YAW轴丝杆移动到限位开关位置，将双扳机舵机移动到初始位置，将装填电机移动到初始位置
     */
    class ActionResetMotors : public OpenFSMAction
    {
        // Dart_FSM Flags使用
        // 0 for MotorYawLS Reset Success
        // 2 for MotorTriggerLS Reset Success
    public:
        void enter(OpenFSM& fsm) const override
        {
            motor::MotorLoad[0].setNextState(motor::E_MotorState::RUNNING);
            motor::MotorLoad[1].setNextState(motor::E_MotorState::RUNNING);
            motor::MotorYawLS.setNextState(motor::E_MotorState::RUNNING);
            motor::MotorTriggerLS.setNextState(motor::E_MotorState::RUNNING);
            fsm.custom<Dart_FSM>()->ActionResetMotors_Load_0_Reset_State = false;
            fsm.custom<Dart_FSM>()->ActionResetMotors_Load_1_Reset_State = false;
            fsm.custom<Dart_FSM>()->ActionResetMotors_both_initialized = false;
            fsm.custom<Dart_FSM>()->ActionResetMotors_TriggerLS_Reset_State = false;
            fsm.custom<Dart_FSM>()->ActionGeneral_Timer0_ = 0;
            fsm.custom<Dart_FSM>()->ActionGeneral_Timer1_ = 0;
            fsm.custom<Dart_FSM>()->ActionGeneral_Timer2_ = 0;

            setTriggerServotoReload();
            enableTriggerServo();

            msgDartStatus.dart_state = dart_fsm.openFSM_.focusEState();
        }

        void update(OpenFSM& fsm) const override
        {
            //            使用与逻辑保证所有函数执行完成
            bool success = true;
            success &= actionResetLSUntilTrigger<>(motor_controller::MotorYawLSController,
                                                   yaw_switch_state,
                                                   CONFIG_TARGET_RESET_VELOCITY_YAWLS) ==
                E_ResetActionReturnState::Finished;
            success &= actionResetMotorUntilBlocked(motor_controller::MotorTriggerLSController,
                                                    CONFIG_TARGET_RESET_VELOCITY_TRIGGERLS,
                                                    CONFIG_GATE_VELOCITY_TRIGGERLS,
                                                    CONFIG_GATE_CURRENT_TRIGGERLS,
                                                    pdMS_TO_TICKS(CONFIG_TIMEOUT_RESET_TRIGGER),
                                                    fsm.custom<Dart_FSM>()->ActionGeneral_Timer0_,
                                                    fsm.custom<Dart_FSM>()->ActionResetMotors_TriggerLS_Reset_State,
                                                    false) ==
                E_ResetActionReturnState::Finished;
            success &= actionResetMotorUntilBlocked(motor_controller::MotorLoadController[0],
                                                    CONFIG_TARGET_RESET_VELOCITY_LOAD,
                                                    CONFIG_GATE_VELOCITY_LOAD,
                                                    CONFIG_GATE_CURRENT_LOAD,
                                                    pdMS_TO_TICKS(CONFIG_TIMEOUT_RESET_LOAD),
                                                    fsm.custom<Dart_FSM>()->ActionGeneral_Timer1_,
                                                    fsm.custom<Dart_FSM>()->ActionResetMotors_Load_0_Reset_State,
                                                    true) ==
                E_ResetActionReturnState::Finished;
            success &= actionResetMotorUntilBlocked(motor_controller::MotorLoadController[1],
                                                    CONFIG_TARGET_RESET_VELOCITY_LOAD,
                                                    CONFIG_GATE_VELOCITY_LOAD,
                                                    CONFIG_GATE_CURRENT_LOAD,
                                                    pdMS_TO_TICKS(CONFIG_TIMEOUT_RESET_LOAD),
                                                    fsm.custom<Dart_FSM>()->ActionGeneral_Timer2_,
                                                    fsm.custom<Dart_FSM>()->ActionResetMotors_Load_1_Reset_State,
                                                    true) ==
                E_ResetActionReturnState::Finished;

            if (fsm.custom<Dart_FSM>()->ActionResetMotors_Load_0_Reset_State == 2
                && fsm.custom<Dart_FSM>()->ActionResetMotors_Load_1_Reset_State == 2
                && fsm.custom<Dart_FSM>()->ActionResetMotors_both_initialized == false)
            {
                fsm.custom<Dart_FSM>()->ActionResetMotors_both_initialized = true;
                motor::MotorLoad[0].resetRound();
                motor::MotorLoad[1].resetRound();
                motor_controller::motor_load_sync_offset =
                    motor::MotorLoad[0].current_angle_ - motor::MotorLoad[1].current_angle_;
                motor::MotorLoad[0].setNextState(motor::E_MotorState::IDLE);
                motor::MotorLoad[1].setNextState(motor::E_MotorState::IDLE);
                motor_controller::MotorLoadController[0].target_openloop_ = 0;
                motor_controller::MotorLoadController[1].target_openloop_ = 0;
            }

            if (success)
            {
                soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_chunriying));
                fsm.nextAction();
            }
        }
    };

    class ActionReleaseMotors : public OpenFSMAction
    {
    public:
        void enter(OpenFSM& fsm) const override
        {
            msgDartStatus.dart_state = dart_fsm.openFSM_.
                                                focusEState();
        }

        void update(OpenFSM& fsm) const override
        {
            // 将TriggerLS电机移动到初始位置
            motor::MotorLoad[0].setNextState(motor::E_MotorState::IDLE);
            motor::MotorLoad[1].setNextState(motor::E_MotorState::IDLE);
            motor::MotorYawLS.setNextState(motor::E_MotorState::RUNNING);
            motor::MotorTriggerLS.setNextState(motor::E_MotorState::RUNNING);
            motor_controller::MotorTriggerLSController.set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);
            motor_controller::MotorYawLSController.set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);
            motor_controller::MotorLoadController[0].set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
            motor_controller::MotorLoadController[1].set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
            motor_controller::MotorTriggerLSController.target_angle_with_rounds_ = 6000000;
            motor_controller::MotorYawLSController.target_angle_with_rounds_ = 40000;

            // MotorYawLS、MotorPitchLS、MotorTriggerLS到达目标位置
            if (abs(motor_controller::MotorTriggerLSController.current_angle_with_rounds_ - 6000000) < 10000 &&
                abs(motor_controller::MotorYawLSController.current_angle_with_rounds_ - 40000) < 1000)
            {
                fsm.nextAction();
            }
        }
    };

    class ActionProtect : public OpenFSMAction
    {
    public:
        mutable int song_index = 0; // 当前歌曲索引
        mutable int last_sw_left = 0; // 上一首歌曲索引

        void enter(OpenFSM& fsm) const override
        {
            // 保护状态
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_autopilot_disconnect));
            // 关闭激光器
            disableLaser();
            meter::velocity_meter.disable();
            msgDartStatus.dart_state = dart_fsm.openFSM_.focusEState();
            last_sw_left = RC_Data.Switch_Left; // 保存上一次拨轮位置
        }

        void update(OpenFSM& fsm) const override
        {
            // 保护状态，但是可以手动触发重新零点标定
            motor::MotorLoad[0].setNextState(motor::E_MotorState::RUNNING);
            motor::MotorLoad[1].setNextState(motor::E_MotorState::RUNNING);

            motor_controller::MotorLoadController[0].target_velocity_ = 0;
            motor_controller::MotorLoadController[1].target_velocity_ = 0;

            motor::MotorYawLS.setNextState(motor::E_MotorState::IDLE);
            motor::MotorTriggerLS.setNextState(motor::E_MotorState::IDLE);

            // 内八触发重新标定
            bool reset_grant_ = false;
            if ((RC_Data.ch2 > 1400 && RC_Data.ch0 < 400))
                reset_grant_ = true;

            if (reset_grant_)
            {
                fsm.enterState(E_Dart_State::Boot);
                reboot_can(&hcan1);
                reboot_can(&hcan2);
                return;
            }

            // 使用拨轮选择歌曲
            if (RC_Data.Switch_Left != last_sw_left)
            {
                if (RC_Data.Switch_Left == RC_SW_UP)
                {
                    song_index = (song_index + 1) % BuzzerSound::BuzzerSoundMax; // 顺时针拨动，下一首
                    // 播放选定的歌曲
                    soundEffectManager.clearSoundEffects();
                    choose_sound_effect(song_index);
                }
                else if (RC_Data.Switch_Left == RC_SW_DOWN)
                {
                    song_index =
                        (song_index - 1 + BuzzerSound::BuzzerSoundMax) %
                        BuzzerSound::BuzzerSoundMax; // 逆时针拨动，上一首
                    // 播放选定的歌曲
                    soundEffectManager.clearSoundEffects();
                    choose_sound_effect(song_index);
                }
                last_sw_left = RC_Data.Switch_Left;
            }

            setNextStateByRemote();
        }

        void exit(OpenFSM& fsm) const override
        {
            enableLaser();
            enableTriggerServo();
            enableSlidedownServo();
            soundEffectManager.clearSoundEffects();
        }
    };

    bool updateAutoAim(dart_msgs__msg__DartLauncherParams& msgDartParams_)
    {
        static uint8_t no_autoaim_count = 0;
        static TickType_t last_autoaim_update_tick = 0;
        if (xTaskGetTickCount() - last_autoaim_update_tick > 100)
        {
            // 10 fps
            last_autoaim_update_tick = xTaskGetTickCount();
            if (msgDartParams_.auto_aim_enabled)
            {
                if (msgGreenLight.is_detected
                    && xTaskGetTickCount() - last_greenlight_update_time < 400)
                {
                    // TODO: 引入非线性PID Error, 加快自瞄收敛速度
                    int error = msgDartParams_.target_auto_aim_x_axis - msgGreenLight.location.x;
                    // if (error >= 220)
                    // {
                    //     msgDartStatus.primary_yaw_offset = motor_controller::AutoAimController.update(
                    //         sqrt(abs(error)));
                    // }
                    // else if (error <= -220)
                    // {
                    //     msgDartStatus.primary_yaw_offset = motor_controller::AutoAimController.update(
                    //         -sqrt(abs(error)));
                    // }
                    if (abs(error) > 20)
                    {
                        msgDartStatus.primary_yaw_offset = motor_controller::AutoAimController.update(error);
                    }else if (abs(error) > 5)
                    {
                        msgDartStatus.primary_yaw_offset = motor_controller::AutoAimController.update(0.5 * error * sqrt(abs(error)));
                    }

                    no_autoaim_count = 0;
                    static char buf[30];
                    snprintf(buf, sizeof(buf), "Autoaim updated to %d", msgDartStatus.primary_yaw_offset);
                    dart_mcu_log(buf);
                    motor_controller::MotorYawLSController.target_angle_with_rounds_ =
                        msgDartParams_.primary_yaw + msgDartStatus.primary_yaw_offset;

                    return true;
                }
                else
                {
                    no_autoaim_count++;
                    if (no_autoaim_count > 5)
                    {
                        motor_controller::MotorYawLSController.target_angle_with_rounds_ =
                            msgDartParams_.primary_yaw;
                        msgDartStatus.primary_yaw_offset = 0;
                        motor_controller::AutoAimController.reset();
                    }
                    dart_mcu_log("Skipped autoaim update.");
                    return false;
                }
            }
        }
        return false;
    }

    class ActionRemote : public OpenFSMAction
    {
    public:
        void enter(OpenFSM& fsm) const override
        {
            enableTriggerServo();
            enableSlidedownServo();
            setTriggerServotoReload();


            setLoadServotoUP();
            setSlidedownServotoCut();

            enableLaser();
            motor_controller::MotorLoadController[0].set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);

            motor_controller::MotorLoadController[1].set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);

            motor_controller::MotorYawLSController.set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);

            motor_controller::MotorTriggerLSController.set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);

            // 重置状态变量
            fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 0;
            fsm.custom<Dart_FSM>()->launch_operating_ = false;
            fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reload_State = 0;
            fsm.custom<Dart_FSM>()->ActionRemoteandReload_Slidedown_State = 0;
            fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reset_State = 0;
            fsm.custom<Dart_FSM>()->reset_operating_ = false;
            fsm.custom<Dart_FSM>()->ActionRemote_launch_complete_ = false;

            motor_controller::MotorLoadSyncController.reset();

            msgDartStatus.dart_state = dart_fsm.openFSM_.focusEState();

            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_dji_startup));
        }

        void update(OpenFSM& fsm) const override
        {
            // 遥控状态
            motor::MotorLoad[0].setNextState(motor::E_MotorState::RUNNING);
            motor::MotorLoad[1].setNextState(motor::E_MotorState::RUNNING);
            motor::MotorYawLS.setNextState(motor::E_MotorState::RUNNING);
            motor::MotorTriggerLS.setNextState(motor::E_MotorState::RUNNING);

            // 响应遥控器指令
            if (RC_Data.Switch_Left == RC_SW_UP || RC_Data.Switch_Left == RC_SW_MID)
            {
                fsm.custom<Dart_FSM>()->launch_operating_ = false;
                // 扳机锁定在初始位置，不可触发操作，可以操作Yaw、Load电机和扳机丝杆
                {
                    // Yaw轴控制
                    // <--- 700 --- 900 --- 中点 --- 1100 --- 1310 --->
                    // <+
                    // 100    +10                      -10      -100>
                    // 如果有速度，则转为速度控制模式，否则转为位置控制模式

                    if (RC_Data.ch0 > 900 && RC_Data.ch0 < 1100)
                    {
                        if (motor_controller::MotorYawLSController.state_ !=
                            motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL)
                        {
                            motor_controller::MotorYawLSController.set_state(
                                motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);
                            motor_controller::MotorYawLSController.
                                target_angle_with_rounds_ = motor::MotorYawLS.current_round_ * 8192 +
                                motor::MotorYawLS.current_angle_;
                            msgDartParams.primary_yaw = motor_controller::MotorYawLSController.
                                target_angle_with_rounds_;

                            msgDartParams.last_param_update_time = rmw_uros_epoch_millis();
                        }
                        // 从msgDartParams里获取
                        // 判断是否更新自瞄
                        if (!updateAutoAim(msgDartParams) && !msgDartParams.auto_aim_enabled)
                            motor_controller::MotorYawLSController.
                                target_angle_with_rounds_ = msgDartParams.primary_yaw;
                    }
                    else
                    {
                        if (motor_controller::MotorYawLSController.state_ !=
                            motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL)
                            motor_controller::MotorYawLSController.set_state(
                                motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
                        if (RC_Data.ch0 <= 700 && !yaw_switch_state == Triggered)
                            motor_controller::MotorYawLSController.target_velocity_ = -100;
                        else if (RC_Data.ch0 > 700 && RC_Data.ch0 <= 900 && !yaw_switch_state == Triggered)
                            motor_controller::MotorYawLSController.target_velocity_ = -20;
                        else if (RC_Data.ch0 >= 1100 && RC_Data.ch0 < 1310)
                            motor_controller::MotorYawLSController.target_velocity_ = 20;
                        else if (RC_Data.ch0 >= 1310)
                            motor_controller::MotorYawLSController.target_velocity_ = 100;
                        motor_controller::AutoAimController.reset();
                    }
                }

                // Trigger扳机丝杆控制
                {
                    // <--- 600 --- 800 --- 中点 --- 1200 --- 1410 --->
                    // 如果有速度，则转为速度控制模式，否则转为位置控制模式
                    if (RC_Data.ch2 > 900 && RC_Data.ch2 < 1100)
                    {
                        if (motor_controller::MotorTriggerLSController.state_ !=
                            motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL)
                        {
                            motor_controller::MotorTriggerLSController.set_state(
                                motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);
                            motor_controller::MotorTriggerLSController.target_angle_with_rounds_ =
                                motor::MotorTriggerLS.current_round_ * 8192 +
                                motor::MotorTriggerLS.current_angle_;
                            msgDartParams.primary_force = motor_controller::MotorTriggerLSController.
                                target_angle_with_rounds_;
                            msgDartParams.last_param_update_time = rmw_uros_epoch_millis();
                        }
                        // 从msgDartParams里获取新值
                        motor_controller::MotorTriggerLSController.
                            target_angle_with_rounds_ = msgDartParams.primary_force;
                    }
                    else
                    {
                        if (motor_controller::MotorTriggerLSController.state_ !=
                            motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL)
                            motor_controller::MotorTriggerLSController.set_state(
                                motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
                        if (RC_Data.ch2 <= 600)
                            motor_controller::MotorTriggerLSController.target_velocity_ = -8000;
                        else if (RC_Data.ch2 > 600 && RC_Data.ch2 <= 800)
                            motor_controller::MotorTriggerLSController.target_velocity_ = -1000;
                        else if (RC_Data.ch2 >= 1200 && RC_Data.ch2 < 1410)
                            motor_controller::MotorTriggerLSController.target_velocity_ = 1000;
                        else if (RC_Data.ch2 >= 1410)
                            motor_controller::MotorTriggerLSController.target_velocity_ = 8000;
                    }
                }
                // Load电机控制，后面的代码操作优先
                // < --- 950 --- 中点 --- 1400 --- >
                // Load导轨状态机
                // 0. Lock状态：Load电机不动，均角度闭环在当前位置
                // 1. Operating to Reload状态：Load电机向下运动到装填位置
                // 2. Operating to Launch状态：Load电机向上运动到发射位置
                // 3. Operating状态：遥控器控制Load电机运动
                int16_t base_velocity = 0;
                {
                    switch (fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State)
                    {
                    case 0:
                        base_velocity = 0;

                    // 状态转移
                        if (RC_Data.ch3 >= 1600)
                        {
                            fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 2;
                        }
                        else if ((RC_Data.ch3 > 1400 && RC_Data.ch3 < 1600) |
                            (RC_Data.ch3 >= 366 && RC_Data.ch3 < 950))
                        {
                            fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 3;
                        }
                        else if (RC_Data.ch3 < 366)
                        {
                            fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 1;
                        }
                        break;
                    case 1:
                        base_velocity = CONFIG_MOTOR_LOAD_OPERATION_VELOCITY_DOWNWARD;
                    // 状态转移
                        if ((RC_Data.ch3 >= 1600) |
                            (motor_controller::MotorLoadController[0].current_angle_with_rounds_ >=
                                CONFIG_MOTOR_LOAD_ANGLE_DOWN |
                                motor_controller::MotorLoadController[1].current_angle_with_rounds_ >=
                                CONFIG_MOTOR_LOAD_ANGLE_DOWN))
                        {
                            fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 0;
                            base_velocity = 0;
                        }
                        break;

                    case 2:
                        base_velocity = -CONFIG_MOTOR_LOAD_OPERATION_VELOCITY_DOWNWARD;
                    // 状态转移
                        if ((RC_Data.ch3 < 950) | (
                            motor_controller::MotorLoadController[0].current_angle_with_rounds_ <=
                            CONFIG_MOTOR_LOAD_ANGLE_UP |
                            motor_controller::MotorLoadController[1].current_angle_with_rounds_ <=
                            CONFIG_MOTOR_LOAD_ANGLE_UP))
                        {
                            fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 0;
                            base_velocity = 0;
                        }
                        break;

                    case 3:
                        // 下
                        if (RC_Data.ch3 <= 950 &&
                            !(motor_controller::MotorLoadController[0].current_angle_with_rounds_ >=
                                CONFIG_MOTOR_LOAD_ANGLE_DOWN |
                                motor_controller::MotorLoadController[1].current_angle_with_rounds_ >=
                                CONFIG_MOTOR_LOAD_ANGLE_DOWN))
                        {
                            base_velocity = CONFIG_MOTOR_LOAD_OPERATION_VELOCITY_DOWNWARD;
                        }
                        // 上
                        else if (RC_Data.ch3 >= 1400 && !((
                            motor_controller::MotorLoadController[0].current_angle_with_rounds_ <=
                            CONFIG_MOTOR_LOAD_ANGLE_UP |
                            motor_controller::MotorLoadController[1].current_angle_with_rounds_ <=
                            CONFIG_MOTOR_LOAD_ANGLE_UP)))
                        {
                            base_velocity = -CONFIG_MOTOR_LOAD_OPERATION_VELOCITY_DOWNWARD;
                        }

                    // 状态转移
                        if (RC_Data.ch3 < 366)
                        {
                            fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 1;
                        }
                        else if (RC_Data.ch3 >= 1600)
                        {
                            fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 2;
                        }
                        else if (RC_Data.ch3 >= 950 && RC_Data.ch3 < 1400)
                        {
                            fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 0;
                        }

                        break;
                    default:
                        fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 0;
                    }
                }

                // 解除扳机控制
                switch (fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reset_State)
                {
                case 0:
                    if (RC_Data.Switch_Left == RC_SW_UP && RC_Data.ch4_wheel <= 514)
                    {
                        fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reset_State = 1;
                    }
                    break;
                case 1:
                    // 将拉簧拉到底
                    // 装填电机向下运动到装填位置
                    base_velocity = CONFIG_MOTOR_LOAD_OPERATION_VELOCITY_DOWNWARD;
                    if (motor_controller::MotorLoadController[0].current_angle_with_rounds_ >=
                        CONFIG_MOTOR_LOAD_ANGLE_DOWN |
                        motor_controller::MotorLoadController[1].current_angle_with_rounds_ >=
                        CONFIG_MOTOR_LOAD_ANGLE_DOWN)
                    {
                        fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reset_State = 2;
                        setTriggerServotoTrigger();
                        fsm.custom<Dart_FSM>()->ActionGeneral_Timer3_ = xTaskGetTickCount();
                    }
                    break;
                case 2:
                    // 等待一小会，舵机到位
                    if (xTaskGetTickCount() - fsm.custom<Dart_FSM>()->ActionGeneral_Timer3_ >
                        pdMS_TO_TICKS(CONFIG_LAUNCH_WAIT_TIME))
                    {
                        fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reset_State = 3;
                    }
                    break;
                case 3:
                    // 装填电机向上运动到初始位置
                    base_velocity = -CONFIG_MOTOR_LOAD_OPERATION_VELOCITY_DOWNWARD;
                    if (motor_controller::MotorLoadController[0].current_angle_with_rounds_ <=
                        CONFIG_MOTOR_LOAD_ANGLE_UP |
                        motor_controller::MotorLoadController[1].current_angle_with_rounds_ <=
                        CONFIG_MOTOR_LOAD_ANGLE_UP)
                    {
                        fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reset_State = 4;
                        setTriggerServotoReload();
                    }
                    break;
                case 4:
                    // 等待Wheel复位
                    if (RC_Data.ch4_wheel > 514)
                    {
                        fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reset_State = 0;
                    }
                    break;
                default:
                    fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reset_State = 0;
                }

                // 升降机装填控制
                switch (fsm.custom<Dart_FSM>()->ActionRemoteandReload_Slidedown_State)
                {
                case 0:
                    if (RC_Data.Switch_Left == RC_SW_MID && RC_Data.ch4_wheel <= 514)
                    {
                        fsm.custom<Dart_FSM>()->ActionRemoteandReload_Slidedown_State = 1;
                        setSlidedownServotoSlide();
                        fsm.custom<Dart_FSM>()->ActionGeneral_Timer2_ = xTaskGetTickCount();
                    }
                    break;
                case 1:
                    if (xTaskGetTickCount() - fsm.custom<Dart_FSM>()->ActionGeneral_Timer2_ >
                        pdMS_TO_TICKS(CONFIG_SLIDE_SERVO_SLIDE_TIME))
                    {
                        fsm.custom<Dart_FSM>()->ActionRemoteandReload_Slidedown_State = 2;
                        setSlidedownServotoCut();
                    }
                    break;
                case 2:
                    // 等待遥控器复位
                    if (RC_Data.ch4_wheel > 514)
                        fsm.custom<Dart_FSM>()->ActionRemoteandReload_Slidedown_State = 0;
                    break;
                default:
                    fsm.custom<Dart_FSM>()->ActionRemoteandReload_Slidedown_State = 0;
                }

                // 升降机控制
                switch (fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reload_State)
                {
                case 0:
                    if (RC_Data.ch4_wheel >= 1622 && RC_Data.Switch_Left == RC_SW_MID)
                    {
                        fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reload_State = 1;
                    }
                    break;
                case 1:
                    // 装填电机向下运动到装填位置
                    base_velocity = CONFIG_MOTOR_LOAD_OPERATION_VELOCITY_DOWNWARD;
                    if (motor_controller::MotorLoadController[0].current_angle_with_rounds_ >=
                        CONFIG_MOTOR_LOAD_ANGLE_DOWN |
                        motor_controller::MotorLoadController[1].current_angle_with_rounds_ >=
                        CONFIG_MOTOR_LOAD_ANGLE_DOWN)
                    {
                        fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reload_State = 2;
                        fsm.custom<Dart_FSM>()->ActionGeneral_Timer0_ = xTaskGetTickCount();
                        setLoadServotoDOWN();
                        setTriggerServotoTrigger();
                    }
                    break;
                case 2:
                    // 降下升降机并等待时间到达
                    if (xTaskGetTickCount() - fsm.custom<Dart_FSM>()->ActionGeneral_Timer0_ >
                        pdMS_TO_TICKS(CONFIG_LIFT_WAIT_TIME))
                    {
                        fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reload_State = 3;
                    }
                    break;
                case 3:
                    base_velocity = -(CONFIG_MOTOR_LOAD_OPERATION_VELOCITY_DOWNWARD);
                // 装填电机向上运动到发射位置
                    if (motor_controller::MotorLoadController[0].current_angle_with_rounds_ <=
                        CONFIG_MOTOR_LOAD_ANGLE_POST_LOAD |
                        motor_controller::MotorLoadController[1].current_angle_with_rounds_ <=
                        CONFIG_MOTOR_LOAD_ANGLE_POST_LOAD)
                    {
                        fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reload_State = 4;
                        setLoadServotoUP();
                        setTriggerServotoReload();
                    }
                    break;
                case 4:
                    // 等待Wheel复位
                    if (RC_Data.ch4_wheel < 1622)
                    {
                        fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reload_State = 0;
                    }
                }

                motor_controller::MotorLoadController[0].target_velocity_ =
                    base_velocity + motor_controller::MotorLoadSyncController.output;
                motor_controller::MotorLoadController[1].target_velocity_ =
                    base_velocity - motor_controller::MotorLoadSyncController.output;
            }
            else if (RC_Data.Switch_Left == RC_SW_DOWN)
            {
                int16_t base_velocity = 0;
                motor_controller::MotorTriggerLSController.target_angle_with_rounds_
                    = msgDartParams.primary_force;
                if (!fsm.custom<Dart_FSM>()->launch_operating_)
                {
                    // 解锁扳机，内八触发一次发射，Load电机带动同步带到顶端，扳机解锁
                    bool launch_grant_ = false;
                    if ((RC_Data.ch2 > 1400 && RC_Data.ch0 < 400))
                        launch_grant_ = true;

                    if (launch_grant_)
                    {
                        meter::velocity_meter.enable();
                        fsm.custom<Dart_FSM>()->launch_operating_ = true;
                        soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_approach));
                    }
                }
                else
                {
                    if (!fsm.custom<Dart_FSM>()->ActionRemote_launch_complete_)
                    {
                        if (motor_controller::MotorLoadController[0].current_angle_with_rounds_ <=
                            CONFIG_MOTOR_LOAD_ANGLE_UP)
                        {
                            base_velocity = -CONFIG_MOTOR_LOAD_OPERATION_VELOCITY_DOWNWARD / 5;
                        }
                        else
                        {
                            base_velocity = -CONFIG_MOTOR_LOAD_OPERATION_VELOCITY_DOWNWARD;
                        }
                        if (motor_controller::MotorLoadController[0].current_angle_with_rounds_ <=
                            CONFIG_MOTOR_LOAD_ANGLE_LAUNCH |
                            motor_controller::MotorLoadController[1].current_angle_with_rounds_ <=
                            CONFIG_MOTOR_LOAD_ANGLE_LAUNCH)
                        {
                            fsm.custom<Dart_FSM>()->ActionRemote_launch_complete_ = true;
                            fsm.custom<Dart_FSM>()->ActionGeneral_Timer1_ = xTaskGetTickCount();
                            base_velocity = 0;
                            setTriggerServotoTrigger();

                            motor_controller::MotorLoadController[0].set_state(
                                motor_controller::E_PID_Velocity_Angle_Controller_State::OPEN_LOOP);
                            motor_controller::MotorLoadController[0].target_openloop_ =
                                CONFIG_TARGET_RESET_VELOCITY_LOAD;

                            motor_controller::MotorLoadController[1].set_state(
                                motor_controller::E_PID_Velocity_Angle_Controller_State::OPEN_LOOP);

                            motor_controller::MotorLoadController[1].target_openloop_ =
                                CONFIG_TARGET_RESET_VELOCITY_LOAD;
                        }
                    }
                    else
                    {
                        if (xTaskGetTickCount() - fsm.custom<Dart_FSM>()->ActionGeneral_Timer1_ >
                            pdMS_TO_TICKS(CONFIG_LAUNCH_WAIT_TIME))
                        {
                            setTriggerServotoReload();
                            // 等待发射信号解除
                            if ((RC_Data.ch2 <= 1400 && RC_Data.ch0 >= 400))
                            {
                                fsm.custom<Dart_FSM>()->ActionRemote_launch_complete_ = false;
                                fsm.custom<Dart_FSM>()->launch_operating_ = false;
                                // 切闭环
                                motor_controller::MotorLoadController[0].set_state(
                                    motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
                                motor_controller::MotorLoadController[1].set_state(
                                    motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
                            }
                        }
                    }
                }

                // 设置base_velocity
                motor_controller::MotorLoadController[0].target_velocity_ =
                    base_velocity + motor_controller::MotorLoadSyncController.output;
                motor_controller::MotorLoadController[1].target_velocity_ =
                    base_velocity - motor_controller::MotorLoadSyncController.output;
            }
            setNextStateByRemote();
        }
    };

    class ActionMatch_Enter : public OpenFSMAction
    {
        void enter(OpenFSM& fsm) const override
        {
            // 比赛状态
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_never_forget));
            msgDartStatus.dart_launch_process = msgDartProtocols.dart_launch_process_offset_begin;
            msgDartStatus.dart_state = E_Match_Actions::Enter + E_Dart_State::Match;

            setLoadServotoUP();

            fsm.custom<Dart_FSM>()->ActionMatch_Wait_Continuous_Fire = false;
            msgDartStatus.dart_state = dart_fsm.openFSM_.focusEState() + 0;
            msgDartStatus.primary_yaw_offset = 0;
        }

        void update(OpenFSM& fsm) const override
        {
            setNextStateByRemote(false, true);
            // 将Yaw，Trigger电机置于打击协议位置，将装填电机置于上端
            motor::MotorLoad[0].setNextState(motor::E_MotorState::RUNNING);
            motor::MotorLoad[1].setNextState(motor::E_MotorState::RUNNING);
            motor::MotorYawLS.setNextState(motor::E_MotorState::RUNNING);
            motor::MotorTriggerLS.setNextState(motor::E_MotorState::RUNNING);
            motor_controller::MotorTriggerLSController.set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);
            motor_controller::MotorYawLSController.set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);
            motor_controller::MotorLoadController[0].set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
            motor_controller::MotorLoadController[1].set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);

            motor_controller::MotorTriggerLSController.target_angle_with_rounds_ =
                msgDartProtocols.primary_force + msgDartProtocols.primary_force_offset;
            motor_controller::MotorYawLSController.target_angle_with_rounds_ = msgDartProtocols.primary_yaw;

            // 计算base_velocity
            int base_velocity, load_reset_complete = false;
            if (motor_controller::MotorLoadController[0].current_angle_with_rounds_ <= CONFIG_MOTOR_LOAD_ANGLE_UP)
            {
                base_velocity = 0;
                // 直到彻底停下来之后认为load_reset_complete = true
                if (abs(motor::MotorLoad[0].current_velocity_ - 0) < 10 &&
                    abs(motor::MotorLoad[1].current_velocity_ - 0) < 10)
                {
                    load_reset_complete = true;
                }
            }
            else
            {
                base_velocity = -CONFIG_MOTOR_LOAD_OPERATION_VELOCITY_DOWNWARD;
            }

            motor_controller::MotorLoadController[0].target_velocity_ =
                base_velocity + motor_controller::MotorLoadSyncController.output;
            motor_controller::MotorLoadController[1].target_velocity_ =
                base_velocity - motor_controller::MotorLoadSyncController.output;

            // MotorYawLS、MotorTriggerLS、MotorLoad到达目标位置
            if (abs(motor_controller::MotorTriggerLSController.current_angle_with_rounds_ -
                    msgDartProtocols.primary_force) < 10000 &&
                abs(motor_controller::MotorYawLSController.current_angle_with_rounds_ -
                    msgDartProtocols.primary_yaw) < 1000 && load_reset_complete)
            {
                fsm.nextAction();
            }
        }

        void exit(OpenFSM& fsm) const override
        {
        }
    };

    class ActionMatch_Wait : public OpenFSMAction
    {
        void enter(OpenFSM& fsm) const override
        {
            // 判断是否连续发射，如果是的话就跳过该action
            if (fsm.custom<Dart_FSM>()->ActionMatch_Wait_Continuous_Fire)
            {
                fsm.custom<Dart_FSM>()->ActionMatch_Wait_Continuous_Fire = false;
                soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_winxp));
                fsm.nextAction();
                return;
            }

            motor_controller::MotorLoadSyncController.reset();

            msgDartStatus.dart_state = dart_fsm.openFSM_.focusEState() + 1;
            fsm.custom<Dart_FSM>()->ActionMatch_Wait_last_game_progress = ext_game_status.game_progress;
        }

        void update(OpenFSM& fsm) const override
        {
            setNextStateByRemote(false, true);
            // 将Yaw，Trigger电机置于打击协议位置，将装填电机置于上端
            motor::MotorLoad[0].setNextState(motor::E_MotorState::RUNNING);
            motor::MotorLoad[1].setNextState(motor::E_MotorState::RUNNING);
            motor::MotorYawLS.setNextState(motor::E_MotorState::RUNNING);
            motor::MotorTriggerLS.setNextState(motor::E_MotorState::RUNNING);
            motor_controller::MotorTriggerLSController.set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);
            motor_controller::MotorYawLSController.set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);
            motor_controller::MotorLoadController[0].set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
            motor_controller::MotorLoadController[1].set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);

            // TODO:计算好primary_yaw_offset的值，并进行调试
            motor_controller::MotorTriggerLSController.target_angle_with_rounds_ =
                msgDartProtocols.primary_force + msgDartProtocols.primary_force_offset +
                msgDartProtocols.auxiliary_force_offsets[msgDartStatus.dart_launch_process];

            // 判断是否更新自瞄
            if (!updateAutoAim(msgDartProtocols) && !msgDartProtocols.auto_aim_enabled)
                motor_controller::MotorYawLSController.
                    target_angle_with_rounds_ = msgDartProtocols.primary_yaw;

            // 读取裁判系统变量，线程安全
            uint8_t dart_launch_opening_status = ext_dart_client_cmd.dart_launch_opening_status;
            uint8_t game_progress = ext_game_status.game_progress;
            uint8_t dart_remaining_time = ext_dart_info.dart_remaining_time;
            uint16_t latest_launch_cmd_time = ext_dart_client_cmd.latest_launch_cmd_time;

            // 准备阶段结束的时候，将自瞄值offsets更新到primary内
            if (fsm.custom<Dart_FSM>()->ActionMatch_Wait_last_game_progress == 2 &&
                game_progress == 3)
            {
                msgDartProtocols.primary_yaw = msgDartProtocols.primary_yaw + msgDartStatus.primary_yaw_offset;
                // 重置自瞄控制器
                motor_controller::AutoAimController.reset();
                msgDartProtocols.last_param_update_time = rmw_uros_epoch_millis();
            }

            // 等待发射信号
            // TODO: 比赛内开启飞镖闸门就预位准备发射，最速化发射
            bool launch_grant_ = false;
            bool pre_launch_grant = false;

            // ====== 自动信号域 ======

            // 信号一：裁判系统飞镖闸门从“正在开启”达到“完全开启”信号，同时比赛正常进行中
            launch_grant_ |= (last_dart_gate_opening_status_ == E_Gate_State::OPERATING &&
                dart_launch_opening_status == E_Gate_State::OPENED &&
                game_progress == 4);

            pre_launch_grant |= (last_dart_gate_opening_status_ == E_Gate_State::CLOSED &&
                dart_launch_opening_status == E_Gate_State::OPERATING &&
                game_progress == 4);

            // 信号二：飞镖发射剩余时间变化，时间落在15s内，而且比赛进行中
            launch_grant_ |= (dart_remaining_time > 0 &&
                dart_remaining_time <= 15 &&
                game_progress == 4);

            // 信号三：选手端手动发送触发
            // 比赛状态确认
            if (latest_launch_cmd_time != 0 &&
                latest_launch_cmd_time != last_dart_launch_time_ &&
                game_progress == 4)
            {
                last_dart_launch_time_ = latest_launch_cmd_time;
                launch_grant_ = true;
            }

            // 自动信号触发时均要求二连发
            if (launch_grant_)
                fsm.custom<Dart_FSM>()->ActionMatch_Wait_Continuous_Fire = true;

            // ===== 手动信号域 =====
            // 信号四：遥控器信号 外八字，不要求比赛进行中 要求不在上场模式
            if (((RC_Data.ch0 > 1400 && RC_Data.ch2 < 400) && (!match_flag_)) ||
                ((RC_Data.ch0 > 1400 && RC_Data.ch2 < 400) && game_progress == 4 && (match_flag_)))
            {
                launch_grant_ = true;
                // 如果是手动发射，则不允许二连发，以防空放
                if (!match_flag_)
                    fsm.custom<Dart_FSM>()->ActionMatch_Wait_Continuous_Fire = false;
                    // 比赛中手动发射允许二连发
                else
                    fsm.custom<Dart_FSM>()->ActionMatch_Wait_Continuous_Fire = true;
            }

            // 内八模拟闸门从关闭切换到开启中的信号
            if (((RC_Data.ch0 < 1400 && RC_Data.ch2 > 400) && (!match_flag_)) ||
                ((RC_Data.ch0 < 1400 && RC_Data.ch2 > 400) && game_progress == 4 && (match_flag_)))
            {
                pre_launch_grant = true;
            }

            // 门控 比赛时间不足\准备阶段\自检时\自瞄进行中 拒绝发射
            if ((ext_game_status.stage_remain_time < 10 && game_progress == 4) || game_progress == 1 ||
                game_progress == 2 || game_progress == 3 || game_progress == 5)
            {
                launch_grant_ = false;
                fsm.custom<Dart_FSM>()->ActionMatch_Wait_Continuous_Fire = false;
            }

            last_dart_gate_opening_status_ = dart_launch_opening_status;


            motor_controller::MotorLoadController[0].target_velocity_ =
                0 + motor_controller::MotorLoadSyncController.output;
            motor_controller::MotorLoadController[1].target_velocity_ =
                0 - motor_controller::MotorLoadSyncController.output;


#if CONFIG_FORCE_WAIT_FOR_GAME_PROGRESS == 1
            if (ext_game_status.game_progress != 4) {
                // 等待比赛开始，未开始则不允许发射
                fsm.custom<Dart_FSM>()->ActionMatch_Wait_Continuous_Fire = false;
                return;
            }
#endif

            if (pre_launch_grant)
            {
                // TODO: 将双装填电机都拉到最下面
            }

            if (launch_grant_)
            {
                dart_mcu_log("Launch granted!");
                fsm.nextAction();
            }
        }

        void exit(OpenFSM& fsm) const override
        {
            soundEffectManager.clearSoundEffects();
        }
    };

    class ActionMatch_Launch : public OpenFSMAction
    {
        void enter(OpenFSM& fsm) const override
        {
            // 比赛状态
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_approach));
            fsm.custom<Dart_FSM>()->ActionGeneral_Timer1_ = xTaskGetTickCount();
            fsm.custom<Dart_FSM>()->ActionMatch_Launch_State = 0;

            setTriggerServotoReload();


            msgDartStatus.dart_state = dart_fsm.openFSM_.focusEState() + 2;
        }

        void update(OpenFSM& fsm) const override
        {
            setNextStateByRemote();
            // 开启电机控制
            motor::MotorLoad[0].setNextState(motor::E_MotorState::RUNNING);
            motor::MotorLoad[1].setNextState(motor::E_MotorState::RUNNING);
            motor::MotorYawLS.setNextState(motor::E_MotorState::RUNNING);
            motor::MotorTriggerLS.setNextState(motor::E_MotorState::RUNNING);

            // 控制器模式：Yaw/Trigger均为角度 Load为速度
            motor_controller::MotorYawLSController.set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);
            motor_controller::MotorTriggerLSController.set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);

            motor_controller::MotorTriggerLSController.target_angle_with_rounds_ =
                msgDartProtocols.primary_force + msgDartProtocols.primary_force_offset +
                msgDartProtocols.auxiliary_force_offsets[msgDartStatus.dart_launch_process];


            // Launch里面有几种连续状态：
            // 0: Wait Autoaim 1: Wait stable 2: Downward 3: Upward 4: Trigger 5. Restore Trigger
            double base_velocity = 0;
            switch (fsm.custom<Dart_FSM>()->ActionMatch_Launch_State)
            {
            case 0:
                // 目标位置
                if (msgDartProtocols.auto_aim_enabled)
                {
                    if (xTaskGetTickCount() -
                        fsm.custom<Dart_FSM>()->ActionGeneral_Timer1_ < CONFIG_AUTOAIM_TIMEOUT_MS
                    )
                    {
                        updateAutoAim(msgDartProtocols);
                    }
                    else
                    {
                        // 按照飞镖专属参数进行发射
                        motor_controller::MotorYawLSController.target_angle_with_rounds_ =
                            msgDartProtocols.primary_yaw + msgDartStatus.primary_yaw_offset +
                            msgDartProtocols.auxiliary_yaw_offsets[msgDartStatus.dart_launch_process];
                        fsm.custom<Dart_FSM>()->ActionMatch_Launch_State = 1;
                        fsm.custom<Dart_FSM>()->ActionGeneral_Timer1_ = xTaskGetTickCount();
                    }
                }
                else
                {
                    motor_controller::MotorYawLSController.target_angle_with_rounds_ =
                        msgDartProtocols.primary_yaw +
                        msgDartProtocols.auxiliary_yaw_offsets[msgDartStatus.dart_launch_process];
                    fsm.custom<Dart_FSM>()->ActionMatch_Launch_State = 1;
                    fsm.custom<Dart_FSM>()->ActionGeneral_Timer1_ = xTaskGetTickCount();
                }
                break;
            case 1:
                // 等待电机稳定
                if (xTaskGetTickCount() - fsm.custom<Dart_FSM>()->ActionGeneral_Timer1_ >
                    pdMS_TO_TICKS(CONFIG_LAUNCH_WAIT_MOTOR_STABLE_TIME))
                {
                    fsm.custom<Dart_FSM>()->ActionMatch_Launch_State = 2;
                }
                break;
            case 2:
                // 向下运动
                base_velocity = CONFIG_MOTOR_LOAD_OPERATION_VELOCITY_DOWNWARD;
                if (motor_controller::MotorLoadController[0].current_angle_with_rounds_ >=
                    CONFIG_MOTOR_LOAD_ANGLE_DOWN ||
                    motor_controller::MotorLoadController[1].current_angle_with_rounds_ >=
                    CONFIG_MOTOR_LOAD_ANGLE_DOWN)
                {
                    meter::velocity_meter.enable();
                    fsm.custom<Dart_FSM>()->launch_operating_ = true;
                    soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_approach));
                    fsm.custom<Dart_FSM>()->ActionMatch_Launch_State = 3;
                }
                break;

            case 3:
                // 向上运动
                if (motor_controller::MotorLoadController[0].current_angle_with_rounds_ <=
                    CONFIG_MOTOR_LOAD_ANGLE_UP)
                {
                    base_velocity = -CONFIG_MOTOR_LOAD_OPERATION_VELOCITY_DOWNWARD / 5;
                }
                else
                {
                    base_velocity = -CONFIG_MOTOR_LOAD_OPERATION_VELOCITY_DOWNWARD;
                }
                if (motor_controller::MotorLoadController[0].current_angle_with_rounds_ <=
                    CONFIG_MOTOR_LOAD_ANGLE_LAUNCH ||
                    motor_controller::MotorLoadController[1].current_angle_with_rounds_ <= -
                    CONFIG_MOTOR_LOAD_ANGLE_LAUNCH)
                {
                    fsm.custom<Dart_FSM>()->ActionMatch_Launch_State = 4;
                    fsm.custom<Dart_FSM>()->ActionGeneral_Timer0_ = xTaskGetTickCount();

                    motor_controller::MotorLoadController[0].set_state(
                        motor_controller::E_PID_Velocity_Angle_Controller_State::OPEN_LOOP);
                    motor_controller::MotorLoadController[0].target_openloop_ = CONFIG_TARGET_RESET_VELOCITY_LOAD;

                    motor_controller::MotorLoadController[1].set_state(
                        motor_controller::E_PID_Velocity_Angle_Controller_State::OPEN_LOOP);

                    motor_controller::MotorLoadController[1].target_openloop_ = CONFIG_TARGET_RESET_VELOCITY_LOAD;
                }
                break;

            case 4:
                // 扳机丝杆扣下后等待
                setTriggerServotoTrigger();
                base_velocity = 0;
                if (xTaskGetTickCount() - fsm.custom<Dart_FSM>()->ActionGeneral_Timer0_ >
                    pdMS_TO_TICKS(CONFIG_LAUNCH_WAIT_TIME))
                {
                    setTriggerServotoReload();
                    // 切闭环
                    motor_controller::MotorLoadController[0].set_state(
                        motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
                    motor_controller::MotorLoadController[1].set_state(
                        motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
                    fsm.nextAction();
                }
                break;
            default:
                fsm.custom<Dart_FSM>()->ActionMatch_Launch_State = 0;
            }

            // 设置Load电机速度
            motor_controller::MotorLoadController[0].target_velocity_ =
                base_velocity + motor_controller::MotorLoadSyncController.output;
            motor_controller::MotorLoadController[1].target_velocity_ =
                base_velocity - motor_controller::MotorLoadSyncController.output;
        }

        void exit(OpenFSM& fsm) const override
        {
            msgDartStatus.dart_launch_process++;
            if (msgDartStatus.dart_launch_process >
                msgDartProtocols.dart_launch_process_offset_end)
            {
                msgDartStatus.dart_launch_process = msgDartProtocols.dart_launch_process_offset_begin;
            }
        }
    };

    // 拉到底，触发一下装填阻挡舵机，
    class ActionMatch_Reload : public OpenFSMAction
    {
        void enter(OpenFSM& fsm) const override
        {
            // 比赛状态
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_winxp));
            // 重置状态机
            fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reload_State = 0;
            fsm.custom<Dart_FSM>()->ActionRemoteandReload_Slidedown_State = 0;

            // 判断是否需要等待下滑
            // 从方便和装填一致性的角度来说，想飞的镖少的时候，直接按照滑台-装填-导轨1-导轨2的队列填充。
            fsm.custom<Dart_FSM>()->ActionReload_Slidedown_Judge = false;
            if (msgDartProtocols.dart_launch_process_offset_end -
                msgDartProtocols.dart_launch_process_offset_begin >= 2)
                if (msgDartStatus.dart_launch_process -
                    msgDartProtocols.dart_launch_process_offset_begin >= 2)
                    fsm.custom<Dart_FSM>()->ActionReload_Slidedown_Judge = true;


            msgDartStatus.dart_state = dart_fsm.openFSM_.focusEState() + 3;
        }

        void update(OpenFSM& fsm) const override
        {
            // 启用Load电机并设置为速度模式
            motor::MotorLoad[0].setNextState(motor::E_MotorState::RUNNING);
            motor::MotorLoad[1].setNextState(motor::E_MotorState::RUNNING);
            motor::MotorYawLS.setNextState(motor::E_MotorState::RUNNING);
            motor::MotorTriggerLS.setNextState(motor::E_MotorState::RUNNING);

            motor_controller::MotorLoadController[0].set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
            motor_controller::MotorLoadController[1].set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);

            motor_controller::MotorYawLSController.set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);

            motor_controller::MotorTriggerLSController.set_state(
                motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);

            double base_velocity = 0;

            // 升降机装填控制
            switch (fsm.custom<Dart_FSM>()->ActionRemoteandReload_Slidedown_State)
            {
            case 0:
                // 触发下滑
                if (fsm.custom<Dart_FSM>()->ActionReload_Slidedown_Judge)
                {
                    fsm.custom<Dart_FSM>()->ActionRemoteandReload_Slidedown_State = 1;
                    setSlidedownServotoSlide();
                    fsm.custom<Dart_FSM>()->ActionGeneral_Timer2_ = xTaskGetTickCount();
                }
                break;
            case 1:
                if (xTaskGetTickCount() - fsm.custom<Dart_FSM>()->ActionGeneral_Timer2_ >
                    pdMS_TO_TICKS(CONFIG_SLIDE_SERVO_SLIDE_TIME))
                {
                    fsm.custom<Dart_FSM>()->ActionRemoteandReload_Slidedown_State = 2;
                    setSlidedownServotoCut();
                    // 操作完成，触发升降机下落和装填
                    fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reload_State = 1;
                }
                break;
            case 2:
                break;
            default:
                fsm.custom<Dart_FSM>()->ActionRemoteandReload_Slidedown_State = 0;
            }

            // 升降机控制
            switch (fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reload_State)
            {
            case 0:
                // 触发升降机
                if (!fsm.custom<Dart_FSM>()->ActionReload_Slidedown_Judge)
                {
                    fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reload_State = 1;
                }
                break;
            case 1:
                // 装填电机向下运动到装填位置
                base_velocity = CONFIG_MOTOR_LOAD_OPERATION_VELOCITY_DOWNWARD;
                if (motor_controller::MotorLoadController[0].current_angle_with_rounds_ >=
                    CONFIG_MOTOR_LOAD_ANGLE_DOWN |
                    motor_controller::MotorLoadController[1].current_angle_with_rounds_ >=
                    CONFIG_MOTOR_LOAD_ANGLE_DOWN)
                {
                    fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reload_State = 2;
                    fsm.custom<Dart_FSM>()->ActionGeneral_Timer0_ = xTaskGetTickCount();
                    setLoadServotoDOWN();
                    setTriggerServotoTrigger();
                }
                break;
            case 2:
                // 降下升降机并等待时间到达
                if (xTaskGetTickCount() - fsm.custom<Dart_FSM>()->ActionGeneral_Timer0_ >
                    pdMS_TO_TICKS(CONFIG_LIFT_WAIT_TIME))
                {
                    fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reload_State = 3;
                }
                break;
            case 3:
                base_velocity = -(CONFIG_MOTOR_LOAD_OPERATION_VELOCITY_DOWNWARD);
            // 装填电机向上运动到发射位置
                if (motor_controller::MotorLoadController[0].current_angle_with_rounds_ <=
                    CONFIG_MOTOR_LOAD_ANGLE_POST_LOAD |
                    motor_controller::MotorLoadController[1].current_angle_with_rounds_ <=
                    CONFIG_MOTOR_LOAD_ANGLE_POST_LOAD)
                {
                    fsm.custom<Dart_FSM>()->ActionRemoteandReload_Reload_State = 4;
                    setLoadServotoUP();
                    setTriggerServotoReload();
                    fsm.custom<Dart_FSM>()->ActionGeneral_Timer0_ = xTaskGetTickCount();
                }
                break;
            case 4:
                // 再等一小段时间等待装填复位
                if (xTaskGetTickCount() - fsm.custom<Dart_FSM>()->ActionGeneral_Timer0_ > CONFIG_LIFT_WAIT_TIME)
                    fsm.nextAction();
                break;
            }

            // 设置Load电机速度
            motor_controller::MotorLoadController[0].target_velocity_ =
                base_velocity + motor_controller::MotorLoadSyncController.output;
            motor_controller::MotorLoadController[1].target_velocity_ =
                base_velocity - motor_controller::MotorLoadSyncController.output;
        }

        void exit(OpenFSM& fsm) const override
        {
        }
    };

    void Dart_FSM::start()
    {
        OpenFSM::RegisterAction<ActionWaitForAllMotorOnline>("ActionWaitForAllMotorOnline");
        OpenFSM::RegisterAction<ActionResetMotors>("ActionResetMotors");
        OpenFSM::RegisterAction<ActionReleaseMotors>("ActionReleaseMotors");
        OpenFSM::RegisterAction<ActionProtect>("ActionProtect");
        OpenFSM::RegisterAction<ActionRemote>("ActionRemote");
        OpenFSM::RegisterAction<ActionMatch_Enter>("ActionMatch_Enter");
        OpenFSM::RegisterAction<ActionMatch_Wait>("ActionMatch_Wait");
        OpenFSM::RegisterAction<ActionMatch_Launch>("ActionMatch_Launch");
        OpenFSM::RegisterAction<ActionMatch_Reload>("ActionMatch_Reload");

        OpenFSM::RegisterState("StateBoot",
                               {"ActionWaitForAllMotorOnline", "ActionResetMotors", "ActionReleaseMotors"},
                               E_Dart_State::Boot);
        OpenFSM::RegisterState("StateProtect", {"ActionProtect"}, E_Dart_State::Protect);
        OpenFSM::RegisterState("StateRemote", {"ActionRemote"}, E_Dart_State::Remote);
        OpenFSM::RegisterState("StateMatch", {
                                   "ActionMatch_Enter", "ActionMatch_Wait", "ActionMatch_Launch",
                                   "ActionMatch_Reload", "ActionMatch_Wait", "ActionMatch_Launch",
                                   "ActionMatch_Reload", "ActionMatch_Wait", "ActionMatch_Launch",
                                   "ActionMatch_Reload", "ActionMatch_Wait", "ActionMatch_Launch",
                               },
                               E_Dart_State::Match);

        OpenFSM::RegisterRelation("StateBoot", {"StateProtect"});
        OpenFSM::RegisterRelation("StateProtect", {"StateRemote", "StateMatch"});
        OpenFSM::RegisterRelation("StateRemote", {"StateProtect", "StateMatch"});
        OpenFSM::RegisterRelation("StateMatch", {"StateProtect", "StateRemote"});

        openFSM_.setCustom(this);
        openFSM_.setStates({E_Dart_State::Boot, E_Dart_State::Protect, E_Dart_State::Remote, E_Dart_State::Match});

        openFSM_.enterState(E_Dart_State::Boot);

        dart_mcu_log("Fsm Initiated.");
    }

    void fsm_thread(void* parameters)
    {
        TickType_t last_time;

        dart_fsm.start();

        while (true)
        {
            dart_fsm.update();
            vTaskDelayUntil(&last_time, pdMS_TO_TICKS(1)); // 200Hz
        }
    }
} // state_machine
