//
// Created by cheny on 24-9-18.
//

#ifndef DART_MCU_DART_STATEMACHINE_H
#define DART_MCU_DART_STATEMACHINE_H

#include "openfsm.h"
#include "stdint.h"
#include "FreeRTOS.h"
#include "dartmcu_node.h"

extern uint16_t wheel;

using namespace openfsm;

namespace state_machine {
    [[noreturn]] void fsm_thread(void *parameters);

    // 状态机:
    // Update method: Broadcast
    // 0. 上电状态 复位 Action: Reset
    // 1. 保护状态 遥控器右打上
    // 2. 调试模式 遥控器右打中
    // 3. 比赛模式 Action = Wait, Launch, Reload, Reset 遥控器右打下
    enum E_Dart_State {
        Boot = 100,
        Protect = 101,
        Remote = 102,
        Match = 103
    };

    enum E_Match_Actions {
        Enter,
        Wait,
        Launch,
        Reload,
        Exit,
        Undefined
    };

    enum E_ResetActionReturnState {
        Operating,
        Finished,
        Failed
    };

    enum E_Gate_State {
        OPENED,
        CLOSED,
        OPERATING
    };

    enum E_Target_Type
    {
        Default, // 开局默认/未选定/前哨站
        Stationary, // 基地固定目标
        RandomStationary, // 基地随机固定目标
        RandomMoving // 基地随机移动目标
    };

    struct FSM {
        OpenFSM openFSM_;

        virtual void start() = 0;

        void update();
    };

    struct Dart_FSM : public FSM {
        bool boot_success = false;

        // ActionResetMotors
        uint8_t ActionResetMotors_Load_0_Reset_State = 0;
        uint8_t ActionResetMotors_Load_1_Reset_State = 0;
        uint8_t ActionResetMotors_both_initialized = 0;
        uint8_t ActionResetMotors_TriggerLS_Reset_State = 0;
        uint8_t ActionMatch_Launch_State = 0;
        TickType_t ActionGeneral_Timer0_;
        TickType_t ActionGeneral_Timer1_;
        TickType_t ActionGeneral_Timer2_;
        TickType_t ActionGeneral_Timer3_;
        // ActionRemote
        uint8_t ActionRemote_MotorLoad_State = 0;
        bool ActionRemoteandMatch_launch_complete_ = false;
        // Public
        // 0: 空闲 1: 将Load电机拉到装填位置 2: 降下升降机 3: 放开扳机舵机 4: 将Load电机拉到初始位置 5: 升起升降机
        uint8_t ActionRemoteandReload_Reload_State = 0;
        uint8_t ActionRemoteandReload_Slidedown_State = 0;
        uint8_t ActionRemoteandReload_Reset_State = 0;
        uint8_t ActionMatch_Wait_last_game_progress = 0;
        bool ActionReload_Slidedown_Judge = false;
        // 该值当且仅当发射信号确认，并且在比赛模式中的时候生效。为真则会跳过一次发射确认过程。
        bool ActionMatch_Wait_Continuous_Fire = false;
        bool launch_operating_ = false;
        bool reset_operating_ = false;

    public:
        void start();
    };

    extern Dart_FSM dart_fsm;
    enum class UpsideState {
        MovingUp,
        MovingDown,
        Idle
    };


    extern UpsideState upside_state;
} // state_machine

#endif //DART_MCU_DART_STATEMACHINE_H
