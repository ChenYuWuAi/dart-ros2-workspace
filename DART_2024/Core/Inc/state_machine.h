//
// Created by cheny on 24-6-29.
//

#ifndef DART_24_STATE_MACHINE_H
#define DART_24_STATE_MACHINE_H

#include "openfsm.h"
#include "judge_receive.h"

#include "main.h"

using namespace openfsm;

namespace state_machine {

    [[noreturn]] void fsm_thread(void *parameters);

    // 状态机:
    // Update method: Broadcast
    // 0. 上电状态 复位 Action: Reset
    // 1. 保护状态 遥控器左打上
    // 2. 调试模式 遥控器左打中
    // 3. 比赛模式 Action = Wait, Launch, Reload, Reset 遥控器左打下
    // 4. 上控模式 - for ROS
    enum E_Dart_State {
        Boot = 100,
        Protect = 101,
        Remote = 102,
        Match = 103
    };

    enum E_Lead_Screw_Switch_State {
        Untriggered,
        Triggered
    };

    enum E_Match_Actions {
        Enter = 0,
        Wait,
        Launch,
        Reload,
        Undefined
    };

    struct Dart_FSM {
        OpenFSM openFSM_;

        void start();

        void update();
    };

    extern Dart_FSM dart_fsm;
}//namespace state_machine

#endif //DART_24_STATE_MACHINE_H
