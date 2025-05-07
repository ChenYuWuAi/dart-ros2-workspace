//
// Created by cheny on 24-9-18.
//

#ifndef DART_MCU_MICRO_SWITCH_H
#define DART_MCU_MICRO_SWITCH_H

#include "gpio.h"

// 高电平触发
#define Yaw_Switch_GPIO_Port GPIOA
#define Yaw_Switch_Pin GPIO_PIN_0

#define Trigger_Switch_GPIO_Port GPIOA
#define Trigger_Switch_Pin GPIO_PIN_2

enum E_Lead_Screw_Switch_State {
    Untriggered,
    Triggered
};

extern E_Lead_Screw_Switch_State yaw_switch_state;
extern E_Lead_Screw_Switch_State trigger_switch_state;

void micro_switch_read();

#endif //DART_MCU_MICRO_SWITCH_H
