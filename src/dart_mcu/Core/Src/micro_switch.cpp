//
// Created by cheny on 24-9-18.
//

#include "micro_switch.h"

E_Lead_Screw_Switch_State yaw_switch_state;
E_Lead_Screw_Switch_State trigger_switch_state;

// 使用消抖状态机：
// 状态机的状态有两个：Untriggered, Waiting, Triggered
// Untriggered - 0: 未触发状态
// Waiting - 1: 等待状态，用于消抖
// Triggered - 2: 触发状态
// 未触发状态下，如果检测到触发，则进入等待状态，等待一段时间后判断是否进入触发状态

#define DEBOUNCE_TIME 10
#define read_switch_state_debounce_time(GPIO_Port, GPIO_Pin, switch_state) \
    do { \
        static uint32_t switch_state_last_time = 0; \
        static uint8_t switch_state_machine = 0; \
        if (HAL_GPIO_ReadPin(GPIO_Port, GPIO_Pin) == GPIO_PIN_RESET) { \
            if (switch_state_machine == 0) { \
                switch_state_machine = 1; \
                switch_state_last_time = HAL_GetTick(); \
            } else if (switch_state_machine == 1) { \
                if (HAL_GetTick() - switch_state_last_time > DEBOUNCE_TIME) { \
                    switch_state_machine = 2; \
                } \
            } \
        } else if((HAL_GPIO_ReadPin(GPIO_Port, GPIO_Pin) == GPIO_PIN_SET) && switch_state_machine == 1) { \
            if(HAL_GetTick() - switch_state_last_time > DEBOUNCE_TIME)   { \
                switch_state_machine = 0; \
            } \
        } else { \
            switch_state_machine = 0; \
        } \
        switch_state = switch_state_machine == 2 ? Triggered : Untriggered; \
    } while (0)

// 消抖读取
void micro_switch_read() {
    read_switch_state_debounce_time(Yaw_Switch_GPIO_Port, Yaw_Switch_Pin, yaw_switch_state);
    // read_switch_state_debounce_time(Trigger_Switch_GPIO_Port, Trigger_Switch_Pin, trigger_switch_state);
}