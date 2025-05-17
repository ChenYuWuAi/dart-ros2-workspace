//
// Created by cheny on 24-9-12.
//

#ifndef DART_MCU_LED_H
#define DART_MCU_LED_H

#include "gpio.h"
#include "stm32f4xx_hal_gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"

namespace LED {
    enum LED_Type {
        LED_RED,//PE11
        LED_GREEN,//PF14
        LED_FLOW_0,//PG1
        LED_FLOW_1,//PG2
        LED_FLOW_2,
        LED_FLOW_3,
        LED_FLOW_4,
        LED_FLOW_5,
        LED_FLOW_6,
        LED_FLOW_7//PG8
    };

    void setLED(LED_Type led, bool state);

    void toggleLED(LED_Type led);

    enum LED_Flow_State {
        FLOW_NORMAL,
        FLOW_NONE,
        FLOW_STATE_NUM
    };

    class LED_Flow {
    private:
        bool led_state_[8] = {false, false, false, false, false, false, false, false};
        static constexpr uint16_t flowStackSize = 256;
        StackType_t flowStack_[flowStackSize];
        StaticTask_t flowTaskBuffer_;
        TaskHandle_t flowTaskHandle_;
    public:
        uint8_t flow_state_ = FLOW_NONE;

        LED_Flow() = default;

        void begin();

        void updateStatetoLED();

        static void flowTask(void *pvParameters);
    };

    extern LED_Flow led_flow;
}

#endif //DART_MCU_LED_H
