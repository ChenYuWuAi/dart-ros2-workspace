//
// Created by cheny on 24-9-12.
//

#include "led.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"

namespace LED {

    LED_Flow led_flow;

    void setLED(LED_Type led, bool state) {
        state = !state;
        switch (led) {
            case LED_RED:
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, (GPIO_PinState) state);
                break;
            case LED_GREEN:
                HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, (GPIO_PinState) state);
                break;
            case LED_FLOW_0:
                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, (GPIO_PinState) state);
                break;
            case LED_FLOW_1:
                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, (GPIO_PinState) state);
                break;
            case LED_FLOW_2:
                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, (GPIO_PinState) state);
                break;
            case LED_FLOW_3:
                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, (GPIO_PinState) state);
                break;
            case LED_FLOW_4:
                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, (GPIO_PinState) state);
                break;
            case LED_FLOW_5:
                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, (GPIO_PinState) state);
                break;
            case LED_FLOW_6:
                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, (GPIO_PinState) state);
                break;
            case LED_FLOW_7:
                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, (GPIO_PinState) state);
                break;
        }
    }

    void toggleLED(LED_Type led) {
        switch (led) {
            case LED_RED:
                HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
                break;
            case LED_GREEN:
                HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
                break;
            case LED_FLOW_0:
                HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_1);
                break;
            case LED_FLOW_1:
                HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_2);
                break;
            case LED_FLOW_2:
                HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_3);
                break;
            case LED_FLOW_3:
                HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_4);
                break;
            case LED_FLOW_4:
                HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_5);
                break;
            case LED_FLOW_5:
                HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_6);
                break;
            case LED_FLOW_6:
                HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_7);
                break;
            case LED_FLOW_7:
                HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_8);
                break;
        }
    }

    void LED_Flow::begin() {
        for (int i = 0; i < 8; i++) {
            setLED((LED_Type) (LED_FLOW_0 + i), led_state_[i]);
        }
        flowTaskHandle_ = xTaskCreateStatic(
            flowTask,
            "LED_Flow",
            flowStackSize,
            this,
            18,
            flowStack_,
            &flowTaskBuffer_
        );
    }

    void LED_Flow::updateStatetoLED() {
        for (int i = 0; i < 8; i++) {
            setLED((LED_Type) (LED_FLOW_0 + i), led_state_[i]);
        }
    }

    void LED_Flow::flowTask(void *pvParameters) {
        LED_Flow *led_flow = (LED_Flow *) pvParameters;
        while (true) {
            switch (led_flow->flow_state_) {
                case FLOW_NORMAL: {
                    static uint8_t pointer = 0;
                    static uint8_t length = 0;
                    led_flow->led_state_[(pointer + 7) % 8] = false;
                    for (int i = 0; i < length; i++) {
                        led_flow->led_state_[(pointer + i) % 8] = true;
                    }
                    led_flow->updateStatetoLED();
                    pointer = (pointer + 1) % 8;
                    length = uint8_t(2.0 * sin((xTaskGetTickCount() * 3.1415926 / 4000.0)) + 4.0);
                    // 缓动效果，以6为周期正弦变化
                    vTaskDelay(sin((xTaskGetTickCount()) * 3.1415926 / 2000.0) * 20 + 100);
                    break;
                }
                case FLOW_NONE: {
                    memset(led_flow->led_state_, 0, 8);
                    led_flow->updateStatetoLED();
                    vTaskDelay(500);
                    break;
                }
                default:
                    vTaskDelay(100);
            }
        }
    }

};