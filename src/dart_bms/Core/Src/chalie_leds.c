//
// Created by chenyu on 24-11-25.
//

#include "chalie_leds.h"

uint8_t led_state[4] = {LED_OFF, LED_OFF, LED_OFF, LED_OFF};

void chalie_leds_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = LED1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LED2_Pin;
    HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LED3_Pin;
    HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);
}

void chalie_leds_set(uint16_t led, GPIO_PinState state) {
    switch (led) {
        case 3:
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
            break;
        case 4:
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case 1:
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
            break;
        default:
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
            break;
    }
}

void chalie_led_code(uint8_t code) {
    /*
    *- 灭 * 闪 0 亮
    0    *---
    1    0---
    2    0*--
    3    00--
    4    00*-
    5    000-
    6    000*
    7    0000
     */
    switch (code) {
        case 0:
            led_state[0] = LED_BLINK;
            led_state[1] = LED_OFF;
            led_state[2] = LED_OFF;
            led_state[3] = LED_OFF;
            break;
        case 1:
            led_state[0] = LED_ON;
            led_state[1] = LED_OFF;
            led_state[2] = LED_OFF;
            led_state[3] = LED_OFF;
            break;
        case 2:
            led_state[0] = LED_ON;
            led_state[1] = LED_BLINK;
            led_state[2] = LED_OFF;
            led_state[3] = LED_OFF;
            break;
        case 3:
            led_state[0] = LED_ON;
            led_state[1] = LED_ON;
            led_state[2] = LED_OFF;
            led_state[3] = LED_OFF;
            break;
        case 4:
            led_state[0] = LED_ON;
            led_state[1] = LED_ON;
            led_state[2] = LED_BLINK;
            led_state[3] = LED_OFF;
            break;
        case 5:
            led_state[0] = LED_ON;
            led_state[1] = LED_ON;
            led_state[2] = LED_ON;
            led_state[3] = LED_OFF;
            break;
        case 6:
            led_state[0] = LED_ON;
            led_state[1] = LED_ON;
            led_state[2] = LED_ON;
            led_state[3] = LED_BLINK;
            break;
        case 7:
            led_state[0] = LED_ON;
            led_state[1] = LED_ON;
            led_state[2] = LED_ON;
            led_state[3] = LED_ON;
            break;
        default:
            led_state[0] = LED_ON;
            led_state[1] = LED_ON;
            led_state[2] = LED_ON;
            led_state[3] = LED_ON;
            break;

    }
}

void chalie_led_timer_update(uint8_t clear) {
    // 由定时器定期调用，复用IO口实现LED闪烁，被调用的频率为100Hz，如果数组里的状态是闪烁状态/多亮状态，就通过不停的切换IO口状态来实现复用
    // 状态机，0->更新灯状态，1->更新显示周期
    enum {
        LED_STATE_UPDATE = 0,
        LED_STATE_PERIOD = 1
    };
    static uint8_t led_state_machine = LED_STATE_UPDATE;
    static uint8_t led_period_state = 0;
    static uint8_t led_period_state_max = 0;
    static uint8_t led_state_internal[4] = {LED_OFF, LED_OFF, LED_OFF, LED_OFF};

    if (clear) {
        led_state_machine = LED_STATE_UPDATE;
        led_period_state = 0;
        led_period_state_max = 0;
        for (int i = 0; i < 4; i++) {
            led_state_internal[i] = LED_OFF;
        }
    }

    switch (led_state_machine) {
        case LED_STATE_UPDATE:
            led_period_state_max = 0;
            for (int i = 0; i < 4; i++) {
                led_state_internal[i] = led_state[i];
                if (led_state[i] != LED_OFF)
                    led_period_state_max++;
            }
            led_state_machine = LED_STATE_PERIOD;
            break;
        case LED_STATE_PERIOD:
            chalie_leds_set(led_period_state + 1,
                            (led_state_internal[led_period_state] == LED_BLINK && HAL_GetTick() % 500 > 250)
                                ? GPIO_PIN_RESET
                                : GPIO_PIN_SET);
            led_period_state++;
            if (led_period_state >= led_period_state_max) {
                led_period_state = 0;
                led_state_machine = LED_STATE_UPDATE;
            }
            break;
    }
}
