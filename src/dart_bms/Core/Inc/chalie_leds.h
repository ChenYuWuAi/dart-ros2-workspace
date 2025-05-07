//
// 使用HAL库控制LED灯，LED通过查理复用连接PB7/PA7/PA6引脚
// Created by chenyu on 24-11-25.
//

#ifndef CHALIE_LEDS_H
#define CHALIE_LEDS_H
#include "stm32l0xx_hal.h"

#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_6
#define LED3_GPIO_Port GPIOA

enum led_states {
    LED_OFF = 0,
    LED_ON = 1,
    LED_BLINK = 2,
    LED_STATE_MAX
};

extern uint8_t led_state[4];

void chalie_leds_init(void); // Init Gpio
/**
 * @param led led num, 1-4
 * @param state 1: on, 0: off
 */
void chalie_leds_set(uint16_t led, GPIO_PinState state); // Set led state

/**
 * @param code 0-7le， 0%~12.5% 12.5%~25% 25%~37.5% 37.5%~50% 50%~62.5% 62.5%~75% 75%~87.5% 87.5%~100%
 */
void chalie_led_code(uint8_t code); // Show code on leds

void chalie_led_timer_update(uint8_t clear); // Update led state

#endif //CHALIE_LEDS_H
