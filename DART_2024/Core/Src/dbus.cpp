//
// Created by cheny on 24-6-29.
//
#include "dbus.h"
#include "usart.h"
#include "cmsis_os.h"

RCDecoding_Type RC_Data;
uint8_t RC_Rx_Mem = 0;
uint8_t RC_rx_buffer[18];

void DT7_init() {
    RC_Data.RCFrameCounter = 0; // FrameCounter

    RC_Rx_Mem = MEMORY0; // BufferSelect

    RC_Data.ch0 = RC_CH_VALUE_OFFSET; // ch0 offset
    RC_Data.ch1 = RC_CH_VALUE_OFFSET; // ch1 offset
    RC_Data.ch2 = RC_CH_VALUE_OFFSET; // ch2 offset
    RC_Data.ch3 = RC_CH_VALUE_OFFSET; // ch3 offset
    RC_Data.ch4_wheel = RC_CH_VALUE_OFFSET;

    RC_Data.Switch_Left = RC_SW_UP;     // switch left offset
    RC_Data.Switch_Right = RC_SW_UP; // switch right offset

    RC_Data.mouse.x = MOUSE_SPEED_OFFSET; // Mouse speed-x
    RC_Data.mouse.y = MOUSE_SPEED_OFFSET; // Mouse speed-y
    RC_Data.mouse.z = MOUSE_SPEED_OFFSET; // Mouse speed-z

    RC_Data.mouse.press_left = MOUSE_PRESSED_OFFSET;  // Mouse left button
    RC_Data.mouse.press_right = MOUSE_PRESSED_OFFSET; // Mouse right button

    RC_Data.key_board.key_code = KEY_OFFSET; // Key pressed

}

void DT7_Reset() {
//    memset(&huart3, 0, sizeof(UART_HandleTypeDef));
//    HAL_UART_MspDeInit(&huart3);
//    MX_DMA_Init();
//
//    MX_USART3_UART_Init();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3RecBuffer, UART3_MAX_RECEIVE_BUFFER_LENGTH);
}

void DT7_Decode() {
    RC_Data.last_update_time = xTaskGetTickCount();

    RC_Data.RCFrameCounter++;

    RC_Data.Switch_Right = (RC_rx_buffer[5] >> 4 & 0x03);
    RC_Data.Switch_Left = (RC_rx_buffer[5] >> 6 & 0x03);
    RC_Data.ch0 = ((RC_rx_buffer[0] | (RC_rx_buffer[1] << 8)) &
                   0x07ff);                                           // Channel 0
    RC_Data.ch1 = (((RC_rx_buffer[1] >> 3) | (RC_rx_buffer[2] << 5)) &
                   0x07ff);                                   // Channel 1
    RC_Data.ch2 = (((RC_rx_buffer[2] >> 6) | (RC_rx_buffer[3] << 2) |
                    (RC_rx_buffer[4] << 10)) & 0x07ff); // Channel 2
    RC_Data.ch3 = (((RC_rx_buffer[4] >> 1) | (RC_rx_buffer[5] << 7)) &
                   0x07ff);                                   // Channel 3
    RC_Data.ch4_wheel = ((RC_rx_buffer[16] | (RC_rx_buffer[17] << 8)) & 0x07ff);
    RC_Data.mouse.x = RC_rx_buffer[6] | (RC_rx_buffer[7] << 8);    // Mouse X axis
    RC_Data.mouse.y = RC_rx_buffer[8] | (RC_rx_buffer[9] << 8);    // Mouse Y axis
    RC_Data.mouse.z = RC_rx_buffer[10] | (RC_rx_buffer[11] << 8); // Mouse Z axis

    RC_Data.mouse.press_left = RC_rx_buffer[12];                                       // Mouse Left Pressed
    RC_Data.mouse.press_right = RC_rx_buffer[13];                                   // Mouse Right Pressed
    RC_Data.key_board.key_code = RC_rx_buffer[14] | (RC_rx_buffer[15] << 8); // KeyBoard value
}
