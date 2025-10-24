//
// Created by ROG STRIX on 2025/10/3.
//
#include "M3508_Motor.h"
#include "can.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
M3508_Motor Motor(19.2f, POSITION_SPEED);

extern uint8_t stop_flag;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim6) {
        Motor.handle();
    }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    static uint8_t rx_data[8];
    CAN_RxHeaderTypeDef rx_header;
    if (hcan == &hcan1) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
            if (rx_header.StdId == 0x201) {
                Motor.canRxMsgCallback(rx_data);
            }
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_2) {
        if (stop_flag == 1)
            stop_flag = 0;
        else if (stop_flag == 0)
            stop_flag = 1;
    }
}
