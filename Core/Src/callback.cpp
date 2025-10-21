//
// Created by ROG STRIX on 2025/10/3.
//
#include "M3508_Motor.h"
#include "can.h"
#include "main.h"
#include "tim.h"

#include <cmath>

extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t tx_data[8];
extern uint8_t rx_data[8];
M3508_Motor Motor(19.2f);
extern uint32_t can_tx_mail_box;
extern uint8_t stop_flag;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_2) {
        if (stop_flag == 1)
            stop_flag = 0;
        else if (stop_flag == 0)
            stop_flag = 1;
    }
}
void HAL_CAN_RxFifo0MsgPendingCallback(const CAN_HandleTypeDef* hcan) {
    if (hcan->Instance == CAN1) {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
        if (rx_header.StdId == 0x201) {
            Motor.canRxMsgCallback(rx_data);
            float angle_back = Motor.fdb_angle_;
            float intensity_back = Motor.FeedforwardIntensityCalc(angle_back);
            int16_t intensity_s = static_cast<int16_t>(roundf(intensity_back));
            if (stop_flag == 0)
                intensity_s = 0;
            tx_data[0] = static_cast<uint8_t>((intensity_s >> 8) & 0xFF);
            tx_data[1] = static_cast<uint8_t>(intensity_s & 0xFF);
            tx_data[2] = 0;
            tx_data[3] = 0;
            tx_data[4] = 0;
            tx_data[5] = 0;
            tx_data[6] = 0;
            tx_data[7] = 0;
            HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mail_box);
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(const TIM_HandleTypeDef* htim) {
    if (htim->Instance == htim6.Instance) {
        if (stop_flag == 1) {
            for (uint8_t i = 0; i < 8; i++) {
                tx_data[i] = 0;
            }
        }
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mail_box);
    }
}
