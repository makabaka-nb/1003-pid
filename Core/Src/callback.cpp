//
// Created by ROG STRIX on 2025/10/3.
//
#include "M3508_Motor.h"
#include "can.h"
#include "main.h"
#include "tim.h"

extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t tx_data[8];
extern uint8_t rx_data[8];
M3508_Motor Motor(19.2f);
extern uint32_t can_tx_mail_box;

void HAL_CAN_RxFifo0MsgPendingCallback(const CAN_HandleTypeDef* hcan) {
    if (hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
        if (rx_header.StdId == 0x201) {
            Motor.canRxMsgCallback(rx_data);
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(const TIM_HandleTypeDef* htim) {
    if (htim->Instance == htim6.Instance) {
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mail_box);
    }
}
