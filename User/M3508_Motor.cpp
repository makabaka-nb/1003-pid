//
// Created by ROG STRIX on 2025/10/3.
//

#include "M3508_Motor.h"
#include "can.h"
#include "stm32f4xx_hal.h"
#include <cmath>
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t stop_flag;
uint8_t M3508_Motor::tx_data_[8] = { 0 };
//构造函数
M3508_Motor::M3508_Motor(float ratio, ControlMethod control_method): ratio_(ratio), control_method_(control_method) {
    spid_ = PID(0.008f, 0.0f, 0.0056f, MAX_SPEED, MAX_CURRENT, 0.05f); // 内环 速度环PID
    ppid_ = PID(200.0f, 0.015f, 220.0f, 100.0f, MAX_SPEED, 0.06f); // 外环 位置环PID
}
//线性映射函数
float M3508_Motor::linearMapping(int in, int in_min, int in_max, float out_min, float out_max) {
    float y;
    y = static_cast<float>(in - in_min) * (out_max - out_min) / static_cast<float>(in_max - in_min) + out_min;
    return y;
}
//电机解包
void M3508_Motor::canRxMsgCallback(const uint8_t rx_data[8]) {
    int16_t ecd_raw = (rx_data[0] << 8) | rx_data[1];
    last_ecd_angle_ = ecd_angle_;
    ecd_angle_ = linearMapping(ecd_raw, 0, 8191, 0.0f, 360.0f);
    if (can_init_flag_) {
        last_ecd_angle_ = ecd_angle_;
        can_init_flag_ = false;
    }
    float delta_ecd_angle = ecd_angle_ - last_ecd_angle_;
    if (delta_ecd_angle > 180.0f) {
        delta_ecd_angle -= 360.0f;
    } else if (delta_ecd_angle < -180.0f) {
        delta_ecd_angle += 360.0f;
    }
    delta_angle_ = delta_ecd_angle / ratio_;
    angle_ += delta_angle_;
    rotate_speed_ = static_cast<float>((rx_data[2] << 8) | rx_data[3]);
    int16_t current = ((rx_data[4] << 8) | rx_data[5]);
    current_ = linearMapping(current, -16384, 16384, -20.0f, 20.0f);
    temp_ = static_cast<float>(rx_data[6]);
}
//向电调发送数据
void M3508_Motor::sendcurrent(float current) {
    if (current > MAX_CURRENT) {
        current = MAX_CURRENT;
    } else if (current < -MAX_CURRENT) {
        current = -MAX_CURRENT;
    }
    int16_t current_send = static_cast<int16_t>(current * 16384.0f / 20.0f);
    tx_data_[0] = current_send >> 8;
    tx_data_[1] = current_send & 0xff;
    tx_data_[2] = 0;
    tx_data_[3] = 0;
    tx_data_[4] = 0;
    tx_data_[5] = 0;
    tx_data_[6] = 0;
    tx_data_[7] = 0;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data_, nullptr);
    //不关心发送邮箱的编号
}

void M3508_Motor::SetPosition(float target_position, float feedforward_speed, float feedforward_intensity) {
    control_method_ = POSITION_SPEED;
    this->target_angle_ = target_position;
    this->feedforward_speed_ = feedforward_speed;
    this->feedforward_intensity_ = feedforward_intensity;
}

void M3508_Motor::SetSpeed(float target_speed, float feedforward_intensity) {
    control_method_ = SPEED;
    this->target_speed_ = target_speed;
    this->feedforward_intensity_ = feedforward_intensity;
}

void M3508_Motor::SetIntensity(float intensity) {
    control_method_ = TORQUE;
    this->output_intensity_ = intensity;
}

void M3508_Motor::handle() {
    float output = 0.0f;
    if (prev_method_ != control_method_) {
        spid_.reset();
        ppid_.reset();
        prev_method_ = control_method_;
    }
    if (control_method_ == TORQUE) {
        //不用pid
    } else if (control_method_ == SPEED) {
        fdb_speed_ = rotate_speed_;
        output_intensity_ = spid_.calc(target_speed_, fdb_speed_);
    } else if (control_method_ == POSITION_SPEED) {
        //位置->速度，速度->力矩
        fdb_angle_ = angle_;
        fdb_speed_ = rotate_speed_;
        target_speed_ = ppid_.calc(target_angle_, fdb_angle_);
        target_speed_ += feedforward_speed_;
        output_intensity_ = spid_.calc(target_speed_, fdb_speed_);
    }
    feedforward_intensity_ = FeedforwardIntensityCalc(angle_);
    output = output_intensity_ + feedforward_intensity_;
    if (stop_flag == 1)
        output = 0.0f;
    sendcurrent(output / 0.3f);
}

float M3508_Motor::FeedforwardIntensityCalc(float current_angle) {
    const float m = 500.0f;
    const float r = 55.24f;
    const float g = 9.81f;
    float angle_rad = current_angle * 3.1415926535f / 180.0f;
    return m * g * r * std::sin(angle_rad) / 1e6; // N·m
    //输出的是力矩，要注意单位是否统一，到底是力矩，电流，还是电流映射到电调的值
}
