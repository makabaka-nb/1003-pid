//
// Created by ROG STRIX on 2025/10/3.
//

#include "M3508_Motor.h"

#include <tgmath.h>
M3508_Motor::M3508_Motor(
    const float ratio,
    float spid_kp,
    float spid_ki,
    float spid_kd,
    float spid_i_max,
    float spid_out_max,
    float spid_d_filter_k,
    float ppid_kp,
    float ppid_ki,
    float ppid_kd,
    float ppid_i_max,
    float ppid_out_max,
    float ppid_d_filter_k
):
    ratio_(ratio),
    angle_(0.0f),
    delta_angle_(0.0f),
    ecd_angle_(0.0f),
    last_ecd_angle_(0.0f),
    delta_ecd_angle_(0.0f),
    rotate_speed_(0.0f),
    current_(0.0f),
    temp_(0.0f),
    // 初始化PID控制器
    spid_(spid_kp, spid_ki, spid_kd, spid_i_max, spid_out_max, spid_d_filter_k), // 速度环PID
    ppid_(ppid_kp, ppid_ki, ppid_kd, ppid_i_max, ppid_out_max, ppid_d_filter_k), // 位置环PID
    // 初始化控制变量
    target_angle_(0.0f),
    fdb_angle_(0.0f),
    target_speed_(0.0f),
    fdb_speed_(0.0f),
    feedforward_speed_(0.0f),
    feedforward_intensity_(0.0f),
    output_intensity_(0.0f),
    control_method_(TORQUE) {}
float linearMapping(int in, int in_min, int in_max, float out_min, float out_max) {
    float y;
    y = out_min + (out_max - out_min) / static_cast<float>(in_max - in_min) * static_cast<float>(in - in_min);
    return y;
}

void M3508_Motor::canRxMsgCallback(const uint8_t rx_data[8]) {
    int ecd_raw = (rx_data[0] << 8) | rx_data[1];
    ecd_angle_ = linearMapping(ecd_raw, 0, 8191, 0.0f, 360.0f);
    delta_ecd_angle_ = fabs(ecd_angle_ - last_ecd_angle_);
    if (delta_ecd_angle_ > 180.0 && delta_ecd_angle_ < 360.0) {
        delta_ecd_angle_ = 360.0f - delta_ecd_angle_;
    }
    delta_angle_ = delta_ecd_angle_ / ratio_;
    angle_ += delta_angle_;
    rotate_speed_ = static_cast<float>((rx_data[2] << 8) | rx_data[3]);
    int current = ((rx_data[4] << 8) | rx_data[5]);
    current_ = linearMapping(current, -16384, 16384, -20.0f, 20.0f);
    temp_ = static_cast<float>(rx_data[6]);
    last_ecd_angle_ = ecd_angle_;
    fdb_angle_ = angle_;
    fdb_speed_ = rotate_speed_;
}

void M3508_Motor::SetPosition(float target_position, float feedforward_speed, float feedforward_intensity) {
    control_method_ = POSITION_SPEED;
    target_angle_ = target_position;
    feedforward_speed_ = feedforward_speed;
    feedforward_intensity_ = feedforward_intensity;
}

void M3508_Motor::SetSpeed(float target_speed, float feedforward_intensity) {
    control_method_ = SPEED;
    target_speed_ = target_speed;
    feedforward_intensity_ = feedforward_intensity;
}

void M3508_Motor::SetIntensity(float intensity) {
    control_method_ = TORQUE;
    output_intensity_ = intensity;
}

void M3508_Motor::handle(void) {
    switch (control_method_) {
        case TORQUE:
            break;
        case SPEED: {
            output_intensity_ = feedforward_intensity_ + spid_.calc(target_speed_, fdb_speed_);
            break;
        }
        case POSITION_SPEED: {
            float speed1 = ppid_.calc(target_angle_, fdb_angle_) + feedforward_speed_;
            output_intensity_ = spid_.calc(speed1, fdb_speed_) + feedforward_intensity_;
            break;
        }
    }
}

float M3508_Motor::FeedforwardIntensityCalc(float current_angle) {
    const float m = 0.5f;
    const float l = 0.05524f;
    const float g = 9.80f;
    const float kt = 0.3f;
    float angle_rad = current_angle * 3.1415926535f / 180.0;
    float torque = m * g * l * sinf(angle_rad);
    float current = -torque / kt;
    float intensity = current * 819.2f;
    return intensity;
}
