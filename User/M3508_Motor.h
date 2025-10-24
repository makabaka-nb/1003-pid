//
// Created by xuhao on 2025/10/18.
//

#ifndef PID_MOTOR_H
#define PID_MOTOR_H
#include "pid.h"
#include "stm32f4xx_hal.h"

enum ControlMethod {
    TORQUE,
    SPEED,
    POSITION_SPEED,
};

class M3508_Motor {
public:
    M3508_Motor(float ratio, ControlMethod control_method);
    void SetPosition(float target_position, float feedforward_speed, float feedforward_intensity);
    void SetSpeed(float target_speed, float feedforward_intensity);
    void SetIntensity(float torque);
    void handle();

    void canRxMsgCallback(const uint8_t rx_data[8]);

private:
    const float ratio_;
    float angle_ = 0.0f; // 累计角度（考虑减速比）
    float delta_angle_ = 0.0f; // 角度变化量
    float ecd_angle_ = 0.0f; // 原始电机轴编码器角度（0-360度）
    float last_ecd_angle_ = 0.0f; // 上一次电机轴编码器角度

    float rotate_speed_ = 0.0f; // 转速
    float current_ = 0.0f; // 电流
    float temp_ = 0.0f; // 温度

    float MAX_CURRENT = 15.0f; // 最大电流限制
    float MAX_SPEED = 1000.f; // 最大转速限制
    bool can_init_flag_ = true; // 初始化标志
    // CAN发送接收数据
    static uint8_t tx_data_[8];
    // PID
    PID spid_, ppid_;
    float target_angle_ = 0.0f, fdb_angle_ = 0.0f;
    float target_speed_ = 0.0f, fdb_speed_ = 0.0f, feedforward_speed_ = 0.0f;
    float output_intensity_ = 0.0f, feedforward_intensity_ = 0.0f;
    ControlMethod control_method_;
    ControlMethod prev_method_ = TORQUE;
    static float linearMapping(int in, int in_min, int in_max, float out_min, float out_max);
    void sendcurrent(float current);
    float FeedforwardIntensityCalc(float current_angle);
};

#endif //PID_MOTOR_H