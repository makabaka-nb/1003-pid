//
// Created by ROG STRIX on 2025/10/3.
//

#ifndef CAN_TEST_M3508_MOTOR_H
#define CAN_TEST_M3508_MOTOR_H
#include "pid.h"
#include <stdint.h>

class M3508_Motor {
private:
    const float ratio_; //电机减速比

    float angle_; //deg输出端累计转动角度
    float delta_angle_; //deg输出端新转动角度
    float ecd_angle_; //deg当前电机编码器角度
    float last_ecd_angle_; //deg上次电机编码器角度
    float delta_ecd_angle_; //deg编码器端新转动角度
    float rotate_speed_; //dps反馈转子转速

    float current_; //A 反馈转矩电流
    float temp_; //°C 反馈电机温度

public:
    PID spid_, ppid_;
    float target_angle_, fdb_angle_;
    float target_speed_, fdb_speed_, feedforward_speed_;
    float feedforward_intensity_, output_intensity_;
    enum {
        TORQUE,
        SPEED,
        POSITION_SPEED,
    } control_method_;
    explicit M3508_Motor(
        const float ratio,
        float spid_kp = 0,
        float spid_ki = 0,
        float spid_kd = 0,
        float spid_i_max = 0,
        float spid_out_max = 0,
        float spid_d_filter_k = 0,
        float ppid_kp = 0,
        float ppid_ki = 0,
        float ppid_kd = 0,
        float ppid_i_max = 0,
        float ppid_out_max = 0,
        float ppid_d_filter_k = 0
    );

    void canRxMsgCallback(const uint8_t rx_data[8]);
    void SetPosition(float target_position, float feedforward_speed, float feedforward_intensity);
    void SetSpeed(float target_speed, float feedforward_intensity);
    void SetIntensity(float intensity);
    void handle(void);
};

#endif //CAN_TEST_M3508_MOTOR_H
