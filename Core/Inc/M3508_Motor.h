//
// Created by ROG STRIX on 2025/10/3.
//

#ifndef CAN_TEST_M3508_MOTOR_H
#define CAN_TEST_M3508_MOTOR_H
#include <stdint.h>


class M3508_Motor {
private:
    const float ratio_;//电机减速比

    float angle_;//deg输出端累计转动角度
    float delta_angle_;//deg输出端新转动角度
    float ecd_angle_;//deg当前电机编码器角度
    float last_ecd_angle_;//deg上次电机编码器角度
    float delta_ecd_angle_;//deg编码器端新转动角度
    float rotate_speed_;//dps反馈转子转速

    float current_;//A 反馈转矩电流
    float temp_;//°C 反馈电机温度

public:
    explicit M3508_Motor(const float ratio):
            ratio_(ratio),angle_(0.0f),delta_angle_(0.0f),ecd_angle_(0.0f),
            last_ecd_angle_(0.0f),delta_ecd_angle_(0.0f),rotate_speed_(0.0f),
            current_(0.0f),temp_(0.0f){};
    void canRxMsgCallback(const uint8_t rx_data[8]);


};


#endif //CAN_TEST_M3508_MOTOR_H

