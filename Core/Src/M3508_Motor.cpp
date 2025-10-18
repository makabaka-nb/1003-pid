//
// Created by ROG STRIX on 2025/10/3.
//

#include "M3508_Motor.h"

#include <tgmath.h>

float linearMapping(int in ,int in_min,int in_max,float out_min,float out_max) {
    float y;
    y=out_min+(out_max-out_min)/static_cast<float>(in_max-in_min)*static_cast<float>(in-in_min);
    return y;
}

void M3508_Motor::canRxMsgCallback(const uint8_t rx_data[8]) {
    int ecd_raw=(rx_data[0]<<8)|rx_data[1];
    ecd_angle_=linearMapping(ecd_raw,0,8191,0.0f,360.0f);
    delta_ecd_angle_=fabs(ecd_angle_-last_ecd_angle_);
    if (delta_ecd_angle_>180.0&&delta_ecd_angle_<360.0) {
        delta_ecd_angle_=360.0f-delta_ecd_angle_;
    }
    delta_angle_=delta_ecd_angle_/ratio_;
    angle_+=delta_angle_;
    rotate_speed_=static_cast<float>((rx_data[2]<<8)|rx_data[3]);
    int current=((rx_data[4]<<8)|rx_data[5]);
    current_=linearMapping(current,-16384,16384,-20.0f,20.0f);
    temp_=static_cast<float>(rx_data[6]);
    last_ecd_angle_=ecd_angle_;
}
