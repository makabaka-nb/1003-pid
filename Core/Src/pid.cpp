//
// Created by ROG STRIX on 2025/10/18.
//

#include "../Inc/pid.h"
void PID::reset() {
    ref_ = fdb_ = err_ = err_sum_ = last_err_ = pout_ = iout_ = last_dout_ = 0.0f;
}

float PID::calc(float ref, float fdb) {
    ref_ = ref;
    fdb_ = fdb;
    err_ = ref_ - fdb_;
    pout_ = err_ * kp_;
    err_sum_ += err_;
    iout_ = ki_ * err_sum_;
    if (iout_ > i_max_)
        iout_ = i_max_;
    if (iout_ < -i_max_)
        iout_ = -i_max_;
    float derivative = err_ - last_err_;
    float filtered_err;
    filtered_err = d_filter_k_ * last_dout_ + (1 - d_filter_k_) * derivative;
    last_err_ = err_;
    output_ = pout_ + iout_ + kd_ * filtered_err;
    if (output_ > out_max_)
        output_ = out_max_;
    if (output_ < -out_max_)
        output_ = -out_max_;
    return output_;
}
