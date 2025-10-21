//
// Created by ROG STRIX on 2025/10/18.
//

#ifndef CAN_TEST_PID_H
#define CAN_TEST_PID_H

class PID {
public:
    PID(float kp = 0, float ki = 0, float kd = 0, float i_max = 0, float out_max = 0, float d_filter_k = 1):
        kp_(kp),
        ki_(ki),
        kd_(kd),
        i_max_(i_max),
        out_max_(out_max),
        d_filter_k_(d_filter_k) {}
    void reset(void);
    float calc(float ref, float fdb);

    float kp_, ki_, kd_, d_filter_k_;
    float i_max_, out_max_;
    float output_;

private:
    float ref_, fdb_;
    float err_, err_sum_, last_err_;
    float pout_, iout_, dout_, last_dout_;
};

#endif //CAN_TEST_PID_H
