#pragma once
#include <string>
#include <vector>
#include <algorithm>

#define CLIP(x, min_val, max_val) std::min(std::max(x, min_val), max_val)

class PDHybridMotorControl
{
public:
    PDHybridMotorControl(std::string name, double q_min, double q_max, double kp, double kd, double tau_max, double p_max = 0.0, double d_max = 0.0)
        : q_max_(q_max), q_min_(q_min), kp_(kp), kd_(kd), tau_max_(tau_max), p_max_(p_max), d_max_(d_max){};
    double get_torque(double q_now, double q_dot_now, double q_set, double q_dot_set)
    {
        double torque = 0.0;
        double p_output = kp_ * (q_set - q_now);
        if(p_max_ > 0) p_output = CLIP(p_output, -p_max_, p_max_);
        double d_output = kd_ * (q_dot_set - q_dot_now);
        if(d_max_ > 0) d_output = CLIP(d_output, -d_max_, d_max_);
        torque = CLIP(p_output + d_output, -tau_max_, tau_max_);
        return torque;
    }
    double q_max_;
    double q_min_;
    double kp_;
    double kd_;
    double tau_max_;
    double p_max_;
    double d_max_;
};