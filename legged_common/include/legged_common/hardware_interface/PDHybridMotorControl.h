#pragma once
#include <string>
#include <vector>
#include <algorithm>

#define CLIP(x, min_val, max_val) std::min(std::max(x, min_val), max_val)


class PDHybridMotorControl
{
public:
    PDHybridMotorControl(){};
    PDHybridMotorControl(std::string name, double q_min, double q_max, double kp, double kd, double tau_max, double p_max = 0.0, double d_max = 0.0, double q_increment_max=0.0)
        : name_(name), q_max_(q_max), q_min_(q_min), kp_(kp), kd_(kd), tau_max_(tau_max), p_max_(p_max), d_max_(d_max), q_set_last_(1e5), q_increment_max_(q_increment_max){};
    double get_torque(double q_now, double q_dot_now, double q_set, double q_dot_set, double feed_forward)
    {
        double torque = 0.0;
        q_set = CLIP(q_set, q_min_, q_max_);
        if(q_set_last_ > 1e3)
        {
            q_set_last_ = q_now;
        }
        if(q_increment_max_ > 0) 
        {
            q_set = q_set_last_ + CLIP(q_set - q_set_last_, -q_increment_max_, q_increment_max_) ;
        }
        
        q_set_last_ = q_set;
        
        double p_output = kp_ * (q_set - q_now);
        if(p_max_ > 0) p_output = CLIP(p_output, -p_max_, p_max_);
        double d_output = kd_ * (q_dot_set - q_dot_now);
        if(d_max_ > 0) d_output = CLIP(d_output, -d_max_, d_max_);
        torque = p_output + d_output + feed_forward;
        torque = CLIP(torque, -tau_max_, tau_max_);
        return torque;
    }
    std::string name_;
    double q_max_;
    double q_min_;
    double kp_;
    double kd_;
    double tau_max_;
    double p_max_;
    double d_max_;
    double q_increment_max_;
    double q_set_last_;
};