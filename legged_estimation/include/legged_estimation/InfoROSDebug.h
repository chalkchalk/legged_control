#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
class InfoToROS
{
public:
    InfoToROS(){};
    void Init(ros::NodeHandle &nh, int size_of_value)
    {
        nh_ = nh;
        pub_debug_values_ = nh_.advertise<std_msgs::Float64MultiArray>("debug_info", 1000);
        debug_value.resize(size_of_value);
    }
    void publish_debug_value(void)
    {
        std_msgs::Float64MultiArray debug_msg;
        debug_msg.data = debug_value;
        pub_debug_values_.publish(debug_msg);
    }
    std::vector<double> debug_value;

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_debug_values_;
};