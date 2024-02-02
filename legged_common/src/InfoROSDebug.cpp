#include "InfoROSDebug.h"

void InfoToROS::Init(ros::NodeHandle &nh, int size_of_value)
{
    nh_ = nh;
    pub_debug_values_ = nh_.advertise<std_msgs::Float64MultiArray>("debug_info", 1000);
    debug_value.resize(size_of_value);

}

void InfoToROS::publish_debug_value()
{
    std_msgs::Float64MultiArray debug_msg;
    debug_msg.data = debug_value;
    pub_debug_values_.publish(debug_msg);
}