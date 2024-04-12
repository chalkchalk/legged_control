
#include "legged_unitree_hw/UnitreeHW_sdk2.h"


#include <sensor_msgs/Joy.h>

namespace legged {
bool UnitreeHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  
  return true;
}

void UnitreeHW::read(const ros::Time& time, const ros::Duration& /*period*/) {

}

void UnitreeHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) 
{

}

bool UnitreeHW::setupJoints() 
{

  return true;
}

bool UnitreeHW::setupImu() 
{

  return true;
}

bool UnitreeHW::setupContactSensor(ros::NodeHandle& nh) 
{

  return true;
}

void UnitreeHW::updateJoystick(const ros::Time& time) {


}  // namespace legged
}