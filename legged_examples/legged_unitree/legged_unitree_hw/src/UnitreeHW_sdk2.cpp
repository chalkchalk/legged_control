
#include "legged_unitree_hw/UnitreeHW_sdk2.h"
#include <sensor_msgs/Joy.h>

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define NETWORK_INTERFACE "eno1" //"eno1" "eth0"

namespace legged {
bool UnitreeHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!LeggedHW::init(root_nh, robot_hw_nh)) {
    return false;
  }
  std::string robot_type;
  root_nh.getParam("robot_type", robot_type);
  if (robot_type != "go2") {
    ROS_FATAL("Invalid robot type: %s", robot_type.c_str());
    return false;
  }
  ChannelFactory::Instance()->Init(0, NETWORK_INTERFACE);
  InitLowCmd();
  setupJoints();
  setupImu();
  setupContactSensor(robot_hw_nh);
  lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
  lowcmd_publisher->InitChannel();

  lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
  lowstate_subscriber->InitChannel(std::bind(&UnitreeHW::LowStateMessageHandler, this, std::placeholders::_1), 1);
  joyPublisher_ = root_nh.advertise<sensor_msgs::Joy>("/joy_unitree", 10);

  return true;
}

void UnitreeHW::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
}

void UnitreeHW::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for(int i=0; i<20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}

void UnitreeHW::read(const ros::Time& time, const ros::Duration& /*period*/) {
  for (int i = 0; i < 12; ++i) {
    jointData_[i].pos_ = low_state.motor_state()[i].q();
    jointData_[i].vel_ = low_state.motor_state()[i].dq();
    jointData_[i].tau_ = low_state.motor_state()[i].tau_est();
  }

  imuData_.ori_[0] = low_state.imu_state().quaternion()[1];
  imuData_.ori_[1] = low_state.imu_state().quaternion()[2];
  imuData_.ori_[2] = low_state.imu_state().quaternion()[3];
  imuData_.ori_[3] = low_state.imu_state().quaternion()[0];
  imuData_.angularVel_[0] = low_state.imu_state().gyroscope()[0];
  imuData_.angularVel_[1] = low_state.imu_state().gyroscope()[1];
  imuData_.angularVel_[2] = low_state.imu_state().gyroscope()[2];
  imuData_.linearAcc_[0] = low_state.imu_state().accelerometer()[0];
  imuData_.linearAcc_[1] = low_state.imu_state().accelerometer()[1];
  imuData_.linearAcc_[2] = low_state.imu_state().accelerometer()[2];

  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contact_force_[i] = low_state.foot_force()[i] + contact_offset_[i];
    // std::cout << i <<"," << low_state.foot_force()[i] <<std::endl;
  }


  // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
  std::vector<std::string> names = hybridTorqueJointInterface_.getNames();
  for (const auto& name : names) {
    HybridTorqueJointHandle handle = hybridTorqueJointInterface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setKd(0.0);
    handle.setVelocityDesired(0.);
    handle.setKd(3.);
  }

  updateJoystick(time);
}

uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void UnitreeHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) 
{
  for (int i = 0; i < 12; ++i) {
    low_cmd.motor_cmd()[i].q() = static_cast<float>(jointData_[i].posDes_);
    low_cmd.motor_cmd()[i].dq() = static_cast<float>(jointData_[i].velDes_);
    low_cmd.motor_cmd()[i].kp() = static_cast<float>(jointData_[i].kp_);
    low_cmd.motor_cmd()[i].kd() = static_cast<float>(jointData_[i].kd_);
    low_cmd.motor_cmd()[i].tau() = static_cast<float>(jointData_[i].ff_);
  }
  low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
  lowcmd_publisher->Write(low_cmd);
}

bool UnitreeHW::setupJoints() 
{
  for (const auto& joint : urdfModel_->joints_) {
    int leg_index = 0;
    int joint_index = 0;
    if (joint.first.find("RF") != std::string::npos) {
      leg_index = 0;
    } else if (joint.first.find("LF") != std::string::npos) {
      leg_index = 1;
    } else if (joint.first.find("RH") != std::string::npos) {
      leg_index = 2;
    } else if (joint.first.find("LH") != std::string::npos) {
      leg_index = 3;
    } else {
      continue;
    }

    if (joint.first.find("HAA") != std::string::npos) {
      joint_index = 0;
    } else if (joint.first.find("HFE") != std::string::npos) {
      joint_index = 1;
    } else if (joint.first.find("KFE") != std::string::npos) {
      joint_index = 2;
    } else {
      continue;
    }

    int index = leg_index * 3 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridTorqueJointInterface_.registerHandle(HybridTorqueJointHandle(state_handle, &jointData_[index].posDes_, &jointData_[index].velDes_,
                                                           &jointData_[index].kp_, &jointData_[index].kd_, &jointData_[index].ff_));
  }
  return true;
}

bool UnitreeHW::setupImu() 
{
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("unitree_imu", "unitree_imu", imuData_.ori_, imuData_.oriCov_,
                                                                         imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_,
                                                                         imuData_.linearAccCov_));
  imuData_.oriCov_[0] = 0.0012;
  imuData_.oriCov_[4] = 0.0012;
  imuData_.oriCov_[8] = 0.0012;

  imuData_.angularVelCov_[0] = 0.0004;
  imuData_.angularVelCov_[4] = 0.0004;
  imuData_.angularVelCov_[8] = 0.0004;
  return true;
}

bool UnitreeHW::setupContactSensor(ros::NodeHandle& nh) 
{
  nh.getParam("contact_offset_1", contact_offset_[0]);
  nh.getParam("contact_offset_2", contact_offset_[1]);
  nh.getParam("contact_offset_3", contact_offset_[2]);
  nh.getParam("contact_offset_4", contact_offset_[3]);
  
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactSensorInterface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contact_force_[i]));
  }
  return true;
}

void UnitreeHW::updateJoystick(const ros::Time& time) {
if ((time - lastPub_).toSec() < 1 / 50.) {
    return;
  }
  lastPub_ = time;
  xRockerBtnDataStruct keyData;
  memcpy(&keyData, &low_state.wireless_remote()[0], 40);
  sensor_msgs::Joy joyMsg;  // Pack as same as Logitech F710
  joyMsg.axes.push_back(-keyData.lx);
  joyMsg.axes.push_back(keyData.ly);
  joyMsg.axes.push_back(-keyData.rx);
  joyMsg.axes.push_back(keyData.ry);
  joyMsg.buttons.push_back(keyData.btn.components.X);
  joyMsg.buttons.push_back(keyData.btn.components.A);
  joyMsg.buttons.push_back(keyData.btn.components.B);
  joyMsg.buttons.push_back(keyData.btn.components.Y);
  joyMsg.buttons.push_back(keyData.btn.components.L1);
  joyMsg.buttons.push_back(keyData.btn.components.R1);
  joyMsg.buttons.push_back(keyData.btn.components.L2);
  joyMsg.buttons.push_back(keyData.btn.components.R2);
  joyMsg.buttons.push_back(keyData.btn.components.select);
  joyMsg.buttons.push_back(keyData.btn.components.start);
  joyPublisher_.publish(joyMsg);
}

}  // namespace legged