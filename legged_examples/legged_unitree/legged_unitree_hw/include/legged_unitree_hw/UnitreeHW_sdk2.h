#pragma once

#include <legged_hw/LeggedHW.h>
#include <unitree_legged_sdk2/robot/channel/channel_publisher.hpp>
#include <unitree_legged_sdk2/robot/channel/channel_subscriber.hpp>
#include <unitree_legged_sdk2/idl/go2/LowState_.hpp>
#include <unitree_legged_sdk2/idl/go2/LowCmd_.hpp>
#include <unitree_legged_sdk2/robot/go2/robot_state/robot_state_client.hpp>
#include <unitree_legged_sdk2/common/thread/thread.hpp>
#include <unitree_legged_sdk2/common/gamepad.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;

namespace legged {
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

struct UnitreeMotorData {
  double pos_, vel_, tau_;                 // state
  double posDes_, velDes_, kp_, kd_, ff_;  // command
  double q_max_, q_min_, tau_max_, p_max_, d_max_; // software constraint
};

struct UnitreeImuData {
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

class UnitreeHW : public LeggedHW {
 public:
  UnitreeHW() = default;
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /** \brief Communicate with hardware. Get data, status of robot.
   *
   * Call @ref UNITREE_LEGGED_SDK::UDP::Recv() to get robot's state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref UNITREE_LEGGED_SDK::UDP::Recv(). Publish actuator
   * current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

  void updateJoystick(const ros::Time& time);

 private:
  void InitLowCmd();
  bool setupJoints();

  void LowStateMessageHandler(const void* message);

  bool setupImu();

  bool setupContactSensor(ros::NodeHandle& nh);

  ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
  /*subscriber*/
  ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

  UnitreeMotorData jointData_[12]{};  // NOLINT(modernize-avoid-c-arrays)
  UnitreeImuData imuData_{};
  unitree_go::msg::dds_::LowCmd_ low_cmd{};      // default init
  unitree_go::msg::dds_::LowState_ low_state{};  // default init
  double contact_force_[4]{};  // NOLINT(modernize-avoid-c-arrays)
  /*LowCmd write thread*/
  ThreadPtr lowCmdWriteThreadPtr;

  int contact_offset_[4]{};
  ros::Publisher joyPublisher_;
  ros::Time lastPub_;
};

}  // namespace legged
