#include "cmd_to_ocs2.h"
#include <ocs2_core/misc/LoadData.h>
#include "ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>



bool is_zero_vel(const doggy_msgs::DoggyMove &msg)
{
    return msg.yaw_rate == 0 && msg.vel_x == 0 && msg.vel_y == 0;
}

scalar_t CmdToOCS2::estimateTimeToTarget(const vector_t& desiredBaseDisplacement) {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / target_rotation_vel;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / target_lin_vel;
  return std::max(rotationTime, displacementTime);
}

TargetTrajectories CmdToOCS2::targetPoseToTargetTrajectories(const vector_t& targetPose, const SystemObservation& observation, 
const scalar_t& targetReachingTime) 
{
  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  vector_t currentPose = observation.state.segment<6>(6);
  currentPose(2) = doggy_cmd_.body_height;
  currentPose(4) = 0;
  currentPose(5) = 0;
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, defautl_joint_state;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, defautl_joint_state;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

CmdToOCS2::CmdToOCS2(ros::NodeHandle &nh)
    : nh_(nh),obs_received_(false)
{
    targetTrajectoriesPublisher_.reset(new TargetTrajectoriesRosPublisher(nh_, "legged_robot"));
    std::string gaitCommandFile, taskFile, referenceFile;
    // gaitCommandFile = "/root/catkin_ws/src/legged_control/legged_controllers/config/a1/gait.info";
    nh_.getParam("/gaitCommandFile", gaitCommandFile);
    nh_.getParam("/taskFile", taskFile);
    nh_.getParam("/referenceFile", referenceFile);
    defautl_joint_state = vector_t::Zero(12);
    loadData::loadEigenMatrix(referenceFile, "defaultJointState", defautl_joint_state);
    loadData::loadCppDataType(taskFile, "mpc.timeHorizon", time_step);
    loadData::loadStdVector(gaitCommandFile, "list", gaitList_, false);
    loadData::loadCppDataType(referenceFile, "targetRotationVelocity", target_rotation_vel);
    loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", target_lin_vel);
    gaitMap_.clear();
    for (const auto &gaitName : gaitList_)
    {
        gaitMap_.insert({gaitName, legged_robot::loadModeSequenceTemplate(gaitCommandFile, gaitName, false)});
    }
    target_pose = vector_t::Zero(6);
    observation_sub_ = nh.subscribe<ocs2_msgs::mpc_observation>("legged_robot_mpc_observation", 1, &CmdToOCS2::observation_callback, this);
    doggy_cmd_sub_ = nh_.subscribe("/doggy_cmd", 1, &CmdToOCS2::doggy_cmd_callback, this);
    modeSequenceTemplatePublisher_ = nh_.advertise<ocs2_msgs::mode_schedule>("legged_robot_mpc_mode_schedule", 1, true);
}

void CmdToOCS2::publish_ocs2_cmd(void)
{
    publish_ocs2_mode();
    publish_ocs2_trajectory();
}
void CmdToOCS2::publish_ocs2_mode(void)
{
    std::string gaitCommand;
    if (doggy_cmd_.gait_code == doggy_msgs::DoggyMove::GAIT_STANCE)
    {
        gaitCommand = gaitList_[0];
    }
    else if (doggy_cmd_.gait_code == doggy_msgs::DoggyMove::GAIT_TYPE1)
    {
        gaitCommand = gaitList_[1];
    }
    else if (doggy_cmd_.gait_code == doggy_msgs::DoggyMove::GAIT_TYPE2)
    {
        gaitCommand = gaitList_[2];
    }
    else if (doggy_cmd_.gait_code == doggy_msgs::DoggyMove::GAIT_TYPE3)
    {
        gaitCommand = gaitList_[3];
    }
    if(gaitCommand_!= gaitCommand)
    {
        gaitCommand_ = gaitCommand;
        legged_robot::ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(gaitCommand);
        modeSequenceTemplatePublisher_.publish(legged_robot::createModeSequenceTemplateMsg(modeSequenceTemplate));
    }
    
}

void CmdToOCS2::observation_callback(const ocs2_msgs::mpc_observation::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    observation_= ros_msg_conversions::readObservationMsg(*msg);
    if(!obs_received_)
    {
        target_pose_last_static_ = observation_.state.segment<6>(6);
        // target_pose_last_static_(2) = doggy_cmd_.body_height;
        target_pose_last_static_(4) = 0;
        target_pose_last_static_(5) = 0;
        obs_received_ = true;
    }
    
}
void CmdToOCS2::get_trajectory(void)
{
    std::unique_lock<std::mutex> lock(latestObservationMutex_);
    SystemObservation observation = observation_;
    lock.unlock();
    vector_t cmdVel = vector_t::Zero(3);
    cmdVel[0] = doggy_cmd_.vel_x;
    cmdVel[1] = doggy_cmd_.vel_y;
    vector_t currentPose = observation.state.segment<6>(6);
    
    if(is_zero_vel(doggy_cmd_) && (!is_zero_vel(doggy_cmd_last_)) && doggy_cmd_.gait_code != doggy_msgs::DoggyMove::GAIT_STANCE) // turn zero
    {
        target_pose_last_static_ = observation.state.segment<6>(6);
        // target_pose_last_static_(2) = doggy_cmd_.body_height;
        target_pose_last_static_(4) = 0;
        target_pose_last_static_(5) = 0;
    }

    target_pose.setZero();
    Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
    vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel;
    scalar_t timeToTarget = time_step * 4.0; // x y z yaw pitch roll
    target_pose(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;
    target_pose(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;
    target_pose(2) = doggy_cmd_.body_height;
    target_pose(3) = currentPose(3) + doggy_cmd_.yaw_rate * timeToTarget;
    target_pose(4) = 0;
    target_pose(5) = 0;

    if(is_zero_vel(doggy_cmd_) && doggy_cmd_.gait_code != doggy_msgs::DoggyMove::GAIT_STANCE) 
    {
        target_pose_last_static_(2) = doggy_cmd_.body_height;
        trajectory_ = targetPoseToTargetTrajectories(target_pose_last_static_, observation,  observation.time + estimateTimeToTarget(target_pose_last_static_ - currentPose));
    }
    else
    {
        trajectory_ = targetPoseToTargetTrajectories(target_pose, observation,  observation.time + timeToTarget);
    }
    
}
void CmdToOCS2::publish_ocs2_trajectory(void)
{
    get_trajectory();
    targetTrajectoriesPublisher_->publishTargetTrajectories(trajectory_);
    
}


void CmdToOCS2::doggy_cmd_callback(const doggy_msgs::DoggyMove &msg)
{
    doggy_cmd_ = msg;
    if(obs_received_)
    {
        publish_ocs2_cmd();
        doggy_cmd_last_ = doggy_cmd_;
    }
    
}

int main(int argc, char **argv)
{

    // Initialize ros node
    ::ros::init(argc, argv, "cmd_to_ocs2");
    ::ros::NodeHandle nodeHandle;
    CmdToOCS2 cmd_to_ocs2(nodeHandle);

    ros::spin();
    // Successful exit
    return 0;
}
