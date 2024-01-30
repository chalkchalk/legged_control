// created by chalkchalk on 2024.01.30

#pragma once

#include <ros/ros.h>
#include "doggy_msgs/DoggyMove.h"
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>
#include <vector>

using namespace ocs2;

class CmdToOCS2
{
public:
    CmdToOCS2(ros::NodeHandle& nh);

private:
    ros::NodeHandle& nh_;
    doggy_msgs::DoggyMove doggy_cmd_;
    ros::Subscriber doggy_cmd_sub_;
    ros::Subscriber observation_sub_;
    void publish_ocs2_cmd(void);
    void publish_ocs2_mode(void);
    void publish_ocs2_trajectory(void);
    void doggy_cmd_callback(const doggy_msgs::DoggyMove &msg);
    void observation_callback(const ocs2_msgs::mpc_observation::ConstPtr& msg);
    TargetTrajectories targetPoseToTargetTrajectories(const vector_t& targetPose, const SystemObservation& observation, const scalar_t& targetReachingTime);
    void get_trajectory();
    std::vector<std::string> gaitList_;
    std::map<std::string, legged_robot::ModeSequenceTemplate> gaitMap_;
    ros::Publisher modeSequenceTemplatePublisher_;
    std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;
    SystemObservation observation_;
    std::mutex latestObservationMutex_;
    TargetTrajectories trajectory_;
    scalar_t time_step;
    vector_t defautl_joint_state;
    std::string gaitCommand_;
    bool obs_received_;

};
