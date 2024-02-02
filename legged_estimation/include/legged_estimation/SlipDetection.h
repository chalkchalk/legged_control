#pragma once

// #include <ros/ros.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_legged_robot/common/Types.h>
// #include "InfoROSDebug.h"
// 

namespace legged {
using namespace ocs2;
using namespace legged_robot;
class SlipDetector
{
public:
    SlipDetector(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);
    void update();
    Eigen::Vector4i get_slip_state(){return slip_state;}
    Eigen::Vector4d get_slip_score(){return slip_score;}
    Eigen::Matrix<double, 3, 4> get_foot_slip_spot(){return foot_slip_spot;};
    void loadSettings(const std::string& taskFile, bool verbose);
    void updateContact(contact_flag_t contactFlag) { contactFlag_ = contactFlag; }
    void update_estimated_state(const vector_t rgb_state);


private:
    PinocchioInterface pinocchioInterface_;
    CentroidalModelInfo info_;
    std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
    vector_t rbd_state_estimted_;
    contact_flag_t contactFlag_{};
    Eigen::Vector4d foot_lateral_vel_norm;
    Eigen::Vector4d foot_lateral_dis_norm;
    Eigen::Vector4i slip_state;
    Eigen::Vector4i last_contact_state;
    Eigen::Vector4d slip_score;
    Eigen::Vector4d slip_score_filtered;
    Eigen::Matrix<double, 3, 4> foot_slip_spot;
    Eigen::Matrix<double, 3, 4> foot_first_contact_spot;
    double detect_kp;
    double detect_kd;
    double detect_clear_z;
    double lateral_vel_filter;
    // InfoToROS info_to_ros_;
};
}

