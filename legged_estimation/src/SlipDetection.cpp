#include "legged_estimation/SlipDetection.h"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>



namespace legged
{
    SlipDetector::SlipDetector(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                               const PinocchioEndEffectorKinematics &eeKinematics)
        : pinocchioInterface_(std::move(pinocchioInterface)),
          info_(std::move(info)),
          eeKinematics_(eeKinematics.clone()),
          rbd_state_estimted_(vector_t ::Zero(2 * info_.generalizedCoordinatesNum))
    {
        foot_lateral_vel_norm.setZero();
        slip_state.setZero();
        slip_score.setZero();
        slip_score_filtered.setZero();
        foot_lateral_dis_norm.setZero();
        foot_first_contact_spot.setZero();
        last_contact_state.setZero();
        eeKinematics_->setPinocchioInterface(pinocchioInterface_);
        
        // ros::NodeHandle nh;
        // info_to_ros_.Init(nh, 10);
        
    }

    void SlipDetector::update_estimated_state(const vector_t rgb_state)
    {
        rbd_state_estimted_ = rgb_state;
    }

    void SlipDetector::update()
    {
        const auto &model = pinocchioInterface_.getModel();
        auto &data = pinocchioInterface_.getData();
        size_t actuatedDofNum = info_.actuatedDofNum;

        vector_t qPino(info_.generalizedCoordinatesNum);
        vector_t vPino(info_.generalizedCoordinatesNum);
        qPino.setZero();
        qPino.segment<3>(3) = rbd_state_estimted_.head<3>(); 
        qPino.head<3>() = rbd_state_estimted_.segment<3>(3); // set pose
        qPino.tail(actuatedDofNum) = rbd_state_estimted_.segment(6, actuatedDofNum);

        vPino.setZero();
        vPino.segment<3>(0) = rbd_state_estimted_.segment<3>(info_.generalizedCoordinatesNum + 3); // linear vel
        vPino.segment<3>(3) =
            getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(qPino.segment<3>(3),
                                                                            rbd_state_estimted_.segment<3>(info_.generalizedCoordinatesNum)); // Only set angular velocity, let linear velocity be zero
        vPino.tail(actuatedDofNum) = rbd_state_estimted_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum);
        pinocchio::forwardKinematics(model, data, qPino, vPino);
        pinocchio::updateFramePlacements(model, data);
        const auto eePos = eeKinematics_->getPosition(vector_t());
        const auto eeVel = eeKinematics_->getVelocity(vector_t(), vector_t());
        for (int i = 0; i < 4; ++i)
        {
            if (contactFlag_[i] > last_contact_state(i))
            {
                foot_first_contact_spot.block<3, 1>(0, i) = eePos[i];
            }
            foot_lateral_dis_norm(i) = contactFlag_[i] * (foot_first_contact_spot.block<3, 1>(0, i) - eePos[i]).segment<2>(0).norm();

            foot_lateral_vel_norm(i) = contactFlag_[i] * (foot_lateral_vel_norm(i) * lateral_vel_filter + eeVel[i].head<2>().norm() * (1.0 - lateral_vel_filter));
            slip_score(i) = contactFlag_[i] * (0.5 * (std::erf(detect_kd * foot_lateral_vel_norm(i) + detect_kp * foot_lateral_dis_norm(i) - 2.0) + 1.0));
            double filter_ratio = slip_score(i) > slip_score_filtered(i) ? 0.3 : 0.3;
            slip_score_filtered(i) = filter_ratio * slip_score_filtered(i) + (1.0 - filter_ratio) * slip_score(i);
            bool slip_state_temp = slip_score_filtered(i) > 0.5;
            if (slip_state_temp &&!slip_state(i)) // set this contact to be slippery
            {
                slip_state(i) = 1;
                foot_slip_spot.block<3, 1>(0, i) = eePos[i];
            }
            if (eePos[i](2) > detect_clear_z) // clear the slippery state during swing phase.
            {
                slip_state(i) = 0;
            }
            last_contact_state(i) = contactFlag_[i];
        }
        // if(contactFlag_[1]) std::cout << slip_state(1) <<", " << foot_lateral_vel_norm(1) << ", " << foot_lateral_dis_norm(1) << std::endl;
        // InfoToROS::debug_value[0] = foot_lateral_vel_norm(1) ;
        // InfoToROS::debug_value[1] = slip_score_filtered(1) ;
        // InfoToROS::debug_value[2] = slip_state(1) ;
        // InfoToROS::debug_value[3] = foot_lateral_dis_norm(1) ;
    }

    void SlipDetector::loadSettings(const std::string &taskFile, bool verbose)
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);
        std::string prefix = "anti_slip.";
        if (verbose)
        {
            std::cerr << "\n #### Kalman Filter Noise:";
            std::cerr << "\n #### =============================================================================\n";
        }

        loadData::loadPtreeValue(pt, detect_kd, prefix + "detect_kd", verbose);
        loadData::loadPtreeValue(pt, detect_kp, prefix + "detect_kp", verbose);
        loadData::loadPtreeValue(pt, lateral_vel_filter, prefix + "lateral_vel_filter", verbose);
        loadData::loadPtreeValue(pt, detect_clear_z, prefix + "detect_clear_z", verbose);
        
    }
}