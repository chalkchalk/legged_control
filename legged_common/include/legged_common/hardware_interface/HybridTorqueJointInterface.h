//
// Created by qiayuan on 2021/11/5.
//
#pragma once
#include <legged_common/hardware_interface/HybridJointInterface.h>
#include <legged_common/hardware_interface/PDHybridMotorControl.h>

namespace legged {
class HybridTorqueJointHandle : public HybridJointHandle {
 public:
  HybridTorqueJointHandle() = default;

  HybridTorqueJointHandle(const JointStateHandle& js, double* posDes, double* velDes, double* kp, double* kd, double* ff/*, double* q_min, double* q_max, double* tau_max, double* p_max, double* d_max*/)
      : HybridJointHandle(js, posDes, velDes, kp, kd, ff), // q_min_(q_min), q_max_(q_max), tau_max_(tau_max), p_max_(p_max), d_max_(d_max), 
      pd_motor_(js.getName(), 0, 0, 0, 0, 0, 0, 0){
  }

  void setCommand(double pos_des, double vel_des, double kp, double kd, double ff, double q_min, double q_max, double tau_max, double p_max, double d_max) {
    if(q_min == 0 && q_max == 0)
    {
      setPositionDesired(pos_des);
      setVelocityDesired(vel_des);
      setKp(kp);
      setKd(kd);
      setFeedforward(ff);
    }
    else
    {
      setPositionDesired(pos_des);
      setVelocityDesired(vel_des);
      setKp(0.0);
      setKd(0.0);
      pd_motor_.kp_ = kp;
      pd_motor_.kd_ = kd;
      pd_motor_.q_max_ = q_max;
      pd_motor_.q_min_ = q_min;
      pd_motor_.tau_max_ = tau_max;
      pd_motor_.p_max_ = p_max;
      pd_motor_.d_max_ = d_max;
      setFeedforward(pd_motor_.get_torque(getPosition(), getVelocity(), pos_des, vel_des) + ff);
    }
    
  }

  // void setPositionMin(double cmd) {
  //   assert(q_min_);
  //   *q_min_ = cmd;
  // }

  // void setPositionMax(double cmd) {
  //   assert(q_max_);
  //   *q_max_ = cmd;
  // }

  // void setTauMax(double cmd) {
  //   assert(tau_max_);
  //   *tau_max_ = cmd;
  // }

  // void setPOutMax(double cmd) {
  //   assert(p_max_);
  //   *p_max_ = cmd;
  // }

  // void setDOutMax(double cmd) {
  //   assert(d_max_);
  //   *d_max_ = cmd;
  // }

 private:
  // double* q_min_ = {nullptr};
  // double* q_max_ = {nullptr};
  // double* tau_max_ = {nullptr};
  // double* p_max_ = {nullptr};
  // double* d_max_ = {nullptr};
  PDHybridMotorControl pd_motor_;
};

class HybridTorqueJointInterface : public hardware_interface::HardwareResourceManager<HybridTorqueJointHandle, hardware_interface::ClaimResources> {};

}  // namespace legged
