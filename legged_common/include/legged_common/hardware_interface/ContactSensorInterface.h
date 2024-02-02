//
// Created by qiayuan on 2021/11/5.
//
#pragma once
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace legged {
class ContactSensorHandle {
 public:
  ContactSensorHandle() = default;

  ContactSensorHandle(const std::string& name, const double* contact_force) : name_(name), contact_force_(contact_force) {
    if (contact_force == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. contact_force pointer is null.");
    }
  }

  std::string getName() const { return name_; }

  double get_contact_forces() const {
    assert(contact_force_);
    return *contact_force_;
  }

 private:
  std::string name_;

  const double* contact_force_ = {nullptr};
};

class ContactSensorInterface
    : public hardware_interface::HardwareResourceManager<ContactSensorHandle, hardware_interface::DontClaimResources> {};

}  // namespace legged
