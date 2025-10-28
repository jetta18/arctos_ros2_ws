#include "arctos_hardware_interface/arctos_mks_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arctos_hardware_interface
{

bool ArctosMKSHardwareInterface::extract_joint_names()
{
  joint_names_.clear();
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
  }
  return true;
}

void ArctosMKSHardwareInterface::init_state_vectors()
{
  const size_t num_joints = joint_names_.size();
  hw_positions_.assign(num_joints, 0.0);
  hw_velocities_.assign(num_joints, 0.0);
  hw_commands_positions_.assign(num_joints, 0.0);
  hw_commands_velocities_.assign(num_joints, 0.0);
}

void ArctosMKSHardwareInterface::log_joint_names() const
{
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Joint %zu: %s", i, joint_names_[i].c_str());
  }
}

}  // namespace arctos_hardware_interface
