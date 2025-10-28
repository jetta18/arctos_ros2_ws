#include "arctos_hardware_interface/arctos_mks_hardware_interface.hpp"
#include "arctos_hardware_interface/constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>
#include <algorithm>

namespace arctos_hardware_interface
{

bool ArctosMKSHardwareInterface::validate_joint_count() const
{
  if (joint_names_.size() != static_cast<size_t>(constants::kNumJoints))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Expected %d joints, but got %zu joints", constants::kNumJoints, joint_names_.size());
    return false;
  }
  return true;
}

bool ArctosMKSHardwareInterface::validate_expected_joints() const
{
  const std::vector<std::string> expected_joints = {
    "X_joint", "Y_joint", "Z_joint", "A_joint", "B_joint", "C_joint"
  };

  for (const std::string & expected_joint : expected_joints)
  {
    if (std::find(joint_names_.begin(), joint_names_.end(), expected_joint) == joint_names_.end())
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("ArctosMKSHardwareInterface"),
        "Missing expected joint: %s", expected_joint.c_str());
      return false;
    }
  }
  return true;
}

bool ArctosMKSHardwareInterface::validate_joint_interfaces() const
{
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    std::string state_if_list;
    for (const auto & si : joint.state_interfaces) {
      if (!state_if_list.empty()) state_if_list += ", ";
      state_if_list += si.name;
    }

    std::string cmd_if_list;
    for (const auto & ci : joint.command_interfaces) {
      if (!cmd_if_list.empty()) cmd_if_list += ", ";
      cmd_if_list += ci.name;
    }

    RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"),
                "Joint '%s' -> state: [%s], command: [%s]",
                joint.name.c_str(), state_if_list.c_str(), cmd_if_list.c_str());
  }
  return true;
}

}  // namespace arctos_hardware_interface
