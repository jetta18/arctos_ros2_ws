#include "arctos_hardware_interface/arctos_mks_hardware_interface.hpp"
#include "arctos_hardware_interface/constants.hpp"
#include <thread>
#include <cmath>

namespace arctos_hardware_interface {

bool ArctosMKSHardwareInterface::read_initial_encoders_with_settle(int settle_ms, int timeout_ms) {
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Reading initial encoder values...");
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Delaying initial read by %dms", settle_ms);
  std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));
  if (!readInitialEncoderValues(timeout_ms)) {
    RCLCPP_ERROR(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Failed to read initial encoder values");
    return false;
  }
  return true;
}

void ArctosMKSHardwareInterface::initialize_hw_positions_from_manager() {
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    hw_positions_[i] = motor_manager_->getJointPosition(joint_names_[i]);
    hw_commands_positions_[i] = hw_positions_[i];
    hw_commands_velocities_[i] = 0.0;
    RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"),
                "Initial position '%s': %.4f rad (%.2f°)",
                joint_names_[i].c_str(), hw_positions_[i], hw_positions_[i] * 180.0 / M_PI);
  }
}

bool ArctosMKSHardwareInterface::enable_all_motors() {
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Enabling motors...");
  for (const std::string& joint_name : joint_names_) {
    uint32_t motor_id = motor_manager_->getMotorId(joint_name);
    if (!motor_driver_->enableMotor(motor_id, true)) {
      RCLCPP_ERROR(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Failed to enable motor %u (%s)", motor_id, joint_name.c_str());
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(constants::kBetweenEnableMs));
  }
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "✓ All motors configured, initialized and enabled");
  return true;
}

} // namespace arctos_hardware_interface
