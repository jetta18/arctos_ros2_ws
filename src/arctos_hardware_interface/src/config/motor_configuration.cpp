#include "arctos_hardware_interface/arctos_mks_hardware_interface.hpp"
#include "arctos_hardware_interface/constants.hpp"
#include <thread>
#include <algorithm>

namespace arctos_hardware_interface {

bool ArctosMKSHardwareInterface::apply_motor_config(const std::string& joint_name, const arctos_motor_driver::MotorConfig& config) {
  uint32_t motor_id = config.motor_id;
  if (!motor_driver_->setWorkMode(motor_id, config.work_mode)) {
    RCLCPP_ERROR(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Failed to set work mode for motor %u", motor_id);
    return false;
  }
  if (!motor_driver_->setWorkingCurrent(motor_id, config.working_current)) {
    RCLCPP_ERROR(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Failed to set working current for motor %u", motor_id);
    return false;
  }
  uint8_t holding_pct_index = 0;
  if (config.holding_current >= 10) {
    int pct = static_cast<int>(config.holding_current);
    holding_pct_index = static_cast<uint8_t>(std::max(0, std::min(8, (pct / 10) - 1)));
  }
  if (!motor_driver_->setHoldingCurrentPercentage(motor_id, holding_pct_index)) {
    RCLCPP_WARN(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Failed to set holding current for motor %u", motor_id);
  }
  if (config.limit_remap_enabled) {
    if (!motor_driver_->setLimitPortRemap(motor_id, true)) {
      RCLCPP_WARN(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Failed to enable limit remap for motor %u", motor_id);
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Applied configuration to motor %u (%s)", motor_id, joint_name.c_str());
  std::this_thread::sleep_for(std::chrono::milliseconds(constants::kBetweenMotorConfigMs));
  return true;
}

bool ArctosMKSHardwareInterface::apply_all_motor_configs() {
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Applying motor configurations via CAN...");
  for (const std::string& joint_name : joint_names_) {
    const auto* config = motor_manager_->getMotorConfig(joint_name);
    if (!config) {
      RCLCPP_ERROR(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Failed to get config for joint: %s", joint_name.c_str());
      return false;
    }
    if (!apply_motor_config(joint_name, *config)) return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Configuration phase complete. Waiting %dms before reading encoders...", constants::kAfterConfigSettleMs);
  std::this_thread::sleep_for(std::chrono::milliseconds(constants::kAfterConfigSettleMs));
  return true;
}

} // namespace arctos_hardware_interface
