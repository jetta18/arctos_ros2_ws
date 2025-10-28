#include "arctos_hardware_interface/arctos_mks_hardware_interface.hpp"

namespace arctos_hardware_interface {

arctos_motor_driver::MotorConfig ArctosMKSHardwareInterface::load_joint_config(const std::string& joint_name) const {
  arctos_motor_driver::MotorConfig motor_config;
  std::string param_prefix = "motors." + joint_name + ".";
  motor_config.motor_id = std::stoi(getHardwareParameter(param_prefix + "motor_id", "0"));
  motor_config.gear_ratio = std::stod(getHardwareParameter(param_prefix + "gear_ratio", "1.0"));
  motor_config.inverted = (getHardwareParameter(param_prefix + "inverted", "false") == "true");
  motor_config.hardware_type = getHardwareParameter(param_prefix + "hardware_type", "MKS_42D");
  motor_config.working_current = std::stoi(getHardwareParameter(param_prefix + "working_current", "1000"));
  motor_config.holding_current = std::stoi(getHardwareParameter(param_prefix + "holding_current", "70"));
  motor_config.limit_remap_enabled = (getHardwareParameter(param_prefix + "limit_remap_enabled", "false") == "true");
  motor_config.work_mode = static_cast<uint8_t>(std::stoi(getHardwareParameter(param_prefix + "work_mode", "5")));
  return motor_config;
}

bool ArctosMKSHardwareInterface::load_all_joint_configs() {
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Configuring %zu motors...", joint_names_.size());
  for (const std::string& joint_name : joint_names_) {
    auto motor_config = load_joint_config(joint_name);
    if (!motor_manager_->addMotor(joint_name, motor_config)) {
      RCLCPP_ERROR(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Failed to add motor for joint: %s", joint_name.c_str());
      return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"),
                "Configured '%s': motor_id=%u, gear_ratio=%.2f, inverted=%s, type=%s, current=%u mA, mode=%u",
                joint_name.c_str(), motor_config.motor_id, motor_config.gear_ratio,
                motor_config.inverted ? "YES" : "NO", motor_config.hardware_type.c_str(),
                motor_config.working_current, motor_config.work_mode);
  }
  return true;
}

} // namespace arctos_hardware_interface
