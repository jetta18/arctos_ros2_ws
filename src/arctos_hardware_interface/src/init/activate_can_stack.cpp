#include "arctos_hardware_interface/arctos_mks_hardware_interface.hpp"
#include "arctos_hardware_interface/constants.hpp"
#include <thread>

namespace arctos_hardware_interface {

bool ArctosMKSHardwareInterface::init_can_stack() {
  if (motor_driver_) return true;
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Creating CAN node and motor driver...");
  can_node_ = rclcpp::Node::make_shared("arctos_can_node");
  motor_driver_ = std::make_shared<arctos_motor_driver::MKSMotorDriver>(can_node_, can_interface_name_);
  motor_manager_ = std::make_unique<arctos_motor_driver::MKSMotorManager>(motor_driver_);
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Motor driver created on CAN interface: %s", can_interface_name_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Stabilizing CAN transport... waiting %dms", constants::kCanInitDelayMs);
  std::this_thread::sleep_for(std::chrono::milliseconds(constants::kCanInitDelayMs));
  return true;
}

} // namespace arctos_hardware_interface
