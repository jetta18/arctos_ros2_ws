#ifndef ARCTOS_HARDWARE_INTERFACE__ARCTOS_MKS_HARDWARE_INTERFACE_HPP_
#define ARCTOS_HARDWARE_INTERFACE__ARCTOS_MKS_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arctos_motor_driver/mks_motor_driver.hpp"

namespace arctos_hardware_interface
{

class ArctosMKSHardwareInterface : public hardware_interface::SystemInterface
{
public:
  ArctosMKSHardwareInterface();

  // Lifecycle methods
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  // Hardware interface methods
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Motor driver instance
  std::unique_ptr<arctos_motor_driver::MKSMotorDriver> motor_driver_;

  // Joint state storage
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;

  // Joint names
  std::vector<std::string> joint_names_;
  
  // CAN interface name (stored for delayed motor driver creation)
  std::string can_interface_name_;
  
  // Helper function to get hardware parameters
  std::string getHardwareParameter(const std::string& param_name, const std::string& default_value) const;
};

}  // namespace arctos_hardware_interface

#endif  // ARCTOS_HARDWARE_INTERFACE__ARCTOS_MKS_HARDWARE_INTERFACE_HPP_
