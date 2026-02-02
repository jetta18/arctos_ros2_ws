#ifndef ARCTOS_HARDWARE_INTERFACE__STM32_STEPPER_INTERFACE_HPP_
#define ARCTOS_HARDWARE_INTERFACE__STM32_STEPPER_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <atomic>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arctos_hardware_interface/utils/unit_conversion.hpp"
#include "arctos_hardware_interface/utils/stm32_protocol.hpp"
#include "arctos_hardware_interface/utils/socket_manager.hpp"

namespace arctos_hardware_interface
{

/**
 * @brief Hardware interface for STM32-based stepper motor control
 * 
 * This interface communicates with an STM32 controller via UDP
 * using a minimal binary protocol for high-frequency joint trajectory streaming.
 * 
 * Features:
 * - Direct UDP communication with STM32
 * - JTC (Joint Trajectory Controller) streaming at 100-250Hz
 * - 6-axis stepper motor control
 * - Position and velocity command/state interfaces
 */
class STM32StepperInterface : public hardware_interface::SystemInterface
{
public:
  STM32StepperInterface();
  virtual ~STM32StepperInterface();

  // Lifecycle management
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Real-time control loop
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void perform_lifecycle_cleanup();
  void log_throttled(const std::string& message, 
                     std::chrono::steady_clock::time_point& last_log_time);

  // Utility components
  std::unique_ptr<utils::UnitConverter> unit_converter_;
  std::unique_ptr<utils::STM32Protocol> protocol_;
  std::unique_ptr<utils::STM32SocketManager> socket_manager_;
  
  // Reconnection control
  bool reconnect_enabled_;
  std::atomic_bool shutdown_requested_;

  // Joint data (in radians and rad/s)
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;

  // Motor configuration
  std::vector<double> gear_ratios_;
  std::vector<double> steps_per_revolution_;
  std::vector<bool> joint_inversions_;
  static constexpr int STEPS_PER_REV = 200;
  static constexpr int MICROSTEPS = 16;

  // Logging throttle
  static constexpr int LOG_THROTTLE_MS = 1000;

  // Sequence tracking
  uint32_t sequence_number_;

  // Logger
  rclcpp::Logger logger_;
};

}  // namespace arctos_hardware_interface

#endif  // ARCTOS_HARDWARE_INTERFACE__STM32_STEPPER_INTERFACE_HPP_
