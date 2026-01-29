#ifndef ARCTOS_HARDWARE_INTERFACE__STM32_STEPPER_INTERFACE_HPP_
#define ARCTOS_HARDWARE_INTERFACE__STM32_STEPPER_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

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
  // UDP communication
  bool connect_to_stm32();
  void disconnect_from_stm32();
  bool send_jtc_command(const std::vector<double>& positions, 
                        const std::vector<double>& velocities);
  bool read_state();
  void attempt_reconnection();
  bool ping_stm32();

  // Unit conversion
  double rad_to_steps(double radians, size_t joint_index) const;
  double steps_to_rad(double steps, size_t joint_index) const;
  double rad_per_sec_to_steps_per_sec(double rad_per_sec, size_t joint_index) const;
  double steps_per_sec_to_rad_per_sec(double steps_per_sec, size_t joint_index) const;

  // Configuration
  std::string stm32_host_;
  int stm32_port_;
  int socket_fd_;
  bool connected_;
  std::mutex connection_mutex_;
  
  // Reconnection control
  bool reconnect_enabled_;
  std::chrono::steady_clock::time_point last_reconnect_attempt_;
  static constexpr std::chrono::milliseconds RECONNECT_INTERVAL{5000};  // 5 seconds

  // Joint data (in radians and rad/s)
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;

  // Motor configuration
  std::vector<double> gear_ratios_;
  std::vector<double> steps_per_revolution_;  // Per joint (after gear ratio)
  std::vector<bool> joint_inversions_;         // Per joint inversion flags
  static constexpr int STEPS_PER_REV = 200;
  static constexpr int MICROSTEPS = 16;

  // Protocol constants
  static constexpr uint8_t PROTOCOL_VERSION = 1;
  static constexpr uint8_t CMD_JTC_STREAM = 0x01;
  static constexpr uint8_t CMD_PING = 0x20;
  static constexpr uint8_t CMD_GET_STATE = 0x10;
  static constexpr uint8_t RESP_OK = 0x00;
  static constexpr uint8_t RESP_STATE = 0x02;

  // Handshake configuration
  static constexpr int PING_MAX_ATTEMPTS = 5;
  static constexpr std::chrono::milliseconds PING_RETRY_DELAY{200};

  // Sequence tracking
  uint32_t sequence_number_;

  // Logger
  rclcpp::Logger logger_;
};

}  // namespace arctos_hardware_interface

#endif  // ARCTOS_HARDWARE_INTERFACE__STM32_STEPPER_INTERFACE_HPP_
