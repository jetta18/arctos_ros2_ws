#ifndef ARCTOS_HARDWARE_INTERFACE__STM32_HARDWARE_INTERFACE_HPP_
#define ARCTOS_HARDWARE_INTERFACE__STM32_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

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
 * Hardware interface for the Arctos STM32 firmware.
 *
 * Owns the single UDP connection to the STM32. The controller obtains
 * a pointer to this interface (via a custom command interface) and calls
 * trajectory methods through lock_protocol() / unlock_protocol().
 *
 * read() polls GET_STATE unless the protocol is locked by the controller.
 * write() is a no-op.
 */
class STM32HardwareInterface : public hardware_interface::SystemInterface
{
public:
  STM32HardwareInterface();
  ~STM32HardwareInterface() override;

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

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * Lock the protocol for exclusive use by the controller.
   * While locked, read() skips GET_STATE to avoid cross-talk.
   */
  void lock_protocol() { protocol_mutex_.lock(); }
  void unlock_protocol() { protocol_mutex_.unlock(); }

  utils::STM32Protocol * get_protocol() { return protocol_.get(); }
  utils::UnitConverter * get_unit_converter() { return unit_converter_.get(); }

private:
  void perform_lifecycle_cleanup();
  void log_throttled(const std::string & message,
                     std::chrono::steady_clock::time_point & last_log_time);

  std::unique_ptr<utils::UnitConverter> unit_converter_;
  std::unique_ptr<utils::STM32Protocol> protocol_;
  std::unique_ptr<utils::STM32SocketManager> socket_manager_;
  std::mutex protocol_mutex_;

  bool reconnect_enabled_;
  std::atomic_bool shutdown_requested_;

  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  double hw_system_state_;

  /* Stores a pointer to `this` so the controller can retrieve it. */
  double hw_interface_ptr_value_;

  std::vector<double> gear_ratios_;
  std::vector<double> steps_per_revolution_;
  std::vector<bool> joint_inversions_;
  static constexpr int STEPS_PER_REV = 200;
  static constexpr int MICROSTEPS = 16;

  /* Gripper (servo-driven, optional) */
  bool gripper_enabled_;
  double hw_gripper_position_left_;
  double hw_gripper_position_right_;
  double hw_gripper_command_;
  double hw_gripper_command_prev_;
  uint16_t servo_closed_pulse_us_;
  uint16_t servo_open_pulse_us_;
  double gripper_max_opening_m_;
  double servo_speed_m_per_s_;

  static constexpr int LOG_THROTTLE_MS = 1000;

  rclcpp::Logger logger_;
};

}  // namespace arctos_hardware_interface

#endif  // ARCTOS_HARDWARE_INTERFACE__STM32_HARDWARE_INTERFACE_HPP_
