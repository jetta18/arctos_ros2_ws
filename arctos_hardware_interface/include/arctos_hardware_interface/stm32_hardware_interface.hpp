#ifndef ARCTOS_HARDWARE_INTERFACE__STM32_HARDWARE_INTERFACE_HPP_
#define ARCTOS_HARDWARE_INTERFACE__STM32_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "arctos_msgs/msg/arctos_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace arctos_hardware_interface
{

/**
 * Hardware interface for the Arctos robot arm.
 *
 * Thin adapter that reads state from the arctos_bridge node's
 * /arctos/state topic and forwards position commands via
 * /arctos/cmd_positions and /arctos/cmd_servo topics.
 *
 * All direct STM32 communication is handled by the arctos_bridge node.
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

private:
  void start_ros_interface();
  void stop_ros_interface();
  void state_callback(const arctos_msgs::msg::ArctosState::SharedPtr msg);

  /* Internal ROS node for topic communication with the bridge */
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;

  /* Bridge topic interfaces */
  rclcpp::Subscription<arctos_msgs::msg::ArctosState>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_servo_pub_;

  /* State interfaces (read by controller_manager) */
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  double hw_system_state_;

  /* State from bridge (written by callback, read by read()) */
  std::mutex state_mutex_;
  std::vector<double> bridge_positions_;
  std::vector<double> bridge_velocities_;
  double bridge_system_state_;
  double bridge_servo_pulse_us_;
  bool state_received_;

  /* Arm position command interfaces (written by JointGroupPositionController) */
  std::vector<double> hw_cmd_positions_;
  std::vector<double> hw_cmd_positions_prev_;
  static constexpr double POSITION_CMD_DEADBAND_RAD = 1e-5;

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

  rclcpp::Logger logger_;
};

}  // namespace arctos_hardware_interface

#endif  // ARCTOS_HARDWARE_INTERFACE__STM32_HARDWARE_INTERFACE_HPP_
