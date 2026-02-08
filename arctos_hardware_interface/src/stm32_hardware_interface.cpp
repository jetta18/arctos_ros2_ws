#include "arctos_hardware_interface/stm32_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>

namespace arctos_hardware_interface
{

STM32HardwareInterface::STM32HardwareInterface()
: reconnect_enabled_(true),
  shutdown_requested_(false),
  hw_system_state_(0.0),
  hw_interface_ptr_value_(0.0),
  gripper_enabled_(false),
  hw_gripper_position_left_(0.0),
  hw_gripper_position_right_(0.0),
  hw_gripper_command_(0.0),
  hw_gripper_command_prev_(std::numeric_limits<double>::quiet_NaN()),
  servo_closed_pulse_us_(500),
  servo_open_pulse_us_(2500),
  gripper_max_opening_m_(0.015),
  servo_speed_m_per_s_(0.03),
  logger_(rclcpp::get_logger("STM32HardwareInterface"))
{
}

STM32HardwareInterface::~STM32HardwareInterface()
{
  perform_lifecycle_cleanup();
}

hardware_interface::CallbackReturn STM32HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string stm32_host = info_.hardware_parameters["stm32_host"];
  int stm32_port = std::stoi(info_.hardware_parameters["stm32_port"]);

  RCLCPP_INFO(logger_, "STM32 Host: %s, Port: %d", stm32_host.c_str(), stm32_port);

  /* Count only motor-driven arm joints (skip gripper jaw joints) */
  size_t arm_joint_count = 0;
  for (const auto & joint : info_.joints)
  {
    if (joint.name != "Left_jaw_joint" && joint.name != "Right_jaw_joint")
    {
      ++arm_joint_count;
    }
  }

  hw_states_positions_.resize(arm_joint_count, 0.0);
  hw_states_velocities_.resize(arm_joint_count, 0.0);
  gear_ratios_.resize(arm_joint_count, 1.0);
  steps_per_revolution_.resize(arm_joint_count, 0.0);
  joint_inversions_.resize(arm_joint_count, false);

  size_t motor_idx = 0;
  for (const auto & joint : info_.joints)
  {
    if (joint.name == "Left_jaw_joint" || joint.name == "Right_jaw_joint")
    {
      continue;
    }

    std::string gear_param = "motors." + joint.name + ".gear_ratio";
    if (info_.hardware_parameters.count(gear_param))
    {
      gear_ratios_[motor_idx] = std::stod(info_.hardware_parameters.at(gear_param));
    }
    else
    {
      RCLCPP_WARN(logger_, "No gear ratio for %s, using 1.0", joint.name.c_str());
    }

    steps_per_revolution_[motor_idx] = STEPS_PER_REV * MICROSTEPS * gear_ratios_[motor_idx];

    std::string inv_param = "motors." + joint.name + ".inverted";
    if (info_.hardware_parameters.count(inv_param))
    {
      joint_inversions_[motor_idx] = (info_.hardware_parameters.at(inv_param) == "true");
    }

    RCLCPP_INFO(logger_, "Joint %s: gear_ratio=%.2f, steps_per_rev=%.2f, inverted=%s",
                joint.name.c_str(), gear_ratios_[motor_idx], steps_per_revolution_[motor_idx],
                joint_inversions_[motor_idx] ? "true" : "false");
    ++motor_idx;
  }

  unit_converter_ = std::make_unique<utils::UnitConverter>(
    steps_per_revolution_, joint_inversions_);
  socket_manager_ = std::make_unique<utils::STM32SocketManager>(
    stm32_host, stm32_port, logger_);

  /* Detect gripper joints (optional — missing joints are silently skipped) */
  gripper_enabled_ = false;
  for (const auto & joint : info_.joints)
  {
    if (joint.name == "Left_jaw_joint" || joint.name == "Right_jaw_joint")
    {
      gripper_enabled_ = true;
      break;
    }
  }

  if (gripper_enabled_)
  {
    if (info_.hardware_parameters.count("servo.closed_pulse_us"))
    {
      servo_closed_pulse_us_ = static_cast<uint16_t>(
        std::stoi(info_.hardware_parameters.at("servo.closed_pulse_us")));
    }
    if (info_.hardware_parameters.count("servo.open_pulse_us"))
    {
      servo_open_pulse_us_ = static_cast<uint16_t>(
        std::stoi(info_.hardware_parameters.at("servo.open_pulse_us")));
    }
    if (info_.hardware_parameters.count("servo.max_opening_m"))
    {
      gripper_max_opening_m_ = std::stod(
        info_.hardware_parameters.at("servo.max_opening_m"));
    }
    if (info_.hardware_parameters.count("servo.speed_m_per_s"))
    {
      servo_speed_m_per_s_ = std::stod(
        info_.hardware_parameters.at("servo.speed_m_per_s"));
    }
    RCLCPP_INFO(logger_, "Gripper enabled: closed=%u us, open=%u us, max_opening=%.4f m",
                servo_closed_pulse_us_, servo_open_pulse_us_, gripper_max_opening_m_);
  }

  RCLCPP_INFO(logger_, "Initialized with %zu arm joints (gripper %s)",
              arm_joint_count, gripper_enabled_ ? "enabled" : "disabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32HardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Configuring STM32 Hardware Interface...");

  if (!socket_manager_->connect())
  {
    RCLCPP_ERROR(logger_, "Failed to connect to STM32");
    return hardware_interface::CallbackReturn::ERROR;
  }

  protocol_ = std::make_unique<utils::STM32Protocol>(
    socket_manager_->get_socket_fd(), logger_);

  if (!protocol_->ping())
  {
    RCLCPP_ERROR(logger_, "STM32 did not respond to PING");
    socket_manager_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Successfully configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32HardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up...");
  perform_lifecycle_cleanup();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32HardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Shutting down...");
  perform_lifecycle_cleanup();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32HardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(logger_, "Error state, stopping and disconnecting...");
  if (protocol_)
  {
    protocol_->stop();
  }
  perform_lifecycle_cleanup();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
STM32HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  size_t motor_idx = 0;
  for (const auto & joint : info_.joints)
  {
    if (joint.name == "Left_jaw_joint" || joint.name == "Right_jaw_joint")
    {
      continue;
    }
    interfaces.emplace_back(
      joint.name, "position", &hw_states_positions_[motor_idx]);
    interfaces.emplace_back(
      joint.name, "velocity", &hw_states_velocities_[motor_idx]);
    ++motor_idx;
  }

  interfaces.emplace_back(
    "trajectory_hw", "system_state", &hw_system_state_);

  if (gripper_enabled_)
  {
    interfaces.emplace_back(
      "Left_jaw_joint", "position", &hw_gripper_position_left_);
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
STM32HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  /* Expose a pointer to this HW interface so the controller can call
   * lock_protocol() / get_protocol() / get_unit_converter() directly.
   * The pointer is stored as a double via memcpy (safe on x86_64). */
  auto * self = this;
  static_assert(sizeof(self) <= sizeof(double), "pointer must fit in double");
  hw_interface_ptr_value_ = 0.0;
  std::memcpy(&hw_interface_ptr_value_, &self, sizeof(self));

  interfaces.emplace_back(
    "trajectory_hw", "hw_interface_ptr", &hw_interface_ptr_value_);

  if (gripper_enabled_)
  {
    interfaces.emplace_back(
      "Left_jaw_joint", "position", &hw_gripper_command_);
  }

  return interfaces;
}

hardware_interface::CallbackReturn STM32HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating...");

  shutdown_requested_.store(false);
  reconnect_enabled_ = true;

  /* Read initial state */
  utils::StateResponse state;
  if (protocol_ && protocol_->read_state(state))
  {
    for (size_t i = 0; i < hw_states_positions_.size() &&
         i < utils::ProtocolConstants::MAX_JOINTS; ++i)
    {
      hw_states_positions_[i] = unit_converter_->steps_to_rad(state.positions[i], i);
      hw_states_velocities_[i] = unit_converter_->steps_per_sec_to_rad_per_sec(
        state.velocities[i], i);
    }
  }
  else
  {
    RCLCPP_WARN(logger_, "Could not read initial state, using zeros");
  }

  RCLCPP_INFO(logger_, "Successfully activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating...");

  shutdown_requested_.store(true);
  reconnect_enabled_ = false;

  if (protocol_)
  {
    protocol_->stop();
  }

  if (socket_manager_)
  {
    socket_manager_->disconnect();
  }

  RCLCPP_INFO(logger_, "Successfully deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type STM32HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  /* Skip GET_STATE if the controller currently holds the protocol lock
   * (e.g. during trajectory upload). Use try_lock to avoid blocking. */
  std::unique_lock<std::mutex> lock(protocol_mutex_, std::try_to_lock);
  if (!lock.owns_lock())
  {
    return hardware_interface::return_type::OK;
  }

  if (!socket_manager_->is_connected() && reconnect_enabled_ && !shutdown_requested_.load())
  {
    socket_manager_->attempt_reconnection(reconnect_enabled_, shutdown_requested_);
    if (socket_manager_->is_connected())
    {
      protocol_ = std::make_unique<utils::STM32Protocol>(
        socket_manager_->get_socket_fd(), logger_);
    }
  }

  if (!protocol_)
  {
    static auto last_warn = std::chrono::steady_clock::now();
    log_throttled("Protocol not initialized", last_warn);
    return hardware_interface::return_type::ERROR;
  }

  utils::StateResponse state;
  if (!protocol_->read_state(state))
  {
    static auto last_warn = std::chrono::steady_clock::now();
    log_throttled("Failed to read state from STM32", last_warn);
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < hw_states_positions_.size() &&
       i < utils::ProtocolConstants::MAX_JOINTS; ++i)
  {
    hw_states_positions_[i] = unit_converter_->steps_to_rad(state.positions[i], i);
    hw_states_velocities_[i] = unit_converter_->steps_per_sec_to_rad_per_sec(
      state.velocities[i], i);
  }

  hw_system_state_ = static_cast<double>(state.system_state);

  if (gripper_enabled_)
  {
    double position_m = 0.0;
    uint16_t pulse = state.servo_pulse_us;
    if (pulse >= servo_closed_pulse_us_ && pulse <= servo_open_pulse_us_)
    {
      double fraction = static_cast<double>(pulse - servo_closed_pulse_us_) /
                        static_cast<double>(servo_open_pulse_us_ - servo_closed_pulse_us_);
      position_m = fraction * gripper_max_opening_m_;
    }
    hw_gripper_position_left_ = position_m;
    hw_gripper_position_right_ = position_m;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type STM32HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  /* Trajectories are sent out-of-band by the controller via the protocol pointer. */

  if (gripper_enabled_ && protocol_)
  {
    bool command_changed = std::isnan(hw_gripper_command_prev_) ||
                           std::abs(hw_gripper_command_ - hw_gripper_command_prev_) > 1e-6;
    if (command_changed)
    {
      double clamped = std::clamp(hw_gripper_command_, 0.0, gripper_max_opening_m_);
      double fraction = clamped / gripper_max_opening_m_;
      auto pulse = static_cast<uint16_t>(
        servo_closed_pulse_us_ +
        fraction * (servo_open_pulse_us_ - servo_closed_pulse_us_));

      double distance_m = std::abs(clamped - hw_gripper_position_left_);
      auto duration_ms = static_cast<uint16_t>(
        std::clamp(distance_m / servo_speed_m_per_s_ * 1000.0, 0.0, 5000.0));

      std::unique_lock<std::mutex> lock(protocol_mutex_, std::try_to_lock);
      if (lock.owns_lock())
      {
        protocol_->set_servo(pulse, duration_ms);
        hw_gripper_command_prev_ = hw_gripper_command_;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

void STM32HardwareInterface::perform_lifecycle_cleanup()
{
  shutdown_requested_.store(true);
  reconnect_enabled_ = false;

  if (socket_manager_)
  {
    socket_manager_->disconnect();
  }

  protocol_.reset();
}

void STM32HardwareInterface::log_throttled(
  const std::string & message,
  std::chrono::steady_clock::time_point & last_log_time)
{
  auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() >
      LOG_THROTTLE_MS)
  {
    RCLCPP_WARN(logger_, "%s", message.c_str());
    last_log_time = now;
  }
}

}  // namespace arctos_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arctos_hardware_interface::STM32HardwareInterface,
  hardware_interface::SystemInterface)
