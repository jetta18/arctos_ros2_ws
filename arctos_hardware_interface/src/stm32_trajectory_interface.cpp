#include "arctos_hardware_interface/stm32_trajectory_interface.hpp"

#include <chrono>
#include <cstring>

namespace arctos_hardware_interface
{

STM32TrajectoryInterface::STM32TrajectoryInterface()
: reconnect_enabled_(true),
  shutdown_requested_(false),
  hw_system_state_(0.0),
  hw_interface_ptr_value_(0.0),
  logger_(rclcpp::get_logger("STM32TrajectoryInterface"))
{
}

STM32TrajectoryInterface::~STM32TrajectoryInterface()
{
  perform_lifecycle_cleanup();
}

hardware_interface::CallbackReturn STM32TrajectoryInterface::on_init(
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

  const size_t num_joints = info_.joints.size();
  hw_states_positions_.resize(num_joints, 0.0);
  hw_states_velocities_.resize(num_joints, 0.0);
  gear_ratios_.resize(num_joints, 1.0);
  steps_per_revolution_.resize(num_joints, 0.0);
  joint_inversions_.resize(num_joints, false);

  for (size_t i = 0; i < num_joints; ++i)
  {
    const auto & joint_name = info_.joints[i].name;

    std::string gear_param = "motors." + joint_name + ".gear_ratio";
    if (info_.hardware_parameters.count(gear_param))
    {
      gear_ratios_[i] = std::stod(info_.hardware_parameters.at(gear_param));
    }
    else
    {
      RCLCPP_WARN(logger_, "No gear ratio for %s, using 1.0", joint_name.c_str());
    }

    steps_per_revolution_[i] = STEPS_PER_REV * MICROSTEPS * gear_ratios_[i];

    std::string inv_param = "motors." + joint_name + ".inverted";
    if (info_.hardware_parameters.count(inv_param))
    {
      joint_inversions_[i] = (info_.hardware_parameters.at(inv_param) == "true");
    }

    RCLCPP_INFO(logger_, "Joint %s: gear_ratio=%.2f, steps_per_rev=%.2f, inverted=%s",
                joint_name.c_str(), gear_ratios_[i], steps_per_revolution_[i],
                joint_inversions_[i] ? "true" : "false");
  }

  unit_converter_ = std::make_unique<utils::UnitConverter>(
    steps_per_revolution_, joint_inversions_);
  socket_manager_ = std::make_unique<utils::STM32SocketManager>(
    stm32_host, stm32_port, logger_);

  RCLCPP_INFO(logger_, "Initialized with %zu joints", num_joints);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32TrajectoryInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Configuring STM32 Trajectory Interface...");

  if (!socket_manager_->connect())
  {
    RCLCPP_ERROR(logger_, "Failed to connect to STM32");
    return hardware_interface::CallbackReturn::ERROR;
  }

  protocol_ = std::make_unique<utils::STM32ProtocolV2>(
    socket_manager_->get_socket_fd(), logger_);

  if (!protocol_->ping())
  {
    RCLCPP_ERROR(logger_, "STM32 did not respond to v2 PING");
    socket_manager_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Successfully configured (protocol v2)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32TrajectoryInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up...");
  perform_lifecycle_cleanup();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32TrajectoryInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Shutting down...");
  perform_lifecycle_cleanup();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32TrajectoryInterface::on_error(
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
STM32TrajectoryInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    interfaces.emplace_back(
      info_.joints[i].name, "position", &hw_states_positions_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, "velocity", &hw_states_velocities_[i]);
  }

  interfaces.emplace_back(
    "trajectory_hw", "system_state", &hw_system_state_);

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
STM32TrajectoryInterface::export_command_interfaces()
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

  return interfaces;
}

hardware_interface::CallbackReturn STM32TrajectoryInterface::on_activate(
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
         i < utils::ProtocolV2Constants::MAX_JOINTS; ++i)
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

hardware_interface::CallbackReturn STM32TrajectoryInterface::on_deactivate(
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

hardware_interface::return_type STM32TrajectoryInterface::read(
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
      protocol_ = std::make_unique<utils::STM32ProtocolV2>(
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
       i < utils::ProtocolV2Constants::MAX_JOINTS; ++i)
  {
    hw_states_positions_[i] = unit_converter_->steps_to_rad(state.positions[i], i);
    hw_states_velocities_[i] = unit_converter_->steps_per_sec_to_rad_per_sec(
      state.velocities[i], i);
  }

  hw_system_state_ = static_cast<double>(state.system_state);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type STM32TrajectoryInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  /* Trajectories are sent out-of-band by the controller via the protocol pointer.
   * write() is intentionally a no-op. */
  return hardware_interface::return_type::OK;
}

void STM32TrajectoryInterface::perform_lifecycle_cleanup()
{
  shutdown_requested_.store(true);
  reconnect_enabled_ = false;

  if (socket_manager_)
  {
    socket_manager_->disconnect();
  }

  protocol_.reset();
}

void STM32TrajectoryInterface::log_throttled(
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
  arctos_hardware_interface::STM32TrajectoryInterface,
  hardware_interface::SystemInterface)
