#include "arctos_hardware_interface/stm32_stepper_interface.hpp"
#include <chrono>

namespace arctos_hardware_interface
{

STM32StepperInterface::STM32StepperInterface()
: reconnect_enabled_(true),
  shutdown_requested_(false),
  sequence_number_(0),
  logger_(rclcpp::get_logger("STM32StepperInterface"))
{
}

STM32StepperInterface::~STM32StepperInterface()
{
  perform_lifecycle_cleanup();
}

hardware_interface::CallbackReturn STM32StepperInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != 
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string stm32_host = info_.hardware_parameters["stm32_host"];
  int stm32_port = std::stoi(info_.hardware_parameters["stm32_port"]);

  RCLCPP_INFO(logger_, "STM32 Host: %s", stm32_host.c_str());
  RCLCPP_INFO(logger_, "STM32 Port: %d", stm32_port);

  const size_t num_joints = info_.joints.size();
  hw_commands_positions_.resize(num_joints, 0.0);
  hw_commands_velocities_.resize(num_joints, 0.0);
  hw_states_positions_.resize(num_joints, 0.0);
  hw_states_velocities_.resize(num_joints, 0.0);
  gear_ratios_.resize(num_joints, 1.0);
  steps_per_revolution_.resize(num_joints, 0.0);
  joint_inversions_.resize(num_joints, false);

  for (size_t i = 0; i < num_joints; ++i)
  {
    std::string param_name = "motors." + info_.joints[i].name + ".gear_ratio";
    if (info_.hardware_parameters.find(param_name) != info_.hardware_parameters.end())
    {
      gear_ratios_[i] = std::stod(info_.hardware_parameters[param_name]);
    }
    else
    {
      RCLCPP_WARN(logger_, "No gear ratio found for joint %s, using 1.0", info_.joints[i].name.c_str());
      gear_ratios_[i] = 1.0;
    }

    steps_per_revolution_[i] = STEPS_PER_REV * MICROSTEPS * gear_ratios_[i];
    std::string inversion_param_name = "motors." + info_.joints[i].name + ".inverted";
    if (info_.hardware_parameters.find(inversion_param_name) != info_.hardware_parameters.end())
    {
      std::string inversion_str = info_.hardware_parameters[inversion_param_name];
      joint_inversions_[i] = (inversion_str == "true");
    }
    else
    {
      RCLCPP_WARN(logger_, "No inversion parameter found for joint %s, using false", info_.joints[i].name.c_str());
      joint_inversions_[i] = false;
    }

    RCLCPP_INFO(logger_, "Joint %s: gear_ratio=%.2f, steps_per_rev=%.2f, inverted=%s",
                info_.joints[i].name.c_str(), gear_ratios_[i], steps_per_revolution_[i], 
                joint_inversions_[i] ? "true" : "false");
  }

  unit_converter_ = std::make_unique<utils::UnitConverter>(steps_per_revolution_, joint_inversions_);
  socket_manager_ = std::make_unique<utils::STM32SocketManager>(stm32_host, stm32_port, logger_);

  RCLCPP_INFO(logger_, "Initialized with %zu joints", num_joints);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32StepperInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Configuring STM32 Stepper Interface...");
  
  if (!socket_manager_->connect())
  {
    RCLCPP_ERROR(logger_, "Failed to connect to STM32");
    return hardware_interface::CallbackReturn::ERROR;
  }

  protocol_ = std::make_unique<utils::STM32Protocol>(
    socket_manager_->get_socket_fd(), logger_, sequence_number_);

  if (!protocol_->ping())
  {
    RCLCPP_ERROR(logger_, "STM32 did not respond to PING handshake");
    socket_manager_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Successfully configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32StepperInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up STM32 Stepper Interface...");
  perform_lifecycle_cleanup();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32StepperInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Shutting down STM32 Stepper Interface...");
  perform_lifecycle_cleanup();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32StepperInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(logger_, "Hardware interface error, stopping and disconnecting...");
  perform_lifecycle_cleanup();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> 
STM32StepperInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "position", &hw_states_positions_[i]));
    
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "velocity", &hw_states_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> 
STM32StepperInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, "position", &hw_commands_positions_[i]));
    
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, "velocity", &hw_commands_velocities_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn STM32StepperInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating STM32 Stepper Interface...");

  shutdown_requested_.store(false);
  reconnect_enabled_ = true;

  std::vector<float> positions_steps, velocities_steps;
  if (protocol_ && protocol_->read_state(positions_steps, velocities_steps))
  {
    for (size_t i = 0; i < hw_states_positions_.size() && i < positions_steps.size(); ++i)
    {
      hw_states_positions_[i] = unit_converter_->steps_to_rad(positions_steps[i], i);
      hw_states_velocities_[i] = unit_converter_->steps_per_sec_to_rad_per_sec(velocities_steps[i], i);
    }
  }
  else
  {
    RCLCPP_WARN(logger_, "Could not read initial state, using zeros");
  }

  hw_commands_positions_ = hw_states_positions_;
  hw_commands_velocities_ = hw_states_velocities_;

  RCLCPP_INFO(logger_, "Successfully activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32StepperInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating STM32 Stepper Interface...");

  shutdown_requested_.store(true);
  reconnect_enabled_ = false;

  if (socket_manager_)
  {
    socket_manager_->disconnect();
  }

  RCLCPP_INFO(logger_, "Successfully deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type STM32StepperInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!socket_manager_->is_connected() && reconnect_enabled_ && !shutdown_requested_.load())
  {
    socket_manager_->attempt_reconnection(reconnect_enabled_, shutdown_requested_);
    if (socket_manager_->is_connected())
    {
      protocol_ = std::make_unique<utils::STM32Protocol>(
        socket_manager_->get_socket_fd(), logger_, sequence_number_);
    }
  }
  
  if (!protocol_)
  {
    static auto last_warn_time = std::chrono::steady_clock::now();
    log_throttled("Protocol not initialized", last_warn_time);
    return hardware_interface::return_type::ERROR;
  }

  std::vector<float> positions_steps, velocities_steps;
  if (!protocol_->read_state(positions_steps, velocities_steps))
  {
    static auto last_warn_time = std::chrono::steady_clock::now();
    log_throttled("Failed to read state from STM32", last_warn_time);
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < hw_states_positions_.size() && i < positions_steps.size(); ++i)
  {
    hw_states_positions_[i] = unit_converter_->steps_to_rad(positions_steps[i], i);
    hw_states_velocities_[i] = unit_converter_->steps_per_sec_to_rad_per_sec(velocities_steps[i], i);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type STM32StepperInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!socket_manager_->is_connected() && reconnect_enabled_ && !shutdown_requested_.load())
  {
    socket_manager_->attempt_reconnection(reconnect_enabled_, shutdown_requested_);
    if (socket_manager_->is_connected())
    {
      protocol_ = std::make_unique<utils::STM32Protocol>(
        socket_manager_->get_socket_fd(), logger_, sequence_number_);
    }
  }
  
  if (!protocol_)
  {
    static auto last_warn_time = std::chrono::steady_clock::now();
    log_throttled("Protocol not initialized", last_warn_time);
    return hardware_interface::return_type::ERROR;
  }

  std::vector<float> positions_steps(hw_commands_positions_.size());
  std::vector<float> velocities_steps(hw_commands_velocities_.size());

  for (size_t i = 0; i < hw_commands_positions_.size(); ++i)
  {
    positions_steps[i] = static_cast<float>(unit_converter_->rad_to_steps(hw_commands_positions_[i], i));
    velocities_steps[i] = static_cast<float>(unit_converter_->rad_per_sec_to_steps_per_sec(hw_commands_velocities_[i], i));
  }

  if (!protocol_->send_jtc_command(positions_steps, velocities_steps))
  {
    static auto last_warn_time = std::chrono::steady_clock::now();
    log_throttled("Failed to send command to STM32", last_warn_time);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

void STM32StepperInterface::perform_lifecycle_cleanup()
{
  shutdown_requested_.store(true);
  reconnect_enabled_ = false;
  
  if (socket_manager_)
  {
    socket_manager_->disconnect();
  }
  
  protocol_.reset();
}

void STM32StepperInterface::log_throttled(
  const std::string& message,
  std::chrono::steady_clock::time_point& last_log_time)
{
  auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() > LOG_THROTTLE_MS)
  {
    RCLCPP_WARN(logger_, "%s", message.c_str());
    last_log_time = now;
  }
}

}  // namespace arctos_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arctos_hardware_interface::STM32StepperInterface,
  hardware_interface::SystemInterface)
