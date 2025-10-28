#include "arctos_hardware_interface/arctos_mks_hardware_interface.hpp"
#include "arctos_motor_driver/mks_motor_driver.hpp"
#include "arctos_motor_driver/mks_motor_manager.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <thread>
#include <algorithm>
#include <map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "arctos_hardware_interface/constants.hpp"

namespace arctos_hardware_interface
{

ArctosMKSHardwareInterface::ArctosMKSHardwareInterface()
{
  // Constructor - basic initialization
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Constructor called");
}

hardware_interface::CallbackReturn ArctosMKSHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "on_init() called");
  
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Failed to initialize parent SystemInterface");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!extract_joint_names()) return hardware_interface::CallbackReturn::ERROR;
  if (!validate_joint_count()) return hardware_interface::CallbackReturn::ERROR;
  if (!validate_expected_joints()) return hardware_interface::CallbackReturn::ERROR;
  if (!validate_joint_interfaces()) return hardware_interface::CallbackReturn::ERROR;
  init_state_vectors();

  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"),
              "✓ Successfully initialized hardware interface with %zu joints",
              joint_names_.size());
  log_joint_names();

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn ArctosMKSHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "on_configure() called");
  
  // Get CAN interface from hardware parameters
  can_interface_name_ = getHardwareParameter("motor_can_interface", "can0");
  
  RCLCPP_INFO(
    rclcpp::get_logger("ArctosMKSHardwareInterface"),
    "Configured with CAN interface: %s", can_interface_name_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArctosMKSHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "on_cleanup() called");
  
  // Clean up motor manager and driver
  if (motor_manager_) {
    motor_manager_.reset();
  }
  
  if (motor_driver_) {
    motor_driver_.reset();
  }
  
  if (can_node_) {
    can_node_.reset();
  }
  
  // Reset state vectors
  const size_t num_joints = joint_names_.size();
  hw_positions_.assign(num_joints, 0.0);
  hw_velocities_.assign(num_joints, 0.0);
  hw_commands_positions_.assign(num_joints, 0.0);
  hw_commands_velocities_.assign(num_joints, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArctosMKSHardwareInterface::export_state_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "export_state_interfaces() called");
  
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // Create ONLY position state interface for each joint (JointTrajectoryController requirement)
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    // Position state interface
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i], 
        hardware_interface::HW_IF_POSITION, 
        &hw_positions_[i]
      )
    );
    
    RCLCPP_DEBUG(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Created position state interface for joint: %s", joint_names_[i].c_str());
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("ArctosMKSHardwareInterface"),
    "Exported %zu state interfaces (%zu joints x 1 position interface)", 
    state_interfaces.size(), joint_names_.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArctosMKSHardwareInterface::export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "export_command_interfaces() called");
  
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  // Create position and velocity command interfaces for each joint
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    // Position command interface
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_names_[i], 
        hardware_interface::HW_IF_POSITION, 
        &hw_commands_positions_[i]
      )
    );
    
    // Velocity command interface
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_names_[i], 
        hardware_interface::HW_IF_VELOCITY, 
        &hw_commands_velocities_[i]
      )
    );
    
    RCLCPP_DEBUG(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Created position and velocity command interfaces for joint: %s", joint_names_[i].c_str());
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("ArctosMKSHardwareInterface"),
    "Exported %zu command interfaces (%zu joints x 2 interfaces: position + velocity)", 
    command_interfaces.size(), joint_names_.size());

  return command_interfaces;
}

hardware_interface::CallbackReturn ArctosMKSHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "on_activate() called");

  if (!init_can_stack()) return hardware_interface::CallbackReturn::ERROR;
  if (!load_all_joint_configs()) return hardware_interface::CallbackReturn::ERROR;
  if (!apply_all_motor_configs()) return hardware_interface::CallbackReturn::ERROR;
  if (!read_initial_encoders_with_settle(constants::kPreReadSettleMs, constants::kInitialEncoderTimeoutMs)) return hardware_interface::CallbackReturn::ERROR;
  initialize_hw_positions_from_manager();
  if (!enable_all_motors()) return hardware_interface::CallbackReturn::ERROR;

  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"),
              "✓ Hardware interface activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArctosMKSHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "on_deactivate() called");
  
  // Disable all motors
  if (motor_manager_ && motor_driver_) {
    RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Disabling all motors...");
    for (const std::string& joint_name : joint_names_) {
      uint32_t motor_id = motor_manager_->getMotorId(joint_name);
      if (motor_id > 0) {
        motor_driver_->enableMotor(motor_id, false);
      }
    }
  }
  
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Hardware interface deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArctosMKSHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "on_shutdown() called");
  
  // Deactivate first
  on_deactivate(rclcpp_lifecycle::State());
  
  // Then cleanup
  on_cleanup(rclcpp_lifecycle::State());
  
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Shutdown complete");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArctosMKSHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(rclcpp::get_logger("ArctosMKSHardwareInterface"), "❌ ERROR STATE - Executing emergency stop!");
  
  // Emergency stop all motors
  if (motor_manager_ && motor_driver_) {
    for (const std::string& joint_name : joint_names_) {
      uint32_t motor_id = motor_manager_->getMotorId(joint_name);
      if (motor_id > 0) {
        motor_driver_->emergencyStop(motor_id);
        motor_driver_->enableMotor(motor_id, false);
      }
    }
  }
  
  RCLCPP_ERROR(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Emergency stop executed");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArctosMKSHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!motor_driver_ || !motor_manager_) {
    return hardware_interface::return_type::ERROR;
  }

  const size_t n = joint_names_.size();
  if (n == 0) {
    return hardware_interface::return_type::OK;
  }

  static size_t rr_index = 0;
  if (rr_index >= n) rr_index = 0;

  size_t polled = 0;
  for (size_t k = 0; k < n; ++k) {
    size_t i = (rr_index + k) % n;
    const std::string& joint = joint_names_[i];
    uint32_t motor_id = motor_manager_->getMotorId(joint);
    if (motor_id == 0) continue;
    motor_driver_->requestEncoderReading(motor_id);
    rr_index = (i + 1) % n;
    polled = 1;
    break;
  }

  if (can_node_) {
    rclcpp::spin_some(can_node_);
  }

  for (size_t i = 0; i < n; ++i) {
    const std::string& joint = joint_names_[i];
    if (motor_manager_->isJointDataValid(joint, constants::kEncoderValidMaxAgeSec)) {
      hw_positions_[i] = motor_manager_->getJointPosition(joint);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArctosMKSHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!motor_driver_ || !motor_manager_) {
    return hardware_interface::return_type::ERROR;
  }

  const size_t n = joint_names_.size();
  static std::vector<int32_t> last_axis_sent;
  if (last_axis_sent.size() != n) {
    last_axis_sent.assign(n, std::numeric_limits<int32_t>::min());
  }
  static size_t rr_write_index = 0;
  if (rr_write_index >= n) rr_write_index = 0;

  auto clamp_axis24 = [](int64_t v) -> int32_t {
    if (v < -8388607) return -8388607;
    if (v >  8388607) return  8388607;
    return static_cast<int32_t>(v);
  };

  // Non-blocking: send at most one F5 command per cycle (round-robin across joints)
  for (size_t attempt = 0; attempt < n; ++attempt) {
    size_t i = (rr_write_index + attempt) % n;
    const std::string& joint = joint_names_[i];
    const auto* cfg = motor_manager_->getMotorConfig(joint);
    if (!cfg || cfg->motor_id == 0) {
      continue;
    }
    if (!std::isfinite(hw_commands_positions_[i])) {
      continue;
    }

    const double joint_rad = hw_commands_positions_[i];
    const double motor_angle_rad = joint_rad * cfg->gear_ratio * (cfg->inverted ? -1.0 : 1.0);
    const int64_t axis = static_cast<int64_t>(
        std::llround((motor_angle_rad / (2.0 * M_PI)) * 0x4000));
    const int32_t axis24 = clamp_axis24(axis);

    constexpr int kAxisDeadband = 4;
    if (last_axis_sent[i] != std::numeric_limits<int32_t>::min()) {
      if (std::llabs(static_cast<long long>(axis24) - static_cast<long long>(last_axis_sent[i])) < kAxisDeadband) {
        rr_write_index = (i + 1) % n;
        continue;
      }
    }

    uint16_t speed_rpm = 2500; // faster default for better tracking in open-loop
    if (i < hw_commands_velocities_.size() && std::isfinite(hw_commands_velocities_[i]) && hw_commands_velocities_[i] != 0.0) {
      double motor_rpm = (hw_commands_velocities_[i] * cfg->gear_ratio) * (60.0 / (2.0 * M_PI));
      motor_rpm = std::fabs(motor_rpm);
      if (motor_rpm < 50.0) motor_rpm = 50.0;
      if (motor_rpm > 3000.0) motor_rpm = 3000.0;
      speed_rpm = static_cast<uint16_t>(std::lround(motor_rpm));
    }
    const uint8_t acceleration = 250; // slightly higher accel to keep up

    const bool ok = motor_driver_->setAbsolutePositionByAxis(cfg->motor_id, axis24, speed_rpm, acceleration);
    rr_write_index = (i + 1) % n;
    if (ok) {
      last_axis_sent[i] = axis24;
    }
    // send only one joint per cycle
    break;
  }

  return hardware_interface::return_type::OK;
}


}  // namespace arctos_hardware_interface

// Export the hardware interface as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arctos_hardware_interface::ArctosMKSHardwareInterface, hardware_interface::SystemInterface)
