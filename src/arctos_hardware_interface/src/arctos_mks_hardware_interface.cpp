#include "arctos_hardware_interface/arctos_mks_hardware_interface.hpp"
#include "arctos_motor_driver/mks_motor_driver.hpp"

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
  
  // Call parent on_init first
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ArctosMKSHardwareInterface"), "Failed to initialize parent SystemInterface");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Expected joint names for the Arctos robot
  const std::vector<std::string> expected_joints = {
    "X_joint", "Y_joint", "Z_joint", "A_joint", "B_joint", "C_joint"
  };

  // Extract joint names from hardware info
  joint_names_.clear();
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
  }

  // Validate that we have exactly 6 joints
  if (joint_names_.size() != 6)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Expected 6 joints, but got %zu joints", joint_names_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validate that all expected joints are present
  for (const std::string & expected_joint : expected_joints)
  {
    if (std::find(joint_names_.begin(), joint_names_.end(), expected_joint) == joint_names_.end())
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("ArctosMKSHardwareInterface"),
        "Missing expected joint: %s", expected_joint.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Validate joint interfaces
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Check state interfaces (position and velocity)
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("ArctosMKSHardwareInterface"),
        "Joint '%s' has %zu state interfaces. Expected 2 (position and velocity).",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Check command interfaces (position + velocity)
    if (joint.command_interfaces.size() != 2)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("ArctosMKSHardwareInterface"),
        "Joint '%s' has %zu command interfaces. Expected 2 (position + velocity).",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate interface types
    bool has_position_state = false, has_velocity_state = false;
    for (const hardware_interface::InterfaceInfo & interface : joint.state_interfaces)
    {
      if (interface.name == "position") has_position_state = true;
      if (interface.name == "velocity") has_velocity_state = true;
    }

    if (!has_position_state || !has_velocity_state)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("ArctosMKSHardwareInterface"),
        "Joint '%s' missing required state interfaces (position and velocity)",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate command interfaces are position and velocity
    bool has_position_command = false, has_velocity_command = false;
    for (const hardware_interface::InterfaceInfo & interface : joint.command_interfaces)
    {
      if (interface.name == "position") has_position_command = true;
      if (interface.name == "velocity") has_velocity_command = true;
    }
    
    if (!has_position_command || !has_velocity_command)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("ArctosMKSHardwareInterface"),
        "Joint '%s' missing required command interfaces (position and velocity)",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Initialize state and command vectors
  const size_t num_joints = joint_names_.size();
  hw_positions_.assign(num_joints, 0.0);
  hw_velocities_.assign(num_joints, 0.0);
  hw_commands_positions_.assign(num_joints, 0.0);
  hw_commands_velocities_.assign(num_joints, 0.0);

  RCLCPP_INFO(
    rclcpp::get_logger("ArctosMKSHardwareInterface"),
    "Successfully initialized hardware interface with %zu joints", num_joints);

  // Log joint names for verification
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Joint %zu: %s", i, joint_names_[i].c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArctosMKSHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "on_configure() called");
  
  try {
    // Get CAN interface from hardware parameters
    std::string can_interface = "can0";  // Default
    auto it = info_.hardware_parameters.find("motor_can_interface");
    if (it != info_.hardware_parameters.end()) {
      can_interface = it->second;
    }
    
    // CRITICAL FIX: Don't create motor driver in on_configure to avoid segfault
    // Motor driver will be created later in on_activate when it's safer
    // Store CAN interface for later use
    can_interface_name_ = can_interface;
    
    RCLCPP_INFO(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Stored CAN interface name: %s (motor driver will be created in on_activate)", can_interface.c_str());
    
    RCLCPP_INFO(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Successfully configured hardware interface with %zu motors", joint_names_.size());
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Exception during configuration: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArctosMKSHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "on_cleanup() called");
  
  try {
    // Clean up motor driver
    if (motor_driver_) {
      RCLCPP_INFO(
        rclcpp::get_logger("ArctosMKSHardwareInterface"),
        "Cleaning up motor driver...");
      
      // Stop polling if still running
      motor_driver_->stopPolling();
      
      // Reset motor driver (destructor will handle CAN cleanup)
      motor_driver_.reset();
      
      RCLCPP_INFO(
        rclcpp::get_logger("ArctosMKSHardwareInterface"),
        "Motor driver cleaned up successfully");
    }
    
    // Reset internal state vectors
    const size_t num_joints = joint_names_.size();
    hw_positions_.assign(num_joints, 0.0);
    hw_velocities_.assign(num_joints, 0.0);
    hw_commands_positions_.assign(num_joints, 0.0);
    hw_commands_velocities_.assign(num_joints, 0.0);
    
    RCLCPP_INFO(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Successfully cleaned up hardware interface");
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Exception during cleanup: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArctosMKSHardwareInterface::export_state_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "export_state_interfaces() called");
  
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // Create position and velocity state interfaces for each joint
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
    
    // Velocity state interface
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i], 
        hardware_interface::HW_IF_VELOCITY, 
        &hw_velocities_[i]
      )
    );
    
    RCLCPP_DEBUG(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Created state interfaces for joint: %s", joint_names_[i].c_str());
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("ArctosMKSHardwareInterface"),
    "Exported %zu state interfaces (%zu joints x 2 interfaces)", 
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
  
  try {
    // Create motor driver now (safer than in on_configure)
    if (!motor_driver_) {
      RCLCPP_INFO(
        rclcpp::get_logger("ArctosMKSHardwareInterface"),
        "Creating motor driver with CAN interface: %s", can_interface_name_.c_str());
      
      // Create a temporary node for motor driver
      auto temp_node = rclcpp::Node::make_shared("motor_driver_node");
      motor_driver_ = std::make_unique<arctos_motor_driver::MKSMotorDriver>(temp_node, can_interface_name_);
      // Apply optional pacing between CAN requests from hardware params
      try {
        const std::string delay_str = getHardwareParameter("mks_inter_request_delay_us", "150");
        int delay_us = std::stoi(delay_str);
        motor_driver_->setInterRequestDelayUs(delay_us);
        RCLCPP_INFO(
          rclcpp::get_logger("ArctosMKSHardwareInterface"),
          "Set mks_inter_request_delay_us to %d via hardware params", delay_us);
      } catch (...) {
        RCLCPP_WARN(
          rclcpp::get_logger("ArctosMKSHardwareInterface"),
          "Failed to parse mks_inter_request_delay_us; using driver's default");
      }
      
      // Configure all motors
      for (const std::string& joint_name : joint_names_)
      {
        arctos_motor_driver::MotorConfig motor_config;
        
        // Load parameters from hardware parameters
        std::string param_prefix = "motors." + joint_name + ".";
        
        motor_config.motor_id = std::stoi(getHardwareParameter(param_prefix + "motor_id", "0"));
        motor_config.gear_ratio = std::stod(getHardwareParameter(param_prefix + "gear_ratio", "1.0"));
        motor_config.inverted = (getHardwareParameter(param_prefix + "inverted", "false") == "true");
        motor_config.hardware_type = getHardwareParameter(param_prefix + "hardware_type", "MKS_42D");
        motor_config.working_current = std::stoi(getHardwareParameter(param_prefix + "working_current", "1000"));
        motor_config.holding_current = std::stoi(getHardwareParameter(param_prefix + "holding_current", "70"));
        motor_config.limit_remap_enabled = (getHardwareParameter(param_prefix + "limit_remap_enabled", "false") == "true");
        motor_config.work_mode = static_cast<uint8_t>(std::stoi(getHardwareParameter(param_prefix + "work_mode", "5")));  // Default SR_vFOC
        
        // Validate and add motor
        if (motor_config.motor_id == 0 || motor_config.gear_ratio <= 0.0) {
          RCLCPP_ERROR(
            rclcpp::get_logger("ArctosMKSHardwareInterface"),
            "Invalid motor config for joint %s: id=%u, ratio=%.1f", 
            joint_name.c_str(), motor_config.motor_id, motor_config.gear_ratio);
          return hardware_interface::CallbackReturn::ERROR;
        }
        
        // Validate work mode (0-5 are valid MKS work modes)
        if (motor_config.work_mode > 5) {
          RCLCPP_ERROR(
            rclcpp::get_logger("ArctosMKSHardwareInterface"),
            "Invalid work mode %d for joint %s (valid range: 0-5)", 
            motor_config.work_mode, joint_name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
        
        if (!motor_driver_->addMotor(joint_name, motor_config)) {
          RCLCPP_ERROR(
            rclcpp::get_logger("ArctosMKSHardwareInterface"),
            "Failed to add motor for joint: %s", joint_name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
        
        RCLCPP_INFO(
          rclcpp::get_logger("ArctosMKSHardwareInterface"),
          "Configured joint '%s': motor_id=%u, gear_ratio=%.1f, inverted=%s",
          joint_name.c_str(), motor_config.motor_id, motor_config.gear_ratio,
          motor_config.inverted ? "YES" : "NO");
      }
      
      // Initialize all motors
      if (!motor_driver_->initializeMotors()) {
        RCLCPP_ERROR(
          rclcpp::get_logger("ArctosMKSHardwareInterface"),
          "Failed to initialize motors");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    
    // Enable all motors
    RCLCPP_INFO(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Enabling all motors...");
    
    if (!motor_driver_->enableAllMotors(true)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ArctosMKSHardwareInterface"),
        "Failed to enable all motors");
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    // Wait a bit for motors to stabilize
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Start background polling inside the driver
    double polling_hz = 200.0;
    {
      const std::string hz_str = getHardwareParameter("polling_frequency_hz", "200.0");
      try {
        polling_hz = std::stod(hz_str);
      } catch (...) {
        polling_hz = 200.0;
      }
    }
    motor_driver_->startPolling(polling_hz);
    RCLCPP_INFO(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Started motor polling at %.1f Hz", polling_hz);
    
    // Give the polling loop a moment to populate initial cached data
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // Initialize joint state vectors with current motor positions
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const std::string& joint_name = joint_names_[i];
      
      // Get current position from motor driver
      double current_position = motor_driver_->getJointPosition(joint_name);
      double current_velocity = motor_driver_->getJointVelocity(joint_name);
      
      // Initialize state vectors
      hw_positions_[i] = current_position;
      hw_velocities_[i] = current_velocity;
      hw_commands_positions_[i] = current_position;  // Initialize command to current position
      hw_commands_velocities_[i] = 0.0;
      
      // Validate data
      if (!motor_driver_->isJointDataValid(joint_name, 2.0)) {
        RCLCPP_WARN(
          rclcpp::get_logger("ArctosMKSHardwareInterface"),
          "Joint '%s' data may not be valid yet. Position: %.3f rad",
          joint_name.c_str(), current_position);
      }
      
      RCLCPP_INFO(
        rclcpp::get_logger("ArctosMKSHardwareInterface"),
        "Joint '%s' initialized: pos=%.3f rad, vel=%.3f rad/s",
        joint_name.c_str(), current_position, current_velocity);
    }
    
    RCLCPP_INFO(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Successfully activated hardware interface with %zu motors", joint_names_.size());
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Exception during activation: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArctosMKSHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "on_deactivate() called");
  
  if (!motor_driver_) {
    RCLCPP_WARN(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Motor driver not available during deactivation");
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  
  try {
    // Stop background polling before disabling motors
    RCLCPP_INFO(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Stopping motor polling...");
    motor_driver_->stopPolling();
    
    // Update state vectors with final positions
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const std::string& joint_name = joint_names_[i];
      
      double final_position = motor_driver_->getJointPosition(joint_name);
      double final_velocity = motor_driver_->getJointVelocity(joint_name);
      
      hw_positions_[i] = final_position;
      hw_velocities_[i] = final_velocity;
      
      RCLCPP_INFO(
        rclcpp::get_logger("ArctosMKSHardwareInterface"),
        "Joint '%s' final state: pos=%.3f rad, vel=%.3f rad/s",
        joint_name.c_str(), final_position, final_velocity);
    }
    
    // Disable all motors (they will maintain holding torque)
    RCLCPP_INFO(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Disabling all motors...");
    
    if (!motor_driver_->enableAllMotors(false)) {
      RCLCPP_WARN(
        rclcpp::get_logger("ArctosMKSHardwareInterface"),
        "Failed to disable some motors - continuing anyway");
    }
    
    // Reset command velocities to zero
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      hw_commands_velocities_[i] = 0.0;
    }
    
    RCLCPP_INFO(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Successfully deactivated hardware interface");
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Exception during deactivation: %s", e.what());
    // Continue with deactivation even if there's an error
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArctosMKSHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"), "on_shutdown() called");
  
  try {
    if (motor_driver_) {
      RCLCPP_INFO(
        rclcpp::get_logger("ArctosMKSHardwareInterface"),
        "Performing graceful shutdown of motor system...");
      
      // Ensure polling is stopped
      RCLCPP_INFO(
        rclcpp::get_logger("ArctosMKSHardwareInterface"),
        "Stopping motor polling for shutdown...");
      motor_driver_->stopPolling();

      // Disable all motors safely
      motor_driver_->enableAllMotors(false);
      
      // Wait a bit for commands to be processed
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      
      RCLCPP_INFO(
        rclcpp::get_logger("ArctosMKSHardwareInterface"),
        "Motors disabled for shutdown");
    }
    
    // Call cleanup to reset everything
    on_cleanup(rclcpp_lifecycle::State());
    
    RCLCPP_INFO(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Hardware interface shutdown completed");
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Exception during shutdown: %s", e.what());
    // Continue with shutdown even on errors
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArctosMKSHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(rclcpp::get_logger("ArctosMKSHardwareInterface"), "on_error() called - entering error handling mode");
  
  try {
    if (motor_driver_) {
      // Stop polling to prevent further CAN traffic during error handling
      motor_driver_->stopPolling();
      RCLCPP_ERROR(
        rclcpp::get_logger("ArctosMKSHardwareInterface"),
        "Emergency stopping all motors due to error state...");
      
      // Emergency stop all motors immediately
      if (!motor_driver_->emergencyStopAll()) {
        RCLCPP_ERROR(
          rclcpp::get_logger("ArctosMKSHardwareInterface"),
          "Failed to send emergency stop commands");
      }
      
      // Also try to disable motors as backup
      motor_driver_->enableAllMotors(false);
      
      RCLCPP_ERROR(
        rclcpp::get_logger("ArctosMKSHardwareInterface"),
        "Emergency stop commands sent to all motors");
    }
    
    // Reset command vectors to safe values
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      // Keep current positions but zero velocities
      hw_commands_positions_[i] = hw_positions_[i];
      hw_commands_velocities_[i] = 0.0;
    }
    
    RCLCPP_ERROR(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Hardware interface error handling completed. System in safe state.");
    
  } catch (const std::exception& e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "CRITICAL: Exception during error handling: %s", e.what());
    // Even if error handling fails, return SUCCESS to prevent infinite error loops
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArctosMKSHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!motor_driver_) {
    // Hardware interface not configured yet
    return hardware_interface::return_type::ERROR;
  }
  
  try {
    // Read cached joint states (driver polls in background)
    
    // Read current positions and velocities for each joint
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const std::string& joint_name = joint_names_[i];
      
      // Get current joint state from motor driver
      double current_position = motor_driver_->getJointPosition(joint_name);
      double current_velocity = motor_driver_->getJointVelocity(joint_name);
      
      // Update hardware interface state vectors
      // These are directly linked to the state interfaces
      hw_positions_[i] = current_position;
      hw_velocities_[i] = current_velocity;
      
      // Optional: Log data validity issues (but don't fail)
      if (!motor_driver_->isJointDataValid(joint_name, 1.0)) {
        // Data is stale, but continue with last known values
        static auto last_warning = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_warning).count() >= 5) {
          RCLCPP_WARN(
            rclcpp::get_logger("ArctosMKSHardwareInterface"),
            "Joint '%s' data may be stale (pos=%.3f, vel=%.3f)",
            joint_name.c_str(), current_position, current_velocity);
          last_warning = now;
        }
      }
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Exception in read(): %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArctosMKSHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!motor_driver_) {
    return hardware_interface::return_type::ERROR;
  }

  try {
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const std::string& joint_name = joint_names_[i];

      // Directly use the commands from the controller (position + velocity feedforward)
      const double commanded_position = hw_commands_positions_[i];
      const double commanded_velocity = hw_commands_velocities_[i];

      // Use the new, optimized driver method that sends position and velocity together
      if (!motor_driver_->setJointGoal(joint_name, commanded_position, commanded_velocity)) {
        // Log warning but don't fail the entire write operation
        static std::map<std::string, rclcpp::Time> last_error_log_time;
        auto now = rclcpp::Clock().now();

        bool should_log = false;
        if (last_error_log_time.find(joint_name) == last_error_log_time.end()) {
            should_log = true;
        } else {
            if ((now - last_error_log_time[joint_name]).seconds() > 2.0) {
                should_log = true;
            }
        }

        if (should_log) {
            RCLCPP_WARN(
                rclcpp::get_logger("ArctosMKSHardwareInterface"),
                "Failed to send goal to '%s': pos=%.3f rad, vel=%.3f rad/s",
                joint_name.c_str(), commanded_position, commanded_velocity);
            last_error_log_time[joint_name] = now;
        }
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ArctosMKSHardwareInterface"),
      "Exception in write(): %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// Helper function to get hardware parameters from info_.hardware_parameters
std::string ArctosMKSHardwareInterface::getHardwareParameter(
  const std::string& param_name, const std::string& default_value) const
{
  auto it = info_.hardware_parameters.find(param_name);
  if (it != info_.hardware_parameters.end()) {
    return it->second;
  }
  return default_value;
}

}  // namespace arctos_hardware_interface

// Export the hardware interface as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arctos_hardware_interface::ArctosMKSHardwareInterface, hardware_interface::SystemInterface)
