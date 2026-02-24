#include "arctos_hardware_interface/stm32_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace arctos_hardware_interface
{

STM32HardwareInterface::STM32HardwareInterface()
: hw_system_state_(0.0),
  bridge_system_state_(0.0),
  bridge_servo_pulse_us_(1500.0),
  state_received_(false),
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
  stop_ros_interface();
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                          */
/* ------------------------------------------------------------------ */

hardware_interface::CallbackReturn STM32HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

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
  hw_cmd_positions_.resize(arm_joint_count, 0.0);
  hw_cmd_positions_prev_.resize(arm_joint_count, std::numeric_limits<double>::quiet_NaN());
  bridge_positions_.resize(arm_joint_count, 0.0);
  bridge_velocities_.resize(arm_joint_count, 0.0);

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
  RCLCPP_INFO(logger_, "Configuring (bridge-backed mode)...");
  start_ros_interface();
  RCLCPP_INFO(logger_, "Successfully configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32HardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up...");
  stop_ros_interface();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32HardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Shutting down...");
  stop_ros_interface();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32HardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(logger_, "Error state");
  stop_ros_interface();
  return hardware_interface::CallbackReturn::SUCCESS;
}

/* ------------------------------------------------------------------ */
/* Interface export                                                   */
/* ------------------------------------------------------------------ */

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

  size_t cmd_idx = 0;
  for (const auto & joint : info_.joints)
  {
    if (joint.name == "Left_jaw_joint" || joint.name == "Right_jaw_joint")
    {
      continue;
    }
    interfaces.emplace_back(
      joint.name, "position", &hw_cmd_positions_[cmd_idx]);
    ++cmd_idx;
  }

  if (gripper_enabled_)
  {
    interfaces.emplace_back(
      "Left_jaw_joint", "position", &hw_gripper_command_);
  }

  return interfaces;
}

/* ------------------------------------------------------------------ */
/* Activate / Deactivate                                              */
/* ------------------------------------------------------------------ */

hardware_interface::CallbackReturn STM32HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating...");

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t i = 0; i < hw_cmd_positions_.size(); ++i)
    {
      hw_cmd_positions_[i] = bridge_positions_[i];
      hw_cmd_positions_prev_[i] = bridge_positions_[i];
    }
  }

  RCLCPP_INFO(logger_, "Successfully activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating...");
  RCLCPP_INFO(logger_, "Successfully deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/* ------------------------------------------------------------------ */
/* read() — copy latest state from bridge topic callback              */
/* ------------------------------------------------------------------ */

hardware_interface::return_type STM32HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!state_received_)
  {
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < hw_states_positions_.size(); ++i)
  {
    hw_states_positions_[i] = bridge_positions_[i];
    hw_states_velocities_[i] = bridge_velocities_[i];
  }

  hw_system_state_ = bridge_system_state_;

  if (gripper_enabled_)
  {
    double pulse = bridge_servo_pulse_us_;
    double position_m = 0.0;
    if (pulse >= servo_closed_pulse_us_ && pulse <= servo_open_pulse_us_)
    {
      double fraction = (pulse - servo_closed_pulse_us_) /
                        (servo_open_pulse_us_ - servo_closed_pulse_us_);
      position_m = fraction * gripper_max_opening_m_;
    }
    hw_gripper_position_left_ = position_m;
    hw_gripper_position_right_ = position_m;
  }

  return hardware_interface::return_type::OK;
}

/* ------------------------------------------------------------------ */
/* write() — publish position commands to bridge topics               */
/* ------------------------------------------------------------------ */

hardware_interface::return_type STM32HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!cmd_pos_pub_)
  {
    return hardware_interface::return_type::OK;
  }

  bool any_changed = false;
  for (size_t i = 0; i < hw_cmd_positions_.size(); ++i)
  {
    if (std::isnan(hw_cmd_positions_prev_[i]) ||
        std::abs(hw_cmd_positions_[i] - hw_cmd_positions_prev_[i]) > POSITION_CMD_DEADBAND_RAD)
    {
      any_changed = true;
      break;
    }
  }

  if (any_changed)
  {
    auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
    msg->data.resize(hw_cmd_positions_.size());
    for (size_t i = 0; i < hw_cmd_positions_.size(); ++i)
    {
      msg->data[i] = hw_cmd_positions_[i];
    }
    cmd_pos_pub_->publish(std::move(msg));

    for (size_t i = 0; i < hw_cmd_positions_.size(); ++i)
    {
      hw_cmd_positions_prev_[i] = hw_cmd_positions_[i];
    }
  }

  if (gripper_enabled_ && cmd_servo_pub_)
  {
    bool command_changed = std::isnan(hw_gripper_command_prev_) ||
                           std::abs(hw_gripper_command_ - hw_gripper_command_prev_) > 1e-6;
    if (command_changed)
    {
      double clamped = std::clamp(hw_gripper_command_, 0.0, gripper_max_opening_m_);
      double fraction = clamped / gripper_max_opening_m_;
      auto pulse = static_cast<double>(
        servo_closed_pulse_us_ +
        fraction * (servo_open_pulse_us_ - servo_closed_pulse_us_));

      double distance_m = std::abs(clamped - hw_gripper_position_left_);
      double duration_ms = std::clamp(
        distance_m / servo_speed_m_per_s_ * 1000.0, 0.0, 5000.0);

      auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
      msg->data = {pulse, duration_ms};
      cmd_servo_pub_->publish(std::move(msg));

      hw_gripper_command_prev_ = hw_gripper_command_;
    }
  }

  return hardware_interface::return_type::OK;
}

/* ------------------------------------------------------------------ */
/* Internal ROS node for bridge communication                         */
/* ------------------------------------------------------------------ */

void STM32HardwareInterface::start_ros_interface()
{
  if (ros_node_)
  {
    return;
  }

  ros_node_ = rclcpp::Node::make_shared("arctos_hw_bridge_client");

  state_sub_ = ros_node_->create_subscription<arctos_msgs::msg::ArctosState>(
    "/arctos/state", 10,
    std::bind(&STM32HardwareInterface::state_callback, this, std::placeholders::_1));

  cmd_pos_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/arctos/cmd_positions", 10);

  cmd_servo_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/arctos/cmd_servo", 10);

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(ros_node_);

  executor_thread_ = std::thread([this]()
  {
    executor_->spin();
  });

  RCLCPP_INFO(logger_, "ROS bridge interface started");
}

void STM32HardwareInterface::stop_ros_interface()
{
  if (executor_)
  {
    executor_->cancel();
  }

  if (executor_thread_.joinable())
  {
    executor_thread_.join();
  }

  state_sub_.reset();
  cmd_pos_pub_.reset();
  cmd_servo_pub_.reset();

  if (executor_)
  {
    executor_->remove_node(ros_node_);
    executor_.reset();
  }

  ros_node_.reset();
}

void STM32HardwareInterface::state_callback(
  const arctos_msgs::msg::ArctosState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  size_t count = std::min(bridge_positions_.size(), msg->positions.size());
  for (size_t i = 0; i < count; ++i)
  {
    bridge_positions_[i] = msg->positions[i];
    bridge_velocities_[i] = msg->velocities[i];
  }

  bridge_system_state_ = static_cast<double>(msg->system_state);
  bridge_servo_pulse_us_ = static_cast<double>(msg->servo_pulse_us);
  state_received_ = true;
}

}  // namespace arctos_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arctos_hardware_interface::STM32HardwareInterface,
  hardware_interface::SystemInterface)
