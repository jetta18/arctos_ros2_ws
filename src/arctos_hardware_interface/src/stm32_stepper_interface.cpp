#include "arctos_hardware_interface/stm32_stepper_interface.hpp"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <cmath>
#include <thread>
#include <mutex>

namespace arctos_hardware_interface
{

STM32StepperInterface::STM32StepperInterface()
: stm32_port_(8888),
  socket_fd_(-1),
  connected_(false),
  reconnect_enabled_(true),
  last_reconnect_attempt_(std::chrono::steady_clock::time_point::min()),
  sequence_number_(0),
  logger_(rclcpp::get_logger("STM32StepperInterface"))
{
}

STM32StepperInterface::~STM32StepperInterface()
{
  disconnect_from_stm32();
}

hardware_interface::CallbackReturn STM32StepperInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != 
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read configuration parameters
  stm32_host_ = info_.hardware_parameters["stm32_host"];
  stm32_port_ = std::stoi(info_.hardware_parameters["stm32_port"]);

  RCLCPP_INFO(logger_, "STM32 Host: %s", stm32_host_.c_str());
  RCLCPP_INFO(logger_, "STM32 Port: %d", stm32_port_);

  // Initialize joint data vectors
  const size_t num_joints = info_.joints.size();
  hw_commands_positions_.resize(num_joints, 0.0);
  hw_commands_velocities_.resize(num_joints, 0.0);
  hw_states_positions_.resize(num_joints, 0.0);
  hw_states_velocities_.resize(num_joints, 0.0);
  gear_ratios_.resize(num_joints, 1.0);
  steps_per_revolution_.resize(num_joints, 0.0);
  joint_inversions_.resize(num_joints, false);

  // Load gear ratios from parameters
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

    // Calculate steps per revolution for this joint
    steps_per_revolution_[i] = STEPS_PER_REV * MICROSTEPS * gear_ratios_[i];

    // Load inversion parameter
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

  RCLCPP_INFO(logger_, "Initialized with %zu joints", num_joints);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32StepperInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Configuring STM32 Stepper Interface...");
  
  // Connect to STM32
  if (!connect_to_stm32())
  {
    RCLCPP_ERROR(logger_, "Failed to connect to STM32");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Successfully configured");
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

  // Read initial state from STM32
  if (!read_state())
  {
    RCLCPP_WARN(logger_, "Could not read initial state, using zeros");
  }

  // Initialize commands to current state
  hw_commands_positions_ = hw_states_positions_;
  hw_commands_velocities_ = hw_states_velocities_;

  RCLCPP_INFO(logger_, "Successfully activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32StepperInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating STM32 Stepper Interface...");
  
  // Stop all motion by sending zero velocities
  std::vector<double> zero_velocities(info_.joints.size(), 0.0);
  send_jtc_command(hw_states_positions_, zero_velocities);

  disconnect_from_stm32();

  RCLCPP_INFO(logger_, "Successfully deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type STM32StepperInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Check connection and attempt reconnection if needed
  if (!connected_ && reconnect_enabled_)
  {
    attempt_reconnection();
  }
  
  // Read current state from STM32
  if (!read_state())
  {
    static auto last_warn_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_warn_time).count() > 1000)
    {
      RCLCPP_WARN(logger_, "Failed to read state from STM32");
      last_warn_time = now;
    }
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type STM32StepperInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Check connection and attempt reconnection if needed
  if (!connected_ && reconnect_enabled_)
  {
    attempt_reconnection();
  }
  
  // Send JTC command to STM32
  if (!send_jtc_command(hw_commands_positions_, hw_commands_velocities_))
  {
    static auto last_warn_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_warn_time).count() > 1000)
    {
      RCLCPP_WARN(logger_, "Failed to send command to STM32");
      last_warn_time = now;
    }
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool STM32StepperInterface::connect_to_stm32()
{
  std::lock_guard<std::mutex> lock(connection_mutex_);
  
  // Create UDP socket.
  // We use a "connected" UDP socket (connect() on a SOCK_DGRAM socket) so that:
  // - send() can be used instead of sendto()
  // - recv() will only accept packets from the connected peer
  // - the kernel assigns a local ephemeral port automatically
  // The STM32 replies to the source IP/port it sees, so this works well.
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0)
  {
    RCLCPP_ERROR(logger_, "Failed to create socket");
    return false;
  }

  // Set socket timeouts.
  // For UDP streaming we want bounded waiting in read_state().
  struct timeval timeout;
  timeout.tv_sec = 2;
  timeout.tv_usec = 0;
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  // Connect UDP socket to STM32 (sets default peer for send/recv)
  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(stm32_port_);
  
  if (inet_pton(AF_INET, stm32_host_.c_str(), &server_addr.sin_addr) <= 0)
  {
    RCLCPP_ERROR(logger_, "Invalid address: %s", stm32_host_.c_str());
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  if (connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
  {
    RCLCPP_ERROR(logger_, "UDP connect failed to %s:%d", stm32_host_.c_str(), stm32_port_);
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  connected_ = true;
  RCLCPP_INFO(logger_, "UDP ready for STM32 at %s:%d", stm32_host_.c_str(), stm32_port_);
  return true;
}

void STM32StepperInterface::disconnect_from_stm32()
{
  std::lock_guard<std::mutex> lock(connection_mutex_);
  
  if (socket_fd_ >= 0)
  {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  connected_ = false;
}

bool STM32StepperInterface::send_jtc_command(
  const std::vector<double>& positions, 
  const std::vector<double>& velocities)
{
  if (!connected_ || socket_fd_ < 0)
  {
    return false;
  }

  // Build command packet.
  // NOTE: This matches the STM32 firmware's cmd_header_t + cmd_jtc_stream_t layout.
  struct __attribute__((packed)) {
    uint8_t version;
    uint8_t command;
    uint16_t length;
    uint32_t sequence;
    uint32_t timestamp_ms;
    float positions[6];
    float velocities[6];
    float accelerations[6];
  } packet;

  packet.version = PROTOCOL_VERSION;
  packet.command = CMD_JTC_STREAM;
  packet.length = sizeof(packet.timestamp_ms) + sizeof(packet.positions) + 
                  sizeof(packet.velocities) + sizeof(packet.accelerations);
  packet.sequence = ++sequence_number_;
  packet.timestamp_ms = static_cast<uint32_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());

  // Convert from radians to steps and copy joint data
  for (size_t i = 0; i < 6 && i < positions.size(); ++i)
  {
    packet.positions[i] = static_cast<float>(rad_to_steps(positions[i], i));
    packet.velocities[i] = static_cast<float>(rad_per_sec_to_steps_per_sec(velocities[i], i));
    packet.accelerations[i] = 0.0f;  // Not used currently
  }

  // Send packet (fire-and-forget).
  // In the current UDP streaming architecture, the STM32 does not send RESP_OK for setpoints.
  ssize_t sent = send(socket_fd_, &packet, sizeof(packet), 0);
  if (sent != sizeof(packet))
  {
    RCLCPP_ERROR(logger_, "Failed to send complete packet");
    std::lock_guard<std::mutex> lock(connection_mutex_);
    connected_ = false;
    return false;
  }

  return true;
}

bool STM32StepperInterface::read_state()
{
  if (!connected_ || socket_fd_ < 0)
  {
    return false;
  }

  // Build GET_STATE command.
  // The STM32 answers with a single UDP datagram containing:
  // - resp_header_t
  // - resp_state_t payload
  struct __attribute__((packed)) {
    uint8_t version;
    uint8_t command;
    uint16_t length;
    uint32_t sequence;
  } packet;

  packet.version = PROTOCOL_VERSION;
  packet.command = CMD_GET_STATE;
  packet.length = 0;
  packet.sequence = ++sequence_number_;

  // Send packet
  ssize_t sent = send(socket_fd_, &packet, sizeof(packet), 0);
  if (sent != sizeof(packet))
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    connected_ = false;
    return false;
  }

  struct __attribute__((packed)) {
    uint8_t version;
    uint8_t response;
    uint16_t length;
    uint32_t sequence;
    float positions[6];
    float velocities[6];
  } response;

  ssize_t received = recv(socket_fd_, &response, sizeof(response), 0);
  if (received != sizeof(response))
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    connected_ = false;
    return false;
  }

  if (response.version != PROTOCOL_VERSION)
  {
    return false;
  }

  if (response.response != RESP_STATE)
  {
    return false;
  }

  if (response.length != (sizeof(response.positions) + sizeof(response.velocities)))
  {
    return false;
  }

  // Update state (convert from steps to radians)
  for (size_t i = 0; i < 6 && i < hw_states_positions_.size(); ++i)
  {
    hw_states_positions_[i] = steps_to_rad(static_cast<double>(response.positions[i]), i);
    hw_states_velocities_[i] = steps_per_sec_to_rad_per_sec(static_cast<double>(response.velocities[i]), i);
  }

  return true;
}

// Unit conversion functions
double STM32StepperInterface::rad_to_steps(double radians, size_t joint_index) const
{
  if (joint_index >= steps_per_revolution_.size())
  {
    return 0.0;
  }
  
  // Apply inversion if configured
  double effective_radians = joint_inversions_[joint_index] ? -radians : radians;
  
  // steps = radians * (steps_per_revolution / 2π)
  return effective_radians * (steps_per_revolution_[joint_index] / (2.0 * M_PI));
}

double STM32StepperInterface::steps_to_rad(double steps, size_t joint_index) const
{
  if (joint_index >= steps_per_revolution_.size())
  {
    return 0.0;
  }
  
  // radians = steps * (2π / steps_per_revolution)
  double radians = steps * (2.0 * M_PI / steps_per_revolution_[joint_index]);
  
  // Apply inversion if configured
  return joint_inversions_[joint_index] ? -radians : radians;
}

double STM32StepperInterface::rad_per_sec_to_steps_per_sec(double rad_per_sec, size_t joint_index) const
{
  if (joint_index >= steps_per_revolution_.size())
  {
    return 0.0;
  }
  
  // Apply inversion if configured
  double effective_rad_per_sec = joint_inversions_[joint_index] ? -rad_per_sec : rad_per_sec;
  
  // steps/s = rad/s * (steps_per_revolution / 2π)
  return effective_rad_per_sec * (steps_per_revolution_[joint_index] / (2.0 * M_PI));
}

double STM32StepperInterface::steps_per_sec_to_rad_per_sec(double steps_per_sec, size_t joint_index) const
{
  if (joint_index >= steps_per_revolution_.size())
  {
    return 0.0;
  }
  
  // rad/s = steps/s * (2π / steps_per_revolution)
  double rad_per_sec = steps_per_sec * (2.0 * M_PI / steps_per_revolution_[joint_index]);
  
  // Apply inversion if configured
  return joint_inversions_[joint_index] ? -rad_per_sec : rad_per_sec;
}

void STM32StepperInterface::attempt_reconnection()
{
  auto now = std::chrono::steady_clock::now();
  
  // Rate limit reconnection attempts
  if (now - last_reconnect_attempt_ < RECONNECT_INTERVAL)
  {
    return;
  }
  
  last_reconnect_attempt_ = now;
  
  RCLCPP_INFO(logger_, "Attempting to reconnect to STM32...");
  
  // Clean up existing connection
  disconnect_from_stm32();
  
  // Try to reconnect
  if (connect_to_stm32())
  {
    RCLCPP_INFO(logger_, "Successfully reconnected to STM32");
    
    // Read initial state after reconnection
    if (!read_state())
    {
      RCLCPP_WARN(logger_, "Could not read initial state after reconnection");
    }
  }
  else
  {
    RCLCPP_WARN(logger_, "Reconnection failed, will retry in %ld ms", RECONNECT_INTERVAL.count());
  }
}

}  // namespace arctos_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arctos_hardware_interface::STM32StepperInterface,
  hardware_interface::SystemInterface)
