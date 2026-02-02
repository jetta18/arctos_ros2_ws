#include "arctos_hardware_interface/utils/socket_manager.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

namespace arctos_hardware_interface
{
namespace utils
{

STM32SocketManager::STM32SocketManager(
  const std::string& host,
  int port,
  rclcpp::Logger& logger)
: host_(host),
  port_(port),
  socket_fd_(-1),
  connected_(false),
  logger_(logger),
  last_reconnect_attempt_(std::chrono::steady_clock::time_point::min())
{
}

STM32SocketManager::~STM32SocketManager()
{
  disconnect();
}

bool STM32SocketManager::connect()
{
  std::lock_guard<std::mutex> lock(connection_mutex_);
  
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0)
  {
    RCLCPP_ERROR(logger_, "Failed to create socket");
    return false;
  }

  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = SOCKET_TIMEOUT_US;
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port_);
  
  if (inet_pton(AF_INET, host_.c_str(), &server_addr.sin_addr) <= 0)
  {
    RCLCPP_ERROR(logger_, "Invalid address: %s", host_.c_str());
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  if (::connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
  {
    RCLCPP_ERROR(logger_, "UDP connect failed to %s:%d", host_.c_str(), port_);
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  connected_ = true;
  RCLCPP_INFO(logger_, "UDP ready for STM32 at %s:%d", host_.c_str(), port_);
  return true;
}

void STM32SocketManager::disconnect()
{
  std::lock_guard<std::mutex> lock(connection_mutex_);
  
  if (socket_fd_ >= 0)
  {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  connected_ = false;
}

bool STM32SocketManager::is_connected() const
{
  return connected_;
}

int STM32SocketManager::get_socket_fd() const
{
  return socket_fd_;
}

void STM32SocketManager::attempt_reconnection(
  bool reconnect_enabled,
  const std::atomic_bool& shutdown_requested)
{
  if (shutdown_requested.load() || !reconnect_enabled)
  {
    return;
  }

  auto now = std::chrono::steady_clock::now();
  
  if (now - last_reconnect_attempt_ < RECONNECT_INTERVAL)
  {
    return;
  }
  
  last_reconnect_attempt_ = now;
  
  RCLCPP_INFO(logger_, "Attempting to reconnect to STM32...");
  
  disconnect();
  
  if (connect())
  {
    RCLCPP_INFO(logger_, "Successfully reconnected to STM32");
  }
  else
  {
    RCLCPP_WARN(logger_, "Reconnection failed, will retry in %ld ms", RECONNECT_INTERVAL.count());
  }
}

}  // namespace utils
}  // namespace arctos_hardware_interface
