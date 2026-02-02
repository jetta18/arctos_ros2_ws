#ifndef ARCTOS_HARDWARE_INTERFACE__UTILS__SOCKET_MANAGER_HPP_
#define ARCTOS_HARDWARE_INTERFACE__UTILS__SOCKET_MANAGER_HPP_

#include <string>
#include <mutex>
#include <chrono>
#include <atomic>
#include "rclcpp/rclcpp.hpp"

namespace arctos_hardware_interface
{
namespace utils
{

class STM32SocketManager
{
public:
  STM32SocketManager(
    const std::string& host,
    int port,
    rclcpp::Logger& logger);
  
  ~STM32SocketManager();

  bool connect();
  void disconnect();
  bool is_connected() const;
  int get_socket_fd() const;

  void attempt_reconnection(
    bool reconnect_enabled,
    const std::atomic_bool& shutdown_requested);

private:
  std::string host_;
  int port_;
  int socket_fd_;
  bool connected_;
  std::mutex connection_mutex_;
  rclcpp::Logger& logger_;

  std::chrono::steady_clock::time_point last_reconnect_attempt_;
  static constexpr std::chrono::milliseconds RECONNECT_INTERVAL{5000};
  static constexpr int SOCKET_TIMEOUT_US = 200000;
};

}  // namespace utils
}  // namespace arctos_hardware_interface

#endif  // ARCTOS_HARDWARE_INTERFACE__UTILS__SOCKET_MANAGER_HPP_
