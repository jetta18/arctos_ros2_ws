#ifndef ARCTOS_HARDWARE_INTERFACE__UTILS__STM32_PROTOCOL_HPP_
#define ARCTOS_HARDWARE_INTERFACE__UTILS__STM32_PROTOCOL_HPP_

#include <cstdint>
#include <vector>
#include "rclcpp/rclcpp.hpp"

namespace arctos_hardware_interface
{
namespace utils
{

struct ProtocolConstants
{
  static constexpr uint8_t PROTOCOL_VERSION = 1;
  static constexpr uint8_t CMD_JTC_STREAM = 0x01;
  static constexpr uint8_t CMD_PING = 0x20;
  static constexpr uint8_t CMD_GET_STATE = 0x10;
  static constexpr uint8_t RESP_OK = 0x00;
  static constexpr uint8_t RESP_STATE = 0x02;
  static constexpr size_t MAX_JOINTS = 6;
  static constexpr int PING_MAX_ATTEMPTS = 5;
  static constexpr std::chrono::milliseconds PING_RETRY_DELAY{200};
};

class STM32Protocol
{
public:
  STM32Protocol(int socket_fd, rclcpp::Logger& logger, uint32_t& sequence_number);

  bool ping();
  bool send_jtc_command(
    const std::vector<float>& positions,
    const std::vector<float>& velocities);
  bool read_state(
    std::vector<float>& positions,
    std::vector<float>& velocities);

private:
  int socket_fd_;
  rclcpp::Logger& logger_;
  uint32_t& sequence_number_;
};

}  // namespace utils
}  // namespace arctos_hardware_interface

#endif  // ARCTOS_HARDWARE_INTERFACE__UTILS__STM32_PROTOCOL_HPP_
