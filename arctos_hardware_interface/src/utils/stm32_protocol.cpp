#include "arctos_hardware_interface/utils/stm32_protocol.hpp"
#include <sys/socket.h>
#include <cstring>
#include <chrono>
#include <thread>

namespace arctos_hardware_interface
{
namespace utils
{

STM32Protocol::STM32Protocol(int socket_fd, rclcpp::Logger& logger, uint32_t& sequence_number)
: socket_fd_(socket_fd),
  logger_(logger),
  sequence_number_(sequence_number)
{
}

bool STM32Protocol::ping()
{
  if (socket_fd_ < 0)
  {
    return false;
  }

  struct __attribute__((packed)) {
    uint8_t version;
    uint8_t command;
    uint16_t length;
    uint32_t sequence;
  } packet;

  struct __attribute__((packed)) {
    uint8_t version;
    uint8_t response;
    uint16_t length;
    uint32_t sequence;
  } response;

  for (int attempt = 0; attempt < ProtocolConstants::PING_MAX_ATTEMPTS; ++attempt)
  {
    packet.version = ProtocolConstants::PROTOCOL_VERSION;
    packet.command = ProtocolConstants::CMD_PING;
    packet.length = 0;
    packet.sequence = ++sequence_number_;

    ssize_t sent = send(socket_fd_, &packet, sizeof(packet), 0);
    if (sent != sizeof(packet))
    {
      return false;
    }

    ssize_t received = recv(socket_fd_, &response, sizeof(response), 0);
    if (received == sizeof(response) &&
        response.version == ProtocolConstants::PROTOCOL_VERSION &&
        response.response == ProtocolConstants::RESP_OK)
    {
      return true;
    }

    std::this_thread::sleep_for(ProtocolConstants::PING_RETRY_DELAY);
  }

  return false;
}

bool STM32Protocol::send_jtc_command(
  const std::vector<float>& positions,
  const std::vector<float>& velocities)
{
  if (socket_fd_ < 0)
  {
    return false;
  }

  struct __attribute__((packed)) {
    uint8_t version;
    uint8_t command;
    uint16_t length;
    uint32_t sequence;
    uint32_t timestamp_ms;
    float positions[ProtocolConstants::MAX_JOINTS];
    float velocities[ProtocolConstants::MAX_JOINTS];
    float accelerations[ProtocolConstants::MAX_JOINTS];
  } packet;

  packet.version = ProtocolConstants::PROTOCOL_VERSION;
  packet.command = ProtocolConstants::CMD_JTC_STREAM;
  packet.length = sizeof(packet.timestamp_ms) + sizeof(packet.positions) + 
                  sizeof(packet.velocities) + sizeof(packet.accelerations);
  packet.sequence = ++sequence_number_;
  packet.timestamp_ms = static_cast<uint32_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());

  std::memset(packet.positions, 0, sizeof(packet.positions));
  std::memset(packet.velocities, 0, sizeof(packet.velocities));
  std::memset(packet.accelerations, 0, sizeof(packet.accelerations));

  for (size_t i = 0; i < ProtocolConstants::MAX_JOINTS && i < positions.size(); ++i)
  {
    packet.positions[i] = positions[i];
    packet.velocities[i] = velocities[i];
  }

  ssize_t sent = send(socket_fd_, &packet, sizeof(packet), 0);
  if (sent != sizeof(packet))
  {
    RCLCPP_ERROR(logger_, "Failed to send complete packet");
    return false;
  }

  return true;
}

bool STM32Protocol::read_state(
  std::vector<float>& positions,
  std::vector<float>& velocities)
{
  if (socket_fd_ < 0)
  {
    return false;
  }

  struct __attribute__((packed)) {
    uint8_t version;
    uint8_t command;
    uint16_t length;
    uint32_t sequence;
  } packet;

  packet.version = ProtocolConstants::PROTOCOL_VERSION;
  packet.command = ProtocolConstants::CMD_GET_STATE;
  packet.length = 0;
  packet.sequence = ++sequence_number_;

  ssize_t sent = send(socket_fd_, &packet, sizeof(packet), 0);
  if (sent != sizeof(packet))
  {
    return false;
  }

  struct __attribute__((packed)) {
    uint8_t version;
    uint8_t response;
    uint16_t length;
    uint32_t sequence;
    float positions[ProtocolConstants::MAX_JOINTS];
    float velocities[ProtocolConstants::MAX_JOINTS];
  } response;

  ssize_t received = recv(socket_fd_, &response, sizeof(response), 0);
  if (received != sizeof(response))
  {
    return false;
  }

  if (response.version != ProtocolConstants::PROTOCOL_VERSION ||
      response.response != ProtocolConstants::RESP_STATE ||
      response.length != (sizeof(response.positions) + sizeof(response.velocities)))
  {
    return false;
  }

  positions.resize(ProtocolConstants::MAX_JOINTS);
  velocities.resize(ProtocolConstants::MAX_JOINTS);

  for (size_t i = 0; i < ProtocolConstants::MAX_JOINTS; ++i)
  {
    positions[i] = response.positions[i];
    velocities[i] = response.velocities[i];
  }

  return true;
}

}  // namespace utils
}  // namespace arctos_hardware_interface
