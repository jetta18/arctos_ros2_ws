#include "arctos_hardware_interface/utils/stm32_protocol.hpp"

#include <sys/socket.h>
#include <cstring>
#include <chrono>
#include <thread>

namespace arctos_hardware_interface
{
namespace utils
{

STM32Protocol::STM32Protocol(int socket_fd, rclcpp::Logger & logger)
: socket_fd_(socket_fd),
  logger_(logger),
  sequence_number_(0)
{
}

bool STM32Protocol::send_command(
  uint8_t command, const void * payload, uint16_t payload_len)
{
  if (socket_fd_ < 0)
  {
    return false;
  }

  uint8_t buffer[ProtocolConstants::MAX_PACKET_SIZE];
  Header header;
  header.version = ProtocolConstants::PROTOCOL_VERSION;
  header.command = command;
  header.length = payload_len;
  header.sequence = ++sequence_number_;

  std::memcpy(buffer, &header, sizeof(header));
  if (payload_len > 0 && payload != nullptr)
  {
    std::memcpy(buffer + sizeof(header), payload, payload_len);
  }

  uint16_t total = static_cast<uint16_t>(sizeof(header) + payload_len);
  ssize_t sent = send(socket_fd_, buffer, total, 0);
  return sent == static_cast<ssize_t>(total);
}

bool STM32Protocol::receive_response(
  RespHeader & header_out, uint8_t * payload_out,
  uint16_t max_payload, uint16_t & payload_len_out)
{
  uint8_t buffer[ProtocolConstants::MAX_PACKET_SIZE];
  ssize_t received = recv(socket_fd_, buffer, sizeof(buffer), 0);
  if (received < static_cast<ssize_t>(sizeof(RespHeader)))
  {
    return false;
  }

  std::memcpy(&header_out, buffer, sizeof(RespHeader));
  if (header_out.version != ProtocolConstants::PROTOCOL_VERSION)
  {
    return false;
  }

  payload_len_out = header_out.length;
  if (payload_len_out > 0 && payload_out != nullptr)
  {
    uint16_t copy_len = std::min(payload_len_out, max_payload);
    std::memcpy(payload_out, buffer + sizeof(RespHeader), copy_len);
  }

  return true;
}

bool STM32Protocol::send_and_expect_ok(
  uint8_t command, const void * payload, uint16_t payload_len)
{
  if (!send_command(command, payload, payload_len))
  {
    return false;
  }

  RespHeader resp;
  uint16_t resp_len = 0;
  uint8_t resp_payload[64];
  if (!receive_response(resp, resp_payload, sizeof(resp_payload), resp_len))
  {
    return false;
  }

  if (resp.response != ProtocolConstants::RESP_OK)
  {
    if (resp.response == ProtocolConstants::RESP_ERROR && resp_len >= 32)
    {
      char msg[32] = {};
      std::memcpy(msg, resp_payload + 1, 31);
      RCLCPP_WARN(logger_, "STM32 error (code %d): %s", resp_payload[0], msg);
    }
    return false;
  }

  return true;
}

bool STM32Protocol::ping()
{
  for (int attempt = 0; attempt < ProtocolConstants::PING_MAX_ATTEMPTS; ++attempt)
  {
    if (send_and_expect_ok(ProtocolConstants::CMD_PING, nullptr, 0))
    {
      return true;
    }
    std::this_thread::sleep_for(ProtocolConstants::PING_RETRY_DELAY);
  }
  return false;
}

bool STM32Protocol::stop()
{
  return send_and_expect_ok(ProtocolConstants::CMD_STOP, nullptr, 0);
}

bool STM32Protocol::read_state(StateResponse & state_out)
{
  if (!send_command(ProtocolConstants::CMD_GET_STATE, nullptr, 0))
  {
    return false;
  }

  RespHeader resp;
  uint8_t payload[256];
  uint16_t payload_len = 0;
  if (!receive_response(resp, payload, sizeof(payload), payload_len))
  {
    return false;
  }

  if (resp.response != ProtocolConstants::RESP_STATE)
  {
    return false;
  }

  struct __attribute__((packed)) RawState
  {
    float positions[ProtocolConstants::MAX_JOINTS];
    float velocities[ProtocolConstants::MAX_JOINTS];
    uint8_t system_state;
    uint32_t trajectory_id;
    uint16_t traj_points_loaded;
    uint16_t traj_current_segment;
    uint16_t traj_total_segments;
  };

  if (payload_len < sizeof(RawState))
  {
    return false;
  }

  RawState raw;
  std::memcpy(&raw, payload, sizeof(raw));

  for (size_t i = 0; i < ProtocolConstants::MAX_JOINTS; ++i)
  {
    state_out.positions[i] = raw.positions[i];
    state_out.velocities[i] = raw.velocities[i];
  }
  state_out.system_state = static_cast<SystemState>(raw.system_state);
  state_out.trajectory_id = raw.trajectory_id;
  state_out.traj_points_loaded = raw.traj_points_loaded;
  state_out.traj_current_segment = raw.traj_current_segment;
  state_out.traj_total_segments = raw.traj_total_segments;

  return true;
}

bool STM32Protocol::trajectory_begin(uint32_t trajectory_id, uint16_t num_points)
{
  struct __attribute__((packed))
  {
    uint32_t trajectory_id;
    uint16_t num_points;
  } payload;

  payload.trajectory_id = trajectory_id;
  payload.num_points = num_points;

  return send_and_expect_ok(
    ProtocolConstants::CMD_TRAJ_BEGIN, &payload, sizeof(payload));
}

bool STM32Protocol::trajectory_send_point(
  uint32_t trajectory_id, uint16_t index,
  const TrajectoryPoint & point)
{
  struct __attribute__((packed))
  {
    uint32_t trajectory_id;
    uint16_t point_index;
    uint32_t time_from_start_ms;
    float positions[ProtocolConstants::MAX_JOINTS];
    float velocities[ProtocolConstants::MAX_JOINTS];
  } payload;

  payload.trajectory_id = trajectory_id;
  payload.point_index = index;
  payload.time_from_start_ms = point.time_from_start_ms;
  std::memcpy(payload.positions, point.positions, sizeof(payload.positions));
  std::memcpy(payload.velocities, point.velocities, sizeof(payload.velocities));

  /* No ACK for trajectory points (performance) — fire and forget.
   * Small delay prevents overwhelming the STM32 lwIP receive buffer
   * when sending many points in a tight loop from C++. */
  bool ok = send_command(
    ProtocolConstants::CMD_TRAJ_POINT, &payload, sizeof(payload));
  std::this_thread::sleep_for(std::chrono::microseconds(500));
  return ok;
}

bool STM32Protocol::trajectory_execute(uint32_t trajectory_id)
{
  struct __attribute__((packed))
  {
    uint32_t trajectory_id;
  } payload;

  payload.trajectory_id = trajectory_id;

  return send_and_expect_ok(
    ProtocolConstants::CMD_TRAJ_EXECUTE, &payload, sizeof(payload));
}

bool STM32Protocol::trajectory_cancel()
{
  return send_and_expect_ok(ProtocolConstants::CMD_TRAJ_CANCEL, nullptr, 0);
}

}  // namespace utils
}  // namespace arctos_hardware_interface
