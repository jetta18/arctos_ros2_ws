#ifndef ARCTOS_HARDWARE_INTERFACE__UTILS__STM32_PROTOCOL_HPP_
#define ARCTOS_HARDWARE_INTERFACE__UTILS__STM32_PROTOCOL_HPP_

#include <cstdint>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"

namespace arctos_hardware_interface
{
namespace utils
{

struct ProtocolConstants
{
  static constexpr uint8_t PROTOCOL_VERSION = 2;
  static constexpr size_t MAX_JOINTS = 6;
  static constexpr size_t MAX_PACKET_SIZE = 256;

  static constexpr uint8_t CMD_MOVE_AXIS    = 0x01;
  static constexpr uint8_t CMD_MOVE_ALL     = 0x02;
  static constexpr uint8_t CMD_STOP         = 0x03;
  static constexpr uint8_t CMD_SET_POSITION = 0x04;
  static constexpr uint8_t CMD_GET_STATE    = 0x10;
  static constexpr uint8_t CMD_GET_ENDSTOPS = 0x11;
  static constexpr uint8_t CMD_PING         = 0x20;
  static constexpr uint8_t CMD_TRAJ_BEGIN   = 0x40;
  static constexpr uint8_t CMD_TRAJ_POINT   = 0x41;
  static constexpr uint8_t CMD_TRAJ_EXECUTE = 0x42;
  static constexpr uint8_t CMD_TRAJ_CANCEL  = 0x43;

  static constexpr uint8_t RESP_OK       = 0x00;
  static constexpr uint8_t RESP_ERROR    = 0x01;
  static constexpr uint8_t RESP_STATE    = 0x02;
  static constexpr uint8_t RESP_ENDSTOPS = 0x03;

  static constexpr int PING_MAX_ATTEMPTS = 5;
  static constexpr std::chrono::milliseconds PING_RETRY_DELAY{200};
};

enum class SystemState : uint8_t
{
  IDLE = 0,
  MOVING = 1,
  TRAJ_LOADING = 2,
  TRAJ_RUNNING = 3,
  TRAJ_COMPLETE = 4,
  ERROR = 5,
  STOPPING = 6,
};

struct StateResponse
{
  float positions[ProtocolConstants::MAX_JOINTS];
  float velocities[ProtocolConstants::MAX_JOINTS];
  SystemState system_state;
  uint32_t trajectory_id;
  uint16_t traj_points_loaded;
  uint16_t traj_current_segment;
  uint16_t traj_total_segments;
};

struct TrajectoryPoint
{
  uint32_t time_from_start_ms;
  float positions[ProtocolConstants::MAX_JOINTS];
  float velocities[ProtocolConstants::MAX_JOINTS];
};

class STM32Protocol
{
public:
  STM32Protocol(int socket_fd, rclcpp::Logger & logger);

  bool ping();
  bool stop();
  bool read_state(StateResponse & state_out);

  bool trajectory_begin(uint32_t trajectory_id, uint16_t num_points);
  bool trajectory_send_point(
    uint32_t trajectory_id, uint16_t index,
    const TrajectoryPoint & point);
  bool trajectory_execute(uint32_t trajectory_id);
  bool trajectory_cancel();

private:
  struct __attribute__((packed)) Header
  {
    uint8_t version;
    uint8_t command;
    uint16_t length;
    uint32_t sequence;
  };

  struct __attribute__((packed)) RespHeader
  {
    uint8_t version;
    uint8_t response;
    uint16_t length;
    uint32_t sequence;
  };

  bool send_command(uint8_t command, const void * payload, uint16_t payload_len);
  bool receive_response(RespHeader & header_out, uint8_t * payload_out,
                        uint16_t max_payload, uint16_t & payload_len_out);
  bool send_and_expect_ok(uint8_t command, const void * payload, uint16_t payload_len);

  int socket_fd_;
  rclcpp::Logger logger_;
  uint32_t sequence_number_;
};

}  // namespace utils
}  // namespace arctos_hardware_interface

#endif  // ARCTOS_HARDWARE_INTERFACE__UTILS__STM32_PROTOCOL_HPP_
