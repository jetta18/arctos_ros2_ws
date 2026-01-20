#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <optional>

namespace mks {

struct CanFrame {
  uint32_t id = 0;     // 11-bit standard frame id in lower bits
  uint8_t dlc = 0;     // 0..8
  uint8_t data[8]{};   // payload
};

class ICanTransport {
public:
  virtual ~ICanTransport() = default;

  // Open underlying interface (e.g., can0)
  virtual bool open(const std::string& ifname) = 0;

  // Close interface
  virtual void close() = 0;

  // Write a single CAN frame
  virtual bool write(const CanFrame& frame) = 0;

  // Read a single CAN frame with timeout_ms. If timeout expires, return std::nullopt
  virtual std::optional<CanFrame> read(int timeout_ms) = 0;
};

// Linux SocketCAN transport implementation
class SocketCanTransport : public ICanTransport {
public:
  SocketCanTransport();
  ~SocketCanTransport() override;

  bool open(const std::string& ifname) override;
  void close() override;
  bool write(const CanFrame& frame) override;
  std::optional<CanFrame> read(int timeout_ms) override;

private:
  int sock_;
};

} // namespace mks
