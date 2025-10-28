#pragma once

#include <cstdint>

namespace mks
{

enum class Direction : uint8_t {
  CCW = 0,
  CW = 1
};

// Work modes as per manual (Menu->Mode)
enum class WorkMode : uint8_t {
  CR_OPEN  = 0,
  CR_CLOSE = 1,
  CR_vFOC  = 2,
  SR_OPEN  = 3,
  SR_CLOSE = 4,
  SR_vFOC  = 5,
};

// Status codes for F1 query
enum class MotorStatus : uint8_t {
  QueryFail     = 0,
  Stopped       = 1,
  SpeedUp       = 2,
  SpeedDown     = 3,
  FullSpeed     = 4,
  Homing        = 5,
  Calibrating   = 6,
};

// Generic command result: 0 fail, 1 success (per many commands)
enum class Result : uint8_t {
  Fail    = 0,
  Success = 1
};

struct SpeedAcc {
  uint16_t speed = 0;  // 0..3000 RPM
  uint8_t acc = 0;     // 0..255
};

} // namespace mks
