#ifndef ARCTOS_HARDWARE_INTERFACE__CONSTANTS_HPP_
#define ARCTOS_HARDWARE_INTERFACE__CONSTANTS_HPP_

namespace arctos_hardware_interface {
namespace constants {
inline constexpr int kNumJoints = 6;

/// @brief Milliseconds to wait after bringing up the CAN stack before issuing commands.
inline constexpr int kCanInitDelayMs = 200;

/// @brief Pause between individual motor configuration commands to avoid CAN congestion.
inline constexpr int kBetweenMotorConfigMs = 10;

/// @brief Settling time once all motor configuration commands have been sent.
inline constexpr int kAfterConfigSettleMs = 150;

/// @brief Delay before the very first encoder read to ensure drivers are ready.
inline constexpr int kPreReadSettleMs = 200;

/// @brief Pause between motor enable commands when activating the hardware interface.
inline constexpr int kBetweenEnableMs = 10;

/// @brief Timeout used when waiting for encoder responses during activation.
inline constexpr int kInitialEncoderTimeoutMs = 100;

/// @brief Delay between encoder request frames to keep the CAN bus responsive.
inline constexpr int kBetweenEncoderRequestsMs = 5;

/// @brief Sleep duration between polling iterations while waiting for encoder data.
inline constexpr int kEncoderLoopSleepMs = 10;

/// @brief Maximum number of attempts made to retrieve encoder data during startup.
inline constexpr int kEncoderMaxRetries = 3;

/// @brief Maximum allowed age (seconds) for encoder data to be considered valid.
inline constexpr double kEncoderValidMaxAgeSec = 0.5;

} // namespace constants
} // namespace arctos_hardware_interface

#endif
