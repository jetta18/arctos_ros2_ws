#include "arctos_motor_driver/can_protocol.hpp"
#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <sstream>
#include <iomanip>

namespace arctos_motor_driver {

CANProtocol::CANProtocol(rclcpp::Node::SharedPtr node, std::shared_ptr<SocketCANBridge> can_bridge)
    : node_(node), can_bridge_(can_bridge), frames_sent_(0), send_failures_(0) {
    if (!can_bridge_ || !can_bridge_->is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "CANProtocol initialized with an invalid or closed CAN bridge.");
        throw std::runtime_error("CAN bridge is not ready");
    }
    RCLCPP_INFO(node_->get_logger(), "CANProtocol initialized and connected to CAN bridge.");
}

CANProtocol::~CANProtocol() {
    RCLCPP_INFO(node_->get_logger(), "CANProtocol destroyed. Stats: %zu sent, %zu failures", 
                frames_sent_, send_failures_);
}

bool CANProtocol::sendCommand(uint32_t motor_id, const CANCommand& command) {
    CANFrame frame = command.toFrame(motor_id);
    return sendFrame(frame);
}

bool CANProtocol::sendFrame(const CANFrame& frame) {
    return sendFrameInternal(frame);
}

bool CANProtocol::requestEncoderReading(uint32_t motor_id) {
    CANCommand cmd(MKSCommands::READ_ENCODER);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::requestIOStatus(uint32_t motor_id) {
    CANCommand cmd(MKSCommands::READ_IO_STATUS);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::requestSpeedReading(uint32_t motor_id) {
    CANCommand cmd(MKSCommands::READ_SPEED);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::requestMotorStatus(uint32_t motor_id) {
    CANCommand cmd(MKSCommands::READ_MOTOR_STATUS);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::requestMotorType(uint32_t motor_id) {
    CANCommand cmd(MKSCommands::READ_MOTOR_TYPE);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::requestFirmwareVersion(uint32_t motor_id) {
    CANCommand cmd(MKSCommands::READ_FIRMWARE_VERSION);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::enableMotor(uint32_t motor_id, bool enable) {
    std::vector<uint8_t> payload = {enable ? uint8_t(1) : uint8_t(0)};
    CANCommand cmd(MKSCommands::ENABLE_MOTOR, payload);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setZeroPosition(uint32_t motor_id) {
    CANCommand cmd(MKSCommands::SET_ZERO_POSITION);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setWorkMode(uint32_t motor_id, uint8_t mode) {
    std::vector<uint8_t> payload = {mode};
    CANCommand cmd(MKSCommands::SET_WORK_MODE, payload);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setWorkingCurrent(uint32_t motor_id, uint16_t current_ma) {
    std::vector<uint8_t> payload = {
        static_cast<uint8_t>((current_ma >> 8) & 0xFF),  // High byte
        static_cast<uint8_t>(current_ma & 0xFF)          // Low byte
    };
    CANCommand cmd(MKSCommands::SET_WORKING_CURRENT, payload);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setHoldingCurrentPercentage(uint32_t motor_id, uint8_t percentage) {
    std::vector<uint8_t> payload = {percentage};
    CANCommand cmd(MKSCommands::SET_HOLDING_CURRENT, payload);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setSubdivision(uint32_t motor_id, uint8_t subdivision) {
    std::vector<uint8_t> payload = {subdivision};
    CANCommand cmd(MKSCommands::SET_SUBDIVISION, payload);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setEnablePinActive(uint32_t motor_id, uint8_t active_level) {
    std::vector<uint8_t> payload = {active_level};
    CANCommand cmd(MKSCommands::SET_EN_PIN_ACTIVE, payload);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setDirection(uint32_t motor_id, uint8_t direction) {
    std::vector<uint8_t> payload = {direction};
    CANCommand cmd(MKSCommands::SET_DIRECTION, payload);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setAutoScreenOff(uint32_t motor_id, bool enable) {
    std::vector<uint8_t> payload = {enable ? uint8_t(1) : uint8_t(0)};
    CANCommand cmd(MKSCommands::SET_AUTO_SCREEN_OFF, payload);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setShaftProtection(uint32_t motor_id, bool enable) {
    std::vector<uint8_t> payload = {enable ? uint8_t(1) : uint8_t(0)};
    CANCommand cmd(MKSCommands::SET_SHAFT_PROTECTION, payload);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setInterpolation(uint32_t motor_id, bool enable) {
    std::vector<uint8_t> payload = {enable ? uint8_t(1) : uint8_t(0)};
    CANCommand cmd(MKSCommands::SET_INTERPOLATION, payload);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setCANBitrate(uint32_t motor_id, uint8_t bitrate) {
    std::vector<uint8_t> payload = {bitrate};
    CANCommand cmd(MKSCommands::SET_CAN_BITRATE, payload);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setCANID(uint32_t motor_id, uint16_t new_id) {
    std::vector<uint8_t> payload = {
        static_cast<uint8_t>((new_id >> 8) & 0xFF),  // High byte
        static_cast<uint8_t>(new_id & 0xFF)          // Low byte
    };
    CANCommand cmd(MKSCommands::SET_CAN_ID, payload);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setResponseActive(uint32_t motor_id, bool response_enabled, bool active_enabled) {
    std::vector<uint8_t> payload = {
        response_enabled ? uint8_t(1) : uint8_t(0),
        active_enabled ? uint8_t(1) : uint8_t(0)
    };
    CANCommand cmd(MKSCommands::SET_RESPONSE_ACTIVE, payload);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setGroupID(uint32_t motor_id, uint16_t group_id) {
    std::vector<uint8_t> payload = {
        static_cast<uint8_t>((group_id >> 8) & 0xFF),  // High byte
        static_cast<uint8_t>(group_id & 0xFF)           // Low byte
    };
    CANCommand cmd(MKSCommands::SET_GROUP_ID, payload);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setKeyLock(uint32_t motor_id, bool lock) {
    std::vector<uint8_t> payload = {lock ? uint8_t(1) : uint8_t(0)};
    CANCommand cmd(MKSCommands::SET_KEY_LOCK, payload);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setLimitPortRemap(uint32_t motor_id, bool enable) {
    std::vector<uint8_t> payload = {enable ? uint8_t(1) : uint8_t(0)};
    CANCommand cmd(MKSCommands::SET_LIMIT_REMAP, payload);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::emergencyStop(uint32_t motor_id) {
    CANCommand cmd(MKSCommands::EMERGENCY_STOP);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::calibrateMotor(uint32_t motor_id) {
    CANCommand cmd(MKSCommands::CALIBRATE);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setHomeParameters(uint32_t motor_id, uint8_t trigger_level, uint8_t direction, uint16_t speed) {
    std::vector<uint8_t> payload = {
        trigger_level,                                   // Home trigger level
        direction,                                       // Home direction
        static_cast<uint8_t>((speed >> 8) & 0xFF),     // Speed high byte
        static_cast<uint8_t>(speed & 0xFF)              // Speed low byte
    };
    CANCommand cmd(MKSCommands::SET_HOME_PARAMETERS, payload);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::goHome(uint32_t motor_id) {
    CANCommand cmd(MKSCommands::GO_HOME);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setZeroModeParameters(uint32_t motor_id, uint8_t param) {
    std::vector<uint8_t> payload = {param};
    CANCommand cmd(MKSCommands::SET_ZERO_MODE_PARAMS, payload);
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::restoreDefaults(uint32_t motor_id) {
    CANCommand cmd(MKSCommands::RESTORE_DEFAULTS);
    return sendCommand(motor_id, cmd);
}

void CANProtocol::resetStatistics() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    frames_sent_ = 0;
    send_failures_ = 0;
}

bool CANProtocol::isReady() const {
    return can_bridge_ && can_bridge_->is_open();
}

bool CANProtocol::sendFrameInternal(const CANFrame& frame) {
    if (!isReady()) {
        RCLCPP_ERROR(node_->get_logger(), "CAN bridge is not ready, cannot send frame.");
        std::lock_guard<std::mutex> lock(stats_mutex_);
        send_failures_++;
        return false;
    }

    if (!frame.isValid()) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid CAN frame: %s", frame.toString().c_str());
        std::lock_guard<std::mutex> lock(stats_mutex_);
        send_failures_++;
        return false;
    }

    if (can_bridge_->send(frame.toRosMessage())) {
        logFrame(frame, "TX");
        std::lock_guard<std::mutex> lock(stats_mutex_);
        frames_sent_++;
        return true;
    } else {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        send_failures_++;
        return false;
    }
}

void CANProtocol::logFrame(const CANFrame& frame, const std::string& direction) const {
    RCLCPP_DEBUG(node_->get_logger(), "CAN %s: %s", direction.c_str(), frame.toString().c_str());
}

bool CANProtocol::setRelativePositionByPulses(uint32_t motor_id, uint32_t relative_pulses, uint16_t speed_rpm, uint8_t acceleration, bool clockwise) {
    // Position Mode 1 (0xFD): Relative motion by pulses
    // Format: [cmd][dir+speed_high][speed_low][acc][pulse_high][pulse_mid][pulse_low]
    
    uint8_t direction = clockwise ? 0x00 : 0x80;  // bit 7: 0=CW, 1=CCW
    
    CANCommand cmd;
    cmd.cmd = MKSCommands::POSITION_MODE_1;  // 0xFD
    cmd.payload = {
        static_cast<uint8_t>(direction | ((speed_rpm >> 8) & 0x0F)),  // Direction + speed high nibble
        static_cast<uint8_t>(speed_rpm & 0xFF),                       // Speed low byte
        acceleration,                                                  // Acceleration
        static_cast<uint8_t>((relative_pulses >> 16) & 0xFF),        // Pulses high byte
        static_cast<uint8_t>((relative_pulses >> 8) & 0xFF),         // Pulses middle byte
        static_cast<uint8_t>(relative_pulses & 0xFF)                 // Pulses low byte
    };
    
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setAbsolutePositionByPulses(uint32_t motor_id, int32_t absolute_pulses, uint16_t speed_rpm, uint8_t acceleration) {
    // Position Mode 2 (0xFE): Absolute motion by pulses
    // Format: [cmd][speed_high][speed_low][acc][pulse_high][pulse_mid][pulse_low]
    
    CANCommand cmd;
    cmd.cmd = 0xFE;  // Position Mode 2
    cmd.payload = {
        static_cast<uint8_t>((speed_rpm >> 8) & 0xFF),               // Speed high byte
        static_cast<uint8_t>(speed_rpm & 0xFF),                      // Speed low byte
        acceleration,                                                 // Acceleration
        static_cast<uint8_t>((absolute_pulses >> 16) & 0xFF),       // Pulses high byte (signed)
        static_cast<uint8_t>((absolute_pulses >> 8) & 0xFF),        // Pulses middle byte
        static_cast<uint8_t>(absolute_pulses & 0xFF)                // Pulses low byte
    };
    
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setRelativePositionByAxis(uint32_t motor_id, int32_t relative_axis, uint16_t speed_rpm, uint8_t acceleration) {
    // Position Mode 3 (0xF4): Relative motion by encoder axis
    // Format: [cmd][speed_high][speed_low][acc][axis_high][axis_mid][axis_low]
    
    CANCommand cmd;
    cmd.cmd = 0xF4;  // Position Mode 3
    cmd.payload = {
        static_cast<uint8_t>((speed_rpm >> 8) & 0xFF),              // Speed high byte
        static_cast<uint8_t>(speed_rpm & 0xFF),                     // Speed low byte
        acceleration,                                                // Acceleration
        static_cast<uint8_t>((relative_axis >> 16) & 0xFF),        // Axis high byte (signed)
        static_cast<uint8_t>((relative_axis >> 8) & 0xFF),         // Axis middle byte
        static_cast<uint8_t>(relative_axis & 0xFF)                 // Axis low byte
    };
    
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setAbsolutePositionByAxis(uint32_t motor_id, int32_t absolute_axis, uint16_t speed_rpm, uint8_t acceleration) {
    // Position Mode 4 (0xF5): Absolute motion by encoder axis
    // Format: [cmd][speed_high][speed_low][acc][axis_high][axis_mid][axis_low]
    
    CANCommand cmd;
    cmd.cmd = 0xF5;  // Position Mode 4
    cmd.payload = {
        static_cast<uint8_t>((speed_rpm >> 8) & 0xFF),              // Speed high byte
        static_cast<uint8_t>(speed_rpm & 0xFF),                     // Speed low byte
        acceleration,                                                // Acceleration
        static_cast<uint8_t>((absolute_axis >> 16) & 0xFF),        // Axis high byte (signed)
        static_cast<uint8_t>((absolute_axis >> 8) & 0xFF),         // Axis middle byte
        static_cast<uint8_t>(absolute_axis & 0xFF)                 // Axis low byte
    };
    
    return sendCommand(motor_id, cmd);
}

bool CANProtocol::setVelocity(uint32_t motor_id, int16_t speed_rpm, uint8_t acceleration) {
    // MKS velocity control command format (0xF6):
    // Byte 0: Command (0xF6)
    // Byte 1: Direction + Speed high nibble
    // Byte 2: Speed low byte
    // Byte 3: Acceleration
    
    uint8_t direction = (speed_rpm >= 0) ? 0x00 : 0x80;  // 0x00 = forward, 0x80 = reverse
    uint16_t abs_speed = static_cast<uint16_t>(std::abs(speed_rpm));
    
    CANCommand cmd;
    cmd.cmd = MKSCommands::SPEED_MODE;  // 0xF6
    cmd.payload = {
        static_cast<uint8_t>(direction | ((abs_speed >> 8) & 0x0F)),  // Direction + high nibble
        static_cast<uint8_t>(abs_speed & 0xFF),                       // Speed low byte
        acceleration                                                   // Acceleration
    };
    
    return sendCommand(motor_id, cmd);
}

} // namespace arctos_motor_driver
