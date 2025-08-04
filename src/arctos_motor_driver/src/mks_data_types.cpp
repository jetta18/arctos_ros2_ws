#include "arctos_motor_driver/mks_data_types.hpp"
#include <sstream>
#include <iomanip>
#include <cmath>

namespace arctos_motor_driver {

// EncoderData Implementation

EncoderData::EncoderData() 
    : raw_value(0), angle_degrees(0.0), angle_radians(0.0), is_valid(false) {}

EncoderData EncoderData::fromCANFrame(const CANFrame& frame) {
    EncoderData data;
    data.timestamp = rclcpp::Clock().now();
    
    // Validate frame for 0x31 command (READ_ENCODER)
    if (!MKSDataUtils::validateFrameForCommand(frame, MKSCommands::READ_ENCODER, 8)) {
        data.is_valid = false;
        return data;
    }
    
    // Extract 48-bit encoder value from bytes 1-6 (skip command byte 0, skip CRC byte 7)
    // Based on MKS manual: [CMD][6 encoder bytes][CRC]
    data.raw_value = MKSDataUtils::extract48BitSigned(&frame.data[1]);
    
    // Convert to angles
    data.angle_degrees = rawToDegrees(data.raw_value);
    data.angle_radians = rawToRadians(data.raw_value);
    
    // Sanity check - reject obviously invalid values
    if (std::isfinite(data.angle_degrees) && std::abs(data.angle_degrees) < 1e6) {
        data.is_valid = true;
    } else {
        data.is_valid = false;
    }
    
    return data;
}

double EncoderData::rawToDegrees(int64_t raw_value) {
    // One revolution = 0x4000 steps = 16384 steps
    // 360 degrees per revolution
    return static_cast<double>(raw_value) * MKSConstants::DEGREES_PER_STEP;
}

double EncoderData::rawToRadians(int64_t raw_value) {
    return rawToDegrees(raw_value) * MKSConstants::DEG_TO_RAD;
}

double EncoderData::toJointAngle(double gear_ratio, bool inverted) const {
    return MKSDataUtils::motorToJointAngle(angle_radians, gear_ratio, inverted);
}

std::string EncoderData::toString() const {
    std::ostringstream oss;
    oss << "EncoderData[valid:" << (is_valid ? "YES" : "NO")
        << " raw:0x" << std::hex << raw_value
        << " deg:" << std::fixed << std::setprecision(3) << angle_degrees
        << " rad:" << std::fixed << std::setprecision(6) << angle_radians << "]";
    return oss.str();
}

// IOStatus Implementation

IOStatus::IOStatus() 
    : in1(false), in2(false), out1(false), out2(false), is_valid(false),
      limit_left(false), limit_right(false), stall_detected(false) {}

IOStatus IOStatus::fromCANFrame(const CANFrame& frame, const std::string& hardware_type, bool limit_remap_enabled) {
    IOStatus status;
    status.timestamp = rclcpp::Clock().now();
    
    // Validate frame for 0x34 command (READ_IO_STATUS)
    if (!MKSDataUtils::validateFrameForCommand(frame, MKSCommands::READ_IO_STATUS, 3)) {
        status.is_valid = false;
        return status;
    }
    
    // Extract status byte (byte 1, skip command byte 0 and CRC byte 2)
    uint8_t status_byte = frame.data[1];
    
    // Parse individual bits: [reserved][reserved][reserved][reserved][OUT_2][OUT_1][IN_2][IN_1]
    status.in1 = (status_byte & 0x01) != 0;
    status.in2 = (status_byte & 0x02) != 0;
    status.out1 = (status_byte & 0x04) != 0;
    status.out2 = (status_byte & 0x08) != 0;
    
    // Map to limit switches based on hardware type and remap setting
    if (limit_remap_enabled) {
        // After limit remap: IN_1 → En, IN_2 → Dir
        // This changes the meaning of the inputs
        status.limit_left = false;   // Not available in remap mode
        status.limit_right = false;  // Not available in remap mode
    } else {
        // Normal mode: IN_1 = left/home limit, IN_2 = right limit (57D only)
        status.limit_left = status.in1;
        
        if (hardware_type == "MKS_57D") {
            status.limit_right = status.in2;
        } else {
            status.limit_right = false;  // 42D has only one limit input
        }
    }
    
    // OUT_1 typically indicates stall protection
    status.stall_detected = status.out1;
    
    status.is_valid = true;
    return status;
}

std::string IOStatus::toString() const {
    std::ostringstream oss;
    oss << "IOStatus[valid:" << (is_valid ? "YES" : "NO")
        << " IN1:" << (in1 ? "1" : "0")
        << " IN2:" << (in2 ? "1" : "0") 
        << " OUT1:" << (out1 ? "1" : "0")
        << " OUT2:" << (out2 ? "1" : "0")
        << " L_limit:" << (limit_left ? "TRIG" : "OK")
        << " R_limit:" << (limit_right ? "TRIG" : "OK")
        << " stall:" << (stall_detected ? "YES" : "NO") << "]";
    return oss.str();
}

// SpeedData Implementation

SpeedData::SpeedData() 
    : rpm(0), rad_per_sec(0.0), is_valid(false) {}

SpeedData SpeedData::fromCANFrame(const CANFrame& frame) {
    SpeedData data;
    data.timestamp = rclcpp::Clock().now();
    
    // Validate frame for 0x32 command (READ_SPEED)
    if (!MKSDataUtils::validateFrameForCommand(frame, MKSCommands::READ_SPEED, 4)) {
        data.is_valid = false;
        return data;
    }
    
    // Extract 16-bit speed from bytes 1-2 (skip command byte 0, skip CRC byte 3)
    data.rpm = MKSDataUtils::extract16BitSigned(&frame.data[1]);
    data.rad_per_sec = rpmToRadPerSec(data.rpm);
    
    data.is_valid = true;
    return data;
}

double SpeedData::rpmToRadPerSec(int16_t rpm) {
    // Convert RPM to rad/s: rad/s = RPM * 2π / 60
    return static_cast<double>(rpm) * 2.0 * M_PI / 60.0;
}

double SpeedData::toJointVelocity(double gear_ratio, bool inverted) const {
    return MKSDataUtils::motorToJointVelocity(rad_per_sec, gear_ratio, inverted);
}

std::string SpeedData::toString() const {
    std::ostringstream oss;
    oss << "SpeedData[valid:" << (is_valid ? "YES" : "NO")
        << " rpm:" << rpm
        << " rad/s:" << std::fixed << std::setprecision(3) << rad_per_sec << "]";
    return oss.str();
}

// MKSMotorStatus Implementation

MKSMotorStatus::MKSMotorStatus() 
    : enabled(false), moving(false), calibrated(false), error(false), 
      status_code(0), is_valid(false) {}

MKSMotorStatus MKSMotorStatus::fromEnableResponse(const CANFrame& frame) {
    MKSMotorStatus status;
    status.timestamp = rclcpp::Clock().now();
    
    // Can be 0xF3 (ENABLE_MOTOR) or 0x3A (READ_ENABLE_STATUS)
    uint8_t cmd = frame.getCommand();
    if (cmd != MKSCommands::ENABLE_MOTOR && cmd != MKSCommands::READ_ENABLE_STATUS) {
        status.is_valid = false;
        return status;
    }
    
    if (frame.dlc >= 3) {
        uint8_t enable_byte = frame.data[1];
        status.enabled = (enable_byte == 1);
        status.is_valid = true;
    }
    
    return status;
}

MKSMotorStatus MKSMotorStatus::fromQueryResponse(const CANFrame& frame) {
    MKSMotorStatus status;
    status.timestamp = rclcpp::Clock().now();
    
    // Validate frame for 0xF1 command (QUERY_STATUS)
    if (!MKSDataUtils::validateFrameForCommand(frame, MKSCommands::QUERY_STATUS, 3)) {
        status.is_valid = false;
        return status;
    }
    
    status.status_code = frame.data[1];
    
    // Decode status according to MKS manual
    switch (status.status_code) {
        case 0: status.error = true; break;
        case 1: status.moving = false; break;  // stopped
        case 2: case 3: case 4: status.moving = true; break;  // accelerating/decelerating/full speed
        case 5: status.moving = true; break;   // homing
        case 6: status.calibrated = false; break;  // calibrating
        default: break;
    }
    
    status.is_valid = true;
    return status;
}

std::string MKSMotorStatus::getStatusDescription() const {
    switch (status_code) {
        case 0: return "Query failed";
        case 1: return "Motor stopped";
        case 2: return "Motor accelerating";
        case 3: return "Motor decelerating";
        case 4: return "Motor at full speed";
        case 5: return "Motor homing";
        case 6: return "Motor calibrating";
        default: return "Unknown status";
    }
}

std::string MKSMotorStatus::toString() const {
    std::ostringstream oss;
    oss << "MKSMotorStatus[valid:" << (is_valid ? "YES" : "NO")
        << " enabled:" << (enabled ? "YES" : "NO")
        << " moving:" << (moving ? "YES" : "NO")
        << " calibrated:" << (calibrated ? "YES" : "NO")
        << " error:" << (error ? "YES" : "NO")
        << " code:" << static_cast<int>(status_code)
        << " desc:\"" << getStatusDescription() << "\"]";
    return oss.str();
}

// MKSDataUtils Implementation

namespace MKSDataUtils {

int64_t extract48BitSigned(const uint8_t* bytes) {
    // Extract 48-bit value from 6 bytes (big-endian)
    int64_t value = 0;
    for (int i = 0; i < 6; ++i) {
        value = (value << 8) | bytes[i];
    }
    
    // Sign extend from 48-bit to 64-bit
    if (value & 0x800000000000LL) {
        value |= 0xFFFF000000000000LL;
    }
    
    return value;
}

int16_t extract16BitSigned(const uint8_t* bytes) {
    // Extract 16-bit value (big-endian)
    return static_cast<int16_t>((bytes[0] << 8) | bytes[1]);
}

int32_t extract32BitSigned(const uint8_t* bytes) {
    // Extract 32-bit value (big-endian)
    return static_cast<int32_t>((bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3]);
}

bool validateFrameForCommand(const CANFrame& frame, uint8_t expected_cmd, uint8_t expected_dlc) {
    // Check basic frame validity
    if (!frame.isValid()) {
        return false;
    }
    
    // Check DLC
    if (frame.dlc != expected_dlc) {
        return false;
    }
    
    // Check command byte
    if (frame.getCommand() != expected_cmd) {
        return false;
    }
    
    return true;
}

double motorToJointAngle(double motor_angle_rad, double gear_ratio, bool inverted) {
    double joint_angle = motor_angle_rad / gear_ratio;
    return inverted ? -joint_angle : joint_angle;
}

double motorToJointVelocity(double motor_velocity_rad_s, double gear_ratio, bool inverted) {
    double joint_velocity = motor_velocity_rad_s / gear_ratio;
    return inverted ? -joint_velocity : joint_velocity;
}

} // namespace MKSDataUtils

} // namespace arctos_motor_driver
