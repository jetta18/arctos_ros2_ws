#ifndef ARCTOS_MOTOR_DRIVER_MKS_DATA_TYPES_HPP_
#define ARCTOS_MOTOR_DRIVER_MKS_DATA_TYPES_HPP_

#include "can_frame.hpp"
#include <rclcpp/rclcpp.hpp>

namespace arctos_motor_driver {

/**
 * @brief Encoder data extracted from MKS Servo 0x31 response
 * 
 * Based on MKS manual: 8-byte response [CMD][6 encoder bytes][CRC]
 * Encoder value is 48-bit signed integer representing absolute position.
 */
struct EncoderData {
    int64_t raw_value;          ///< Raw 48-bit encoder value
    double angle_degrees;       ///< Converted angle in degrees
    double angle_radians;       ///< Converted angle in radians
    bool is_valid;              ///< Data validity flag
    rclcpp::Time timestamp;     ///< When data was received
    
    /**
     * @brief Default constructor - invalid data
     */
    EncoderData();
    
    /**
     * @brief Extract encoder data from CAN frame
     * @param frame CAN frame with 0x31 response
     * @return EncoderData object
     */
    static EncoderData fromCANFrame(const CANFrame& frame);
    
    /**
     * @brief Convert raw encoder value to degrees
     * @param raw_value 48-bit signed encoder value
     * @return Angle in degrees
     */
    static double rawToDegrees(int64_t raw_value);
    
    /**
     * @brief Convert raw encoder value to radians
     * @param raw_value 48-bit signed encoder value
     * @return Angle in radians
     */
    static double rawToRadians(int64_t raw_value);
    
    /**
     * @brief Apply gear ratio and inversion to get joint angle
     * @param gear_ratio Motor gear ratio
     * @param inverted Whether motor direction is inverted
     * @return Joint angle in radians
     */
    double toJointAngle(double gear_ratio, bool inverted) const;
    
    /**
     * @brief Debug string representation
     * @return Human-readable description
     */
    std::string toString() const;
};

/**
 * @brief IO status data extracted from MKS Servo 0x34 response
 * 
 * Based on MKS manual: 3-byte response [CMD][status][CRC]
 * Status byte: [reserved][reserved][reserved][reserved][OUT_2][OUT_1][IN_2][IN_1]
 */
struct IOStatus {
    bool in1;                   ///< IN_1 port (home/left-limit)
    bool in2;                   ///< IN_2 port (right-limit, 57D only)
    bool out1;                  ///< OUT_1 port (stall indication)
    bool out2;                  ///< OUT_2 port (reserved)
    bool is_valid;              ///< Data validity flag
    rclcpp::Time timestamp;     ///< When data was received
    
    // Convenience accessors for limit switches
    bool limit_left;            ///< Left limit switch state
    bool limit_right;           ///< Right limit switch state
    bool stall_detected;        ///< Motor stall detection
    
    /**
     * @brief Default constructor - invalid data
     */
    IOStatus();
    
    /**
     * @brief Extract IO status from CAN frame
     * @param frame CAN frame with 0x34 response
     * @param hardware_type Motor type ("MKS_42D" or "MKS_57D")
     * @param limit_remap_enabled Whether limit port remapping is enabled
     * @return IOStatus object
     */
    static IOStatus fromCANFrame(const CANFrame& frame, 
                                const std::string& hardware_type = "MKS_42D",
                                bool limit_remap_enabled = false);
    
    /**
     * @brief Debug string representation
     * @return Human-readable description
     */
    std::string toString() const;
};

/**
 * @brief Motor speed data extracted from MKS Servo 0x32 response
 * 
 * Based on MKS manual: 4-byte response [CMD][speed(2bytes)][CRC]
 * Speed is int16_t in RPM (positive=CCW, negative=CW)
 */
struct SpeedData {
    int16_t rpm;                ///< Motor speed in RPM
    double rad_per_sec;         ///< Speed in rad/s
    bool is_valid;              ///< Data validity flag
    rclcpp::Time timestamp;     ///< When data was received
    
    /**
     * @brief Default constructor - invalid data
     */
    SpeedData();
    
    /**
     * @brief Extract speed data from CAN frame
     * @param frame CAN frame with 0x32 response
     * @return SpeedData object
     */
    static SpeedData fromCANFrame(const CANFrame& frame);
    
    /**
     * @brief Convert RPM to rad/s
     * @param rpm Speed in RPM
     * @return Speed in rad/s
     */
    static double rpmToRadPerSec(int16_t rpm);
    
    /**
     * @brief Apply gear ratio and inversion to get joint velocity
     * @param gear_ratio Motor gear ratio
     * @param inverted Whether motor direction is inverted
     * @return Joint velocity in rad/s
     */
    double toJointVelocity(double gear_ratio, bool inverted) const;
    
    /**
     * @brief Debug string representation
     * @return Human-readable description
     */
    std::string toString() const;
};

/**
 * @brief Motor status information
 */
struct MKSMotorStatus {
    bool enabled;               ///< Motor enable state (from 0x3A or 0xF3)
    bool moving;                ///< Motor is currently moving
    bool calibrated;            ///< Motor has been calibrated
    bool error;                 ///< Motor error state
    uint8_t status_code;        ///< Raw status code from 0xF1
    bool is_valid;              ///< Data validity flag
    rclcpp::Time timestamp;     ///< When data was received
    
    /**
     * @brief Default constructor - invalid data
     */
    MKSMotorStatus();
    
    /**
     * @brief Extract status from enable response (0xF3 or 0x3A)
     * @param frame CAN frame with enable status response
     * @return MKSMotorStatus object
     */
    static MKSMotorStatus fromEnableResponse(const CANFrame& frame);
    
    /**
     * @brief Extract status from query response (0xF1)
     * @param frame CAN frame with status query response
     * @return MotorStatus object
     */
    static MKSMotorStatus fromQueryResponse(const CANFrame& frame);
    
    /**
     * @brief Get human-readable status description
     * @return Status description string
     */
    std::string getStatusDescription() const;
    
    /**
     * @brief Debug string representation
     * @return Human-readable description
     */
    std::string toString() const;
};

/**
 * @brief Helper functions for MKS data processing
 */
namespace MKSDataUtils {
    
    /**
     * @brief Extract 48-bit signed value from byte array
     * @param bytes Pointer to 6-byte array
     * @return 48-bit signed value with proper sign extension
     */
    int64_t extract48BitSigned(const uint8_t* bytes);
    
    /**
     * @brief Extract 16-bit signed value from byte array (big-endian)
     * @param bytes Pointer to 2-byte array
     * @return 16-bit signed value
     */
    int16_t extract16BitSigned(const uint8_t* bytes);
    
    /**
     * @brief Extract 32-bit signed value from byte array (big-endian)
     * @param bytes Pointer to 4-byte array
     * @return 32-bit signed value
     */
    int32_t extract32BitSigned(const uint8_t* bytes);
    
    /**
     * @brief Validate CAN frame for specific command
     * @param frame CAN frame to validate
     * @param expected_cmd Expected command byte
     * @param expected_dlc Expected data length
     * @return true if frame is valid for the command
     */
    bool validateFrameForCommand(const CANFrame& frame, uint8_t expected_cmd, uint8_t expected_dlc);
    
    /**
     * @brief Convert motor angle to joint angle with gear ratio and inversion
     * @param motor_angle_rad Motor angle in radians
     * @param gear_ratio Gear ratio (motor_revs / joint_revs)
     * @param inverted Whether direction is inverted
     * @return Joint angle in radians
     */
    double motorToJointAngle(double motor_angle_rad, double gear_ratio, bool inverted);
    
    /**
     * @brief Convert motor velocity to joint velocity with gear ratio and inversion
     * @param motor_velocity_rad_s Motor velocity in rad/s
     * @param gear_ratio Gear ratio (motor_revs / joint_revs)
     * @param inverted Whether direction is inverted
     * @return Joint velocity in rad/s
     */
    double motorToJointVelocity(double motor_velocity_rad_s, double gear_ratio, bool inverted);
}

} // namespace arctos_motor_driver

#endif // ARCTOS_MOTOR_DRIVER_MKS_DATA_TYPES_HPP_
