#ifndef ARCTOS_MOTOR_DRIVER_CAN_PROTOCOL_HPP_
#define ARCTOS_MOTOR_DRIVER_CAN_PROTOCOL_HPP_

#include "can_frame.hpp"
#include "mks_data_types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "arctos_motor_driver/socket_can_bridge.hpp"
#include <memory>
#include <mutex>

namespace arctos_motor_driver {

/**
 * @brief Clean, focused CAN protocol implementation for MKS Servo motors
 * 
 * Provides essential CAN communication functions with proper error handling
 * and response validation. Focuses on reliability over feature completeness.
 */
class CANProtocol {
public:
    /**
     * @brief Constructor
     * @param node ROS2 node for communication
     */
    explicit CANProtocol(rclcpp::Node::SharedPtr node, std::shared_ptr<SocketCANBridge> can_bridge);
    
    /**
     * @brief Destructor
     */
    ~CANProtocol();

    // Core communication methods
    
    /**
     * @brief Send a CAN command to a motor
     * @param motor_id Target motor CAN ID
     * @param command Command to send
     * @return true if sent successfully
     */
    bool sendCommand(uint32_t motor_id, const CANCommand& command);
    
    /**
     * @brief Send raw CAN frame
     * @param frame CAN frame to send
     * @return true if sent successfully
     */
    bool sendFrame(const CANFrame& frame);
    
    // Essential motor commands
    
    /**
     * @brief Request encoder reading from motor
     * @param motor_id Target motor CAN ID
     * @return true if request sent
     */
    bool requestEncoderReading(uint32_t motor_id);
    
    /**
     * @brief Request IO status from motor
     * @param motor_id Target motor CAN ID
     * @return true if request sent
     */
    bool requestIOStatus(uint32_t motor_id);
    
    /**
     * @brief Request motor speed reading
     * @param motor_id Target motor CAN ID
     * @return true if request sent
     */
    bool requestSpeedReading(uint32_t motor_id);
    
    /**
     * @brief Request detailed motor status
     * @param motor_id Target motor CAN ID
     * @return true if request sent
     */
    bool requestMotorStatus(uint32_t motor_id);
    
    /**
     * @brief Request motor type information
     * @param motor_id Target motor CAN ID
     * @return true if request sent
     */
    bool requestMotorType(uint32_t motor_id);
    
    /**
     * @brief Request firmware version
     * @param motor_id Target motor CAN ID
     * @return true if request sent
     */
    bool requestFirmwareVersion(uint32_t motor_id);
    
    /**
     * @brief Enable or disable motor
     * @param motor_id Target motor CAN ID
     * @param enable true to enable, false to disable
     * @return true if command sent
     */
    bool enableMotor(uint32_t motor_id, bool enable);
    
    /**
     * @brief Set current position as zero reference
     * @param motor_id Target motor CAN ID
     * @return true if command sent
     */
    bool setZeroPosition(uint32_t motor_id);
    
    /**
     * @brief Set motor work mode
     * @param motor_id Target motor CAN ID
     * @param mode Work mode (use MKSWorkModes constants)
     * @return true if command sent
     */
    bool setWorkMode(uint32_t motor_id, uint8_t mode);
    
    /**
     * @brief Set working current
     * @param motor_id Target motor CAN ID
     * @param current_ma Current in milliamps
     * @return true if command sent
     */
    bool setWorkingCurrent(uint32_t motor_id, uint16_t current_ma);
    
    /**
     * @brief Set holding current percentage
     * @param motor_id Target motor CAN ID
     * @param percentage Holding current percentage (0-8, see MKSConfig constants)
     * @return true if command sent
     */
    bool setHoldingCurrentPercentage(uint32_t motor_id, uint8_t percentage);
    
    /**
     * @brief Set microstepping subdivision
     * @param motor_id Target motor CAN ID
     * @param subdivision Subdivision value (0x00-0xFF)
     * @return true if command sent
     */
    bool setSubdivision(uint32_t motor_id, uint8_t subdivision);
    
    /**
     * @brief Set enable pin active level
     * @param motor_id Target motor CAN ID
     * @param active_level Active level (see MKSConfig::EN_PIN_* constants)
     * @return true if command sent
     */
    bool setEnablePinActive(uint32_t motor_id, uint8_t active_level);
    
    /**
     * @brief Set motor direction for pulse interface
     * @param motor_id Target motor CAN ID
     * @param direction Direction (see MKSConfig::DIRECTION_* constants)
     * @return true if command sent
     */
    bool setDirection(uint32_t motor_id, uint8_t direction);
    
    /**
     * @brief Set auto screen off function
     * @param motor_id Target motor CAN ID
     * @param enable true to enable auto screen off
     * @return true if command sent
     */
    bool setAutoScreenOff(uint32_t motor_id, bool enable);
    
    /**
     * @brief Set shaft protection (locked-rotor protection)
     * @param motor_id Target motor CAN ID
     * @param enable true to enable protection
     * @return true if command sent
     */
    bool setShaftProtection(uint32_t motor_id, bool enable);
    
    /**
     * @brief Set subdivision interpolation
     * @param motor_id Target motor CAN ID
     * @param enable true to enable interpolation
     * @return true if command sent
     */
    bool setInterpolation(uint32_t motor_id, bool enable);
    
    /**
     * @brief Set CAN bitrate
     * @param motor_id Target motor CAN ID
     * @param bitrate Bitrate (see MKSConfig::CAN_BITRATE_* constants)
     * @return true if command sent
     */
    bool setCANBitrate(uint32_t motor_id, uint8_t bitrate);
    
    /**
     * @brief Set CAN ID
     * @param motor_id Target motor CAN ID
     * @param new_id New CAN ID (0x01-0x7FF)
     * @return true if command sent
     */
    bool setCANID(uint32_t motor_id, uint16_t new_id);
    
    /**
     * @brief Set response and active mode
     * @param motor_id Target motor CAN ID
     * @param response_enabled true to enable response
     * @param active_enabled true to enable active mode
     * @return true if command sent
     */
    bool setResponseActive(uint32_t motor_id, bool response_enabled, bool active_enabled);
    
    /**
     * @brief Set group ID for multi-motor control
     * @param motor_id Target motor CAN ID
     * @param group_id Group ID (0x01-0x7FF)
     * @return true if command sent
     */
    bool setGroupID(uint32_t motor_id, uint16_t group_id);
    
    /**
     * @brief Lock/unlock keypad
     * @param motor_id Target motor CAN ID
     * @param lock true to lock keypad
     * @return true if command sent
     */
    bool setKeyLock(uint32_t motor_id, bool lock);
    
    /**
     * @brief Enable/disable limit port remapping
     * @param motor_id Target motor CAN ID
     * @param enable true to enable remap
     * @return true if command sent
     */
    bool setLimitPortRemap(uint32_t motor_id, bool enable);
    
    /**
     * @brief Emergency stop motor
     * @param motor_id Target motor CAN ID
     * @return true if command sent
     */
    bool emergencyStop(uint32_t motor_id);
    
    /**
     * @brief Calibrate motor (motor must be unloaded)
     * @param motor_id Target motor CAN ID
     * @return true if command sent
     */
    bool calibrateMotor(uint32_t motor_id);
    
    /**
     * @brief Set homing parameters
     * @param motor_id Target motor CAN ID
     * @param trigger_level Home trigger level (see MKSConfig::HOME_TRIGGER_* constants)
     * @param direction Home direction (see MKSConfig::HOME_DIRECTION_* constants)
     * @param speed Home speed (RPM)
     * @return true if command sent
     */
    bool setHomeParameters(uint32_t motor_id, uint8_t trigger_level, uint8_t direction, uint16_t speed);
    
    /**
     * @brief Execute homing sequence
     * @param motor_id Target motor CAN ID
     * @return true if command sent
     */
    bool goHome(uint32_t motor_id);
    
    /**
     * @brief Set 0-mode parameters
     * @param motor_id Target motor CAN ID
     * @param param Parameter value
     * @return true if command sent
     */
    bool setZeroModeParameters(uint32_t motor_id, uint8_t param);
    
    /**
     * @brief Restore factory defaults
     * @param motor_id Target motor CAN ID
     * @return true if command sent
     */
    bool restoreDefaults(uint32_t motor_id);
    
    /**
     * @brief Set motor velocity (continuous speed control)
     * @param motor_id Target motor CAN ID
     * @param speed_rpm Speed in RPM (positive = forward, negative = reverse)
     * @param acceleration Acceleration value (0-255)
     * @return true if command sent
     */
    bool setVelocity(uint32_t motor_id, int16_t speed_rpm, uint8_t acceleration = 5);
    
    // Position Control Modes (all 4 variants)
    
    /**
     * @brief Position Mode 1: Relative motion by pulses (0xFD)
     * @param motor_id Target motor CAN ID
     * @param relative_pulses Relative movement in pulses
     * @param speed_rpm Speed in RPM
     * @param acceleration Acceleration value (0-255)
     * @param clockwise Direction (true = CW, false = CCW)
     * @return true if command sent
     */
    bool setRelativePositionByPulses(uint32_t motor_id, uint32_t relative_pulses, uint16_t speed_rpm, uint8_t acceleration, bool clockwise);
    
    /**
     * @brief Position Mode 2: Absolute motion by pulses (0xFE)
     * @param motor_id Target motor CAN ID
     * @param absolute_pulses Absolute target position in pulses (-8388607 to +8388607)
     * @param speed_rpm Speed in RPM
     * @param acceleration Acceleration value (0-255)
     * @return true if command sent
     */
    bool setAbsolutePositionByPulses(uint32_t motor_id, int32_t absolute_pulses, uint16_t speed_rpm, uint8_t acceleration);
    
    /**
     * @brief Position Mode 3: Relative motion by encoder axis (0xF4)
     * @param motor_id Target motor CAN ID
     * @param relative_axis Relative movement in encoder axis units
     * @param speed_rpm Speed in RPM
     * @param acceleration Acceleration value (0-255)
     * @return true if command sent
     */
    bool setRelativePositionByAxis(uint32_t motor_id, int32_t relative_axis, uint16_t speed_rpm, uint8_t acceleration);
    
    /**
     * @brief Position Mode 4: Absolute motion by encoder axis (0xF5)
     * @param motor_id Target motor CAN ID
     * @param absolute_axis Absolute target position in encoder axis units
     * @param speed_rpm Speed in RPM
     * @param acceleration Acceleration value (0-255)
     * @return true if command sent
     */
    bool setAbsolutePositionByAxis(uint32_t motor_id, int32_t absolute_axis, uint16_t speed_rpm, uint8_t acceleration);
    
    // Legacy method (renamed for clarity)
    /**
     * @brief Legacy position method - uses Position Mode 1 (relative by pulses)
     * @deprecated Use setRelativePositionByPulses instead
     */
    bool setPosition(uint32_t motor_id, int32_t position_pulses, uint16_t speed_rpm, uint8_t acceleration = 10) {
        bool clockwise = position_pulses >= 0;
        uint32_t abs_pulses = static_cast<uint32_t>(std::abs(position_pulses));
        return setRelativePositionByPulses(motor_id, abs_pulses, speed_rpm, acceleration, clockwise);
    }
    
    // Statistics and diagnostics
    
    /**
     * @brief Get number of frames sent
     * @return Frame count
     */
    size_t getFramesSent() const { return frames_sent_; }
    
    /**
     * @brief Get number of send failures
     * @return Failure count
     */
    size_t getSendFailures() const { return send_failures_; }
    
    /**
     * @brief Reset statistics
     */
    void resetStatistics();
    
    /**
     * @brief Check if CAN publisher is ready
     * @return true if ready to send
     */
    bool isReady() const;

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<SocketCANBridge> can_bridge_;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    size_t frames_sent_;
    size_t send_failures_;
    
    /**
     * @brief Internal frame sending with error handling
     * @param frame Frame to send
     * @return true if successful
     */
    bool sendFrameInternal(const CANFrame& frame);
    
    /**
     * @brief Log frame for debugging
     * @param frame Frame to log
     * @param direction "TX" or "RX"
     */
    void logFrame(const CANFrame& frame, const std::string& direction) const;
};

} // namespace arctos_motor_driver

#endif // ARCTOS_MOTOR_DRIVER_CAN_PROTOCOL_HPP_