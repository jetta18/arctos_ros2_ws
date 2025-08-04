#ifndef ARCTOS_MOTOR_DRIVER_CAN_FRAME_HPP_
#define ARCTOS_MOTOR_DRIVER_CAN_FRAME_HPP_

#include <array>
#include <vector>
#include <cstdint>
#include <cmath>
#include <can_msgs/msg/frame.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace arctos_motor_driver {

/**
 * @brief Simple CAN frame structure for MKS Servo communication
 * 
 * Represents a standard CAN frame with 11-bit ID and up to 8 data bytes.
 * Includes CRC calculation and validation according to MKS protocol.
 */
struct CANFrame {
    uint32_t id;                    ///< CAN ID (11-bit, 0x001-0x7FF)
    uint8_t dlc;                    ///< Data Length Code (0-8)
    std::array<uint8_t, 8> data;    ///< Data bytes (unused bytes are zero)
    
    /**
     * @brief Default constructor - creates empty frame
     */
    CANFrame();
    
    /**
     * @brief Constructor with parameters
     * @param id CAN ID
     * @param dlc Data length
     * @param data_bytes Data bytes (will be copied and padded to 8 bytes)
     */
    CANFrame(uint32_t id, uint8_t dlc, const std::vector<uint8_t>& data_bytes);
    
    /**
     * @brief Calculate CRC according to MKS protocol
     * @return CRC value: (ID + byte1 + ... + byteN) & 0xFF
     */
    uint8_t calculateCRC() const;
    
    /**
     * @brief Validate frame integrity
     * @return true if frame is valid (ID in range, DLC valid, CRC correct)
     */
    bool isValid() const;
    
    /**
     * @brief Get command byte (first data byte)
     * @return Command byte or 0 if no data
     */
    uint8_t getCommand() const;
    
    /**
     * @brief Convert to ROS CAN message
     * @return ROS can_msgs::msg::Frame
     */
    can_msgs::msg::Frame toRosMessage() const;
    
    /**
     * @brief Create from ROS CAN message
     * @param msg ROS CAN message
     * @return CANFrame object
     */
    static CANFrame fromRosMessage(const can_msgs::msg::Frame::SharedPtr& msg);
    
    /**
     * @brief Create from ROS CAN message (const reference version)
     * @param msg ROS CAN message
     * @return CANFrame object
     */
    static CANFrame fromRosMessage(const can_msgs::msg::Frame& msg);
    
    /**
     * @brief Debug string representation
     * @return Human-readable frame description
     */
    std::string toString() const;
};

/**
 * @brief MKS Servo CAN command structure
 * 
 * Represents a command to be sent to an MKS Servo motor.
 * Automatically handles CRC calculation and frame formatting.
 */
struct CANCommand {
    uint8_t cmd;                    ///< Command byte (e.g., 0x31 for read encoder)
    std::vector<uint8_t> payload;   ///< Command payload (without cmd and CRC)
    
    /**
     * @brief Default constructor
     */
    CANCommand();
    
    /**
     * @brief Constructor with command only (no payload)
     * @param cmd Command byte
     */
    explicit CANCommand(uint8_t cmd);
    
    /**
     * @brief Constructor with command and payload
     * @param cmd Command byte
     * @param payload Command payload
     */
    CANCommand(uint8_t cmd, const std::vector<uint8_t>& payload);
    
    /**
     * @brief Convert to CAN frame for specific motor
     * @param motor_id Target motor CAN ID
     * @return CANFrame ready to send
     */
    CANFrame toFrame(uint32_t motor_id) const;
    
    /**
     * @brief Get total frame size (cmd + payload + CRC)
     * @return Frame DLC value
     */
    uint8_t getFrameSize() const;
    
    /**
     * @brief Debug string representation
     * @return Human-readable command description
     */
    std::string toString() const;
};

/**
 * @brief MKS Servo command constants
 * 
 * Essential commands for motor control and state reading.
 * Based on MKS SERVO 42D/57D manual v1.0.4.
 */
namespace MKSCommands {
    // Read commands (query motor state)
    constexpr uint8_t READ_ENCODER_CARRY    = 0x30;  ///< Read encoder with carry
    constexpr uint8_t READ_ENCODER          = 0x31;  ///< Read encoder (primary) 
    constexpr uint8_t READ_SPEED            = 0x32;  ///< Read motor speed (RPM)
    constexpr uint8_t READ_PULSES           = 0x33;  ///< Read pulse count
    constexpr uint8_t READ_IO_STATUS        = 0x34;  ///< Read IO status 
    constexpr uint8_t READ_ANGLE_ERROR      = 0x39;  ///< Read angle error
    constexpr uint8_t READ_ENABLE_STATUS    = 0x3A;  ///< Read enable status
    constexpr uint8_t READ_MOTOR_STATUS     = 0x3B;  ///< Read detailed motor status
    constexpr uint8_t READ_MOTOR_TYPE       = 0x3D;  ///< Read motor type info
    constexpr uint8_t READ_FIRMWARE_VERSION = 0x3E;  ///< Read firmware version
    constexpr uint8_t RESTORE_DEFAULTS      = 0x3F;  ///< Restore factory defaults
    
    // Motor control commands
    constexpr uint8_t ENABLE_MOTOR          = 0xF3;  ///< Enable/disable motor 
    constexpr uint8_t EMERGENCY_STOP        = 0xF7;  ///< Emergency stop
    constexpr uint8_t QUERY_STATUS          = 0xF1;  ///< Query motor status
    
    // Configuration commands
    constexpr uint8_t CALIBRATE             = 0x80;  ///< Calibrate motor
    constexpr uint8_t SET_WORK_MODE         = 0x82;  ///< Set work mode 
    constexpr uint8_t SET_WORKING_CURRENT   = 0x83;  ///< Set working current
    constexpr uint8_t SET_SUBDIVISION       = 0x84;  ///< Set microstepping
    constexpr uint8_t SET_EN_PIN_ACTIVE     = 0x85;  ///< Set enable pin active level
    constexpr uint8_t SET_DIRECTION         = 0x86;  ///< Set motor direction (pulse interface)
    constexpr uint8_t SET_AUTO_SCREEN_OFF   = 0x87;  ///< Set auto screen off
    constexpr uint8_t SET_SHAFT_PROTECTION  = 0x88;  ///< Set locked-rotor protection
    constexpr uint8_t SET_INTERPOLATION     = 0x89;  ///< Set subdivision interpolation
    constexpr uint8_t SET_CAN_BITRATE       = 0x8A;  ///< Set CAN bitrate
    constexpr uint8_t SET_CAN_ID            = 0x8B;  ///< Set CAN ID
    constexpr uint8_t SET_RESPONSE_ACTIVE   = 0x8C;  ///< Set slave response and active mode
    constexpr uint8_t SET_GROUP_ID          = 0x8D;  ///< Set group ID
    constexpr uint8_t SET_KEY_LOCK          = 0x8F;  ///< Lock/unlock keypad
    constexpr uint8_t SET_HOME_PARAMETERS   = 0x90;  ///< Set homing parameters
    constexpr uint8_t GO_HOME               = 0x91;  ///< Execute homing sequence
    constexpr uint8_t SET_ZERO_POSITION     = 0x92;  ///< Set current pos to zero 
    constexpr uint8_t SET_ZERO_MODE_PARAMS  = 0x9A;  ///< Set 0-mode parameters
    constexpr uint8_t SET_HOLDING_CURRENT   = 0x9B;  ///< Set holding current
    constexpr uint8_t SET_LIMIT_REMAP       = 0x9E;  ///< Set limit port remap 
    
    // Position control commands
    constexpr uint8_t POSITION_MODE_1       = 0xFD;  ///< Relative by pulses
    constexpr uint8_t POSITION_MODE_2       = 0xFE;  ///< Absolute by pulses
    constexpr uint8_t POSITION_MODE_3       = 0xF4;  ///< Relative by axis
    constexpr uint8_t POSITION_MODE_4       = 0xF5;  ///< Absolute by axis
    constexpr uint8_t SPEED_MODE            = 0xF6;  ///< Speed control
}

/**
 * @brief MKS Servo work modes
 */
namespace MKSWorkModes {
    constexpr uint8_t CR_OPEN = 0;   ///< Pulse interface, open loop
    constexpr uint8_t CR_CLOSE = 1;  ///< Pulse interface, closed loop  
    constexpr uint8_t CR_vFOC = 2;   ///< Pulse interface, FOC control
    constexpr uint8_t SR_OPEN = 3;   ///< Serial interface, open loop
    constexpr uint8_t SR_CLOSE = 4;  ///< Serial interface, closed loop
    constexpr uint8_t SR_vFOC = 5;   ///< Serial interface, FOC control  RECOMMENDED
}

/**
 * @brief MKS Servo constants
 */
namespace MKSConstants {
    constexpr uint32_t ENCODER_STEPS_PER_REV = 0x4000;  ///< 16384 steps per revolution
    constexpr double DEGREES_PER_STEP = 360.0 / ENCODER_STEPS_PER_REV;  ///< ~0.022°/step
    constexpr double STEPS_PER_DEGREE = ENCODER_STEPS_PER_REV / 360.0;   ///< ~45.51 steps/°
    
    // Mathematical constants
    constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;
    
    // Motor specifications
    constexpr uint16_t MAX_CURRENT_42D = 3000;  ///< Max current for 42D motors (mA)
    constexpr uint16_t MAX_CURRENT_57D = 5200;  ///< Max current for 57D motors (mA)
    constexpr uint16_t MAX_SPEED = 3000;        ///< Max speed (RPM)
}

/**
 * @brief MKS Servo configuration constants
 */
namespace MKSConfig {
    // Enable pin active levels
    constexpr uint8_t EN_PIN_ACTIVE_LOW = 0;    ///< Enable pin active low
    constexpr uint8_t EN_PIN_ACTIVE_HIGH = 1;   ///< Enable pin active high
    constexpr uint8_t EN_PIN_ALWAYS_HOLD = 2;   ///< Enable pin always active (hold)
    
    // Motor direction (for pulse interface)
    constexpr uint8_t DIRECTION_CW = 0;         ///< Clockwise direction
    constexpr uint8_t DIRECTION_CCW = 1;        ///< Counter-clockwise direction
    
    // CAN bitrates
    constexpr uint8_t CAN_BITRATE_125K = 0;     ///< 125 kbps
    constexpr uint8_t CAN_BITRATE_250K = 1;     ///< 250 kbps
    constexpr uint8_t CAN_BITRATE_500K = 2;     ///< 500 kbps
    constexpr uint8_t CAN_BITRATE_1M = 3;       ///< 1 Mbps
    
    // Holding current percentages (for 0x9B command)
    constexpr uint8_t HOLDING_CURRENT_10_PERCENT = 0;  ///< 10% holding current
    constexpr uint8_t HOLDING_CURRENT_20_PERCENT = 1;  ///< 20% holding current
    constexpr uint8_t HOLDING_CURRENT_30_PERCENT = 2;  ///< 30% holding current
    constexpr uint8_t HOLDING_CURRENT_40_PERCENT = 3;  ///< 40% holding current
    constexpr uint8_t HOLDING_CURRENT_50_PERCENT = 4;  ///< 50% holding current
    constexpr uint8_t HOLDING_CURRENT_60_PERCENT = 5;  ///< 60% holding current
    constexpr uint8_t HOLDING_CURRENT_70_PERCENT = 6;  ///< 70% holding current
    constexpr uint8_t HOLDING_CURRENT_80_PERCENT = 7;  ///< 80% holding current
    constexpr uint8_t HOLDING_CURRENT_90_PERCENT = 8;  ///< 90% holding current
    
    // Home trigger levels
    constexpr uint8_t HOME_TRIGGER_LOW = 0;     ///< Home trigger on low level
    constexpr uint8_t HOME_TRIGGER_HIGH = 1;    ///< Home trigger on high level
    
    // Home directions
    constexpr uint8_t HOME_DIRECTION_CW = 0;    ///< Home in clockwise direction
    constexpr uint8_t HOME_DIRECTION_CCW = 1;   ///< Home in counter-clockwise direction
    
    // Response and active modes
    constexpr uint8_t RESPONSE_DISABLED = 0;   ///< Disable response
    constexpr uint8_t RESPONSE_ENABLED = 1;    ///< Enable response (default)
    constexpr uint8_t ACTIVE_DISABLED = 0;     ///< Disable active mode
    constexpr uint8_t ACTIVE_ENABLED = 1;      ///< Enable active mode (default)
    
    // Generic enable/disable
    constexpr uint8_t DISABLED = 0;            ///< Generic disable value
    constexpr uint8_t ENABLED = 1;             ///< Generic enable value
}

} // namespace arctos_motor_driver

#endif // ARCTOS_MOTOR_DRIVER_CAN_FRAME_HPP_
