#ifndef ARCTOS_MOTOR_DRIVER_MKS_MOTOR_DRIVER_HPP_
#define ARCTOS_MOTOR_DRIVER_MKS_MOTOR_DRIVER_HPP_

#include "can_protocol.hpp"
#include "mks_data_types.hpp"
#include "can_protocol.hpp"
#include "socket_can_bridge.hpp"
#include <memory>
#include <unordered_map>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
// For time types used in periodic logging
#include <chrono>

namespace arctos_motor_driver {

/**
 * @brief Motor configuration for a single joint
 */
struct MotorConfig {
    uint32_t motor_id;          ///< CAN ID of the motor
    std::string hardware_type;  ///< "MKS_42D" or "MKS_57D"
    double gear_ratio;          ///< Gear ratio (motor_revs / joint_revs)
    bool inverted;              ///< Whether direction is inverted
    uint16_t working_current;   ///< Working current in mA
    uint16_t holding_current;   ///< Holding current in mA
    bool limit_remap_enabled;   ///< Whether limit port remapping is enabled
    uint8_t work_mode;          ///< Work mode (MKSWorkModes constants)
    
    MotorConfig() 
        : motor_id(0), hardware_type("MKS_42D"), gear_ratio(1.0), 
          inverted(false), working_current(1000), holding_current(70), 
          limit_remap_enabled(false), work_mode(5) {}  // Default to SR_vFOC
};

/**
 * @brief Current state of a motor/joint
 */
struct MotorState {
    // Latest data
    EncoderData encoder;
    SpeedData speed;
    IOStatus io_status;
    MKSMotorStatus motor_status;
    
    // Joint state (converted from motor data)
    double joint_position;      ///< Joint position in radians
    double joint_velocity;      ///< Joint velocity in rad/s
    
    // State tracking
    rclcpp::Time last_update;
    rclcpp::Time last_command;
    bool data_valid;
    
    MotorState() 
        : joint_position(0.0), joint_velocity(0.0), data_valid(false) {}
};

/**
 * @brief Clean, focused motor driver for MKS Servo motors
 * 
 * This class provides a simplified interface for controlling MKS Servo motors
 * over CAN bus. It focuses on essential functionality needed for ROS2 control.
 */
class MKSMotorDriver {
public:
    /**
     * @brief Constructor
     * @param node ROS2 node for communication
     */
    explicit MKSMotorDriver(rclcpp::Node::SharedPtr node, const std::string& can_interface_name);
    
    /**
     * @brief Destructor
     */
    ~MKSMotorDriver();
    
    // Configuration methods
    
    /**
     * @brief Add a motor/joint to the driver
     * @param joint_name Name of the joint
     * @param config Motor configuration
     * @return true if added successfully
     */
    bool addMotor(const std::string& joint_name, const MotorConfig& config);
    
    /**
     * @brief Remove a motor/joint from the driver
     * @param joint_name Name of the joint
     */
    void removeMotor(const std::string& joint_name);
    
    /**
     * @brief Get list of configured joints
     * @return Vector of joint names
     */
    std::vector<std::string> getJointNames() const;
    
    // Motor control methods
    
    /**
     * @brief Initialize all motors (set work mode, current, etc.)
     * @return true if all motors initialized successfully
     */
    bool initializeMotors();
    
    /**
     * @brief Enable or disable a motor
     * @param joint_name Name of the joint
     * @param enable true to enable, false to disable
     * @return true if command sent successfully
     */
    bool enableMotor(const std::string& joint_name, bool enable);
    
    /**
     * @brief Enable or disable all motors
     * @param enable true to enable, false to disable
     * @return true if all commands sent successfully
     */
    bool enableAllMotors(bool enable);
    
    /**
     * @brief Set zero position for a motor
     * @param joint_name Name of the joint
     * @return true if command sent successfully
     */
    bool setZeroPosition(const std::string& joint_name);
    
    /**
     * @brief Emergency stop all motors
     * @return true if all commands sent successfully
     */
    bool emergencyStopAll();
    
    /**
     * @brief Calibrate a motor (motor must be unloaded)
     * @param joint_name Name of the joint
     * @return true if command sent successfully
     */
    bool calibrateMotor(const std::string& joint_name);
    
    /**
     * @brief Set homing parameters for a motor
     * @param joint_name Name of the joint
     * @param trigger_level Home trigger level (0=low, 1=high)
     * @param direction Home direction (0=CW, 1=CCW)
     * @param speed Home speed in rad/s
     * @return true if command sent successfully
     */
    bool setHomeParameters(const std::string& joint_name, uint8_t trigger_level, uint8_t direction, double speed);
    
    /**
     * @brief Execute homing sequence for a motor
     * @param joint_name Name of the joint
     * @return true if command sent successfully
     */
    bool goHome(const std::string& joint_name);
    
    /**
     * @brief Home all motors sequentially
     * @return true if all homing commands sent successfully
     */
    bool homeAllMotors();
    
    /**
     * @brief Set holding current percentage for a motor
     * @param joint_name Name of the joint
     * @param percentage Holding current percentage (10-90%)
     * @return true if command sent successfully
     */
    bool setHoldingCurrentPercentage(const std::string& joint_name, uint8_t percentage);
    
    /**
     * @brief Set microstepping subdivision for a motor
     * @param joint_name Name of the joint
     * @param subdivision Subdivision value
     * @return true if command sent successfully
     */
    bool setSubdivision(const std::string& joint_name, uint8_t subdivision);
    
    /**
     * @brief Restore factory defaults for a motor
     * @param joint_name Name of the joint
     * @return true if command sent successfully
     */
    bool restoreDefaults(const std::string& joint_name);
    
    /**
     * @brief Set joint velocity (continuous motion)
     * @param joint_name Name of the joint
     * @param velocity Joint velocity in rad/s
     * @return true if command sent successfully
     */
    bool setJointVelocity(const std::string& joint_name, double velocity);
    
    // Position Control Modes - All 4 MKS variants for testing
    
    /**
     * @brief Mode 1: Relative position by pulses (0xFD)
     * @param joint_name Name of the joint
     * @param relative_position Relative movement in radians
     * @param speed Speed in rad/s
     * @return true if command sent successfully
     */
    bool setJointRelativePositionByPulses(const std::string& joint_name, double relative_position, double speed = 1.0);
    
    /**
     * @brief Mode 2: Absolute position by pulses (0xFE) - RECOMMENDED FOR TRAJECTORY FOLLOWING
     * @param joint_name Name of the joint
     * @param absolute_position Target position in radians
     * @param speed Speed in rad/s
     * @return true if command sent successfully
     */
    bool setJointAbsolutePositionByPulses(const std::string& joint_name, double absolute_position, double speed = 1.0);
    
    /**
     * @brief Mode 3: Relative position by encoder axis (0xF4)
     * @param joint_name Name of the joint
     * @param relative_position Relative movement in radians
     * @param speed Speed in rad/s
     * @return true if command sent successfully
     */
    bool setJointRelativePositionByAxis(const std::string& joint_name, double relative_position, double speed = 1.0);
    
    /**
     * @brief Mode 4: Absolute position by encoder axis (0xF5) - ALTERNATIVE FOR TRAJECTORY FOLLOWING
     * @param joint_name Name of the joint
     * @param absolute_position Target position in radians
     * @param speed Speed in rad/s
     * @return true if command sent successfully
     */
    bool setJointAbsolutePositionByAxis(const std::string& joint_name, double absolute_position, double speed = 1.0);
    
    // Legacy method (for backward compatibility)
    /**
     * @brief Legacy position method - uses Mode 1 (relative by pulses)
     * @deprecated Use specific position mode methods instead
     */
    bool setJointPosition(const std::string& joint_name, double position, double speed = 1.0) {
        return setJointRelativePositionByPulses(joint_name, position, speed);
    }
    
    // State reading methods
    
    /**
     * @brief Update joint states by requesting data from motors
     * This should be called regularly to keep joint states current
     */
    void updateJointStates();

    /**
     * @brief Set microsecond delay between consecutive CAN requests.
     * @param us Microseconds to sleep between requests (0 to disable)
     */
    void setInterRequestDelayUs(int us) {
        inter_request_delay_us_.store(us);
    }

    /**
     * @brief Start background polling of joint states at a fixed rate.
     * @param hz Polling frequency in Hz (clamped internally to [1, 1000]).
     */
    void startPolling(double hz = 200.0);

    /**
     * @brief Stop background polling if running.
     */
    void stopPolling();
    
    /**
     * @brief Get current joint position
     * @param joint_name Name of the joint
     * @return Joint position in radians
     */
    double getJointPosition(const std::string& joint_name) const;
    
    /**
     * @brief Get current joint velocity
     * @param joint_name Name of the joint
     * @return Joint velocity in rad/s
     */
    double getJointVelocity(const std::string& joint_name) const;
    
    /**
     * @brief Check if joint data is valid and recent
     * @param joint_name Name of the joint
     * @param max_age_seconds Maximum age of data in seconds
     * @return true if data is valid and recent
     */
    bool isJointDataValid(const std::string& joint_name, double max_age_seconds = 1.0) const;
    
    /**
     * @brief Get motor state for debugging
     * @param joint_name Name of the joint
     * @return Pointer to motor state (nullptr if not found)
     */
    const MotorState* getMotorState(const std::string& joint_name) const;
    
    // CAN message processing
    
    /**
     * @brief Process incoming CAN message
     * @param msg CAN frame message
     */
    void processCANMessage(const can_msgs::msg::Frame& msg);
    
    // Diagnostics and statistics
    
    /**
     * @brief Get driver statistics
     * @return Human-readable statistics string
     */
    std::string getStatistics() const;
    
    /**
     * @brief Reset all statistics
     */
    void resetStatistics();

private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<CANProtocol> can_protocol_;
    std::shared_ptr<SocketCANBridge> can_bridge_;
    
    // Motor management
    mutable std::mutex motors_mutex_;
    std::unordered_map<std::string, MotorConfig> motor_configs_;
    std::unordered_map<std::string, MotorState> motor_states_;
    std::unordered_map<uint32_t, std::string> id_to_joint_map_;  // CAN ID -> joint name
    
    // Statistics
    mutable std::mutex stats_mutex_;
    size_t messages_processed_;
    size_t messages_ignored_;
    size_t encoder_updates_;
    size_t speed_updates_;
    size_t io_updates_;
    size_t status_updates_;
    
    /**
     * @brief Process encoder response (0x31)
     * @param motor_id CAN ID of responding motor
     * @param frame CAN frame with response
     */
    void processEncoderResponse(uint32_t motor_id, const CANFrame& frame);
    
    /**
     * @brief Process speed response (0x32)
     * @param motor_id CAN ID of responding motor
     * @param frame CAN frame with response
     */
    void processSpeedResponse(uint32_t motor_id, const CANFrame& frame);
    
    /**
     * @brief Process IO status response (0x34)
     * @param motor_id CAN ID of responding motor
     * @param frame CAN frame with response
     */
    void processIOStatusResponse(uint32_t motor_id, const CANFrame& frame);
    
    /**
     * @brief Process motor status response (0xF1, 0xF3, 0x3A)
     * @param motor_id CAN ID of responding motor
     * @param frame CAN frame with response
     */
    void processMotorStatusResponse(uint32_t motor_id, const CANFrame& frame);
    
    /**
     * @brief Update joint state from motor data
     * @param joint_name Name of the joint
     */
    void updateJointFromMotorData(const std::string& joint_name);
    
    /**
     * @brief Find joint name by motor CAN ID
     * @param motor_id CAN ID to look up
     * @return Joint name (empty if not found)
     */
    std::string findJointByMotorId(uint32_t motor_id) const;
    
    /**
     * @brief Log CAN message for debugging
     * @param frame CAN frame
     * @param direction "RX" or "TX"
     * @param description Additional description
     */
    void logCANMessage(const CANFrame& frame, const std::string& direction, 
                       const std::string& description = "") const;

    // Background polling
    void pollingLoop();

    // Polling thread/mode
    std::thread polling_thread_;
    std::atomic<bool> polling_running_{false};
    std::atomic<double> polling_rate_hz_{200.0};
    // Optional micro-delay between back-to-back CAN TX in updateJointStates()
    std::atomic<int> inter_request_delay_us_{0};
};

} // namespace arctos_motor_driver

#endif // ARCTOS_MOTOR_DRIVER_MKS_MOTOR_DRIVER_HPP_
