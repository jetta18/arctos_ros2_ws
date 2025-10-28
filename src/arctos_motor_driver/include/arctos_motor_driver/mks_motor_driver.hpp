#ifndef ARCTOS_MOTOR_DRIVER_MKS_MOTOR_DRIVER_HPP_
#define ARCTOS_MOTOR_DRIVER_MKS_MOTOR_DRIVER_HPP_

#include <memory>
#include <string>
#include <functional>
#include <unordered_map>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>

#include "mks_motor_driver/driver.hpp"
#include "mks_motor_driver/can_transport.hpp"
#include "arctos_motor_driver/mks_data_types.hpp"

namespace arctos_motor_driver {

// CANFrame is defined in mks_data_types.hpp

/**
 * @brief ROS2 Wrapper for MKS Protocol Driver
 * 
 * This class wraps the standalone mks::MksDriver and provides ROS2 integration.
 * It maintains backward compatibility with the existing arctos_motor_driver API.
 */
class MKSMotorDriver {
public:
    /**
     * @brief Constructor
     * @param node ROS2 node for communication
     * @param can_interface_name CAN interface (e.g., "can0")
     */
    explicit MKSMotorDriver(rclcpp::Node::SharedPtr node, const std::string& can_interface_name);
    
    /**
     * @brief Destructor
     */
    ~MKSMotorDriver();
    
    /**
     * @brief Get the ROS2 node
     * @return Shared pointer to node
     */
    rclcpp::Node::SharedPtr getNode() const { return node_; }
    
    // =================================================================
    // MKS Manual Commands - Configuration (Section 5.2)
    // =================================================================
    
    bool calibrateMotor(uint32_t motor_id);
    bool setWorkMode(uint32_t motor_id, uint8_t mode);
    bool setWorkingCurrent(uint32_t motor_id, uint16_t current_ma);
    bool setHoldingCurrentPercentage(uint32_t motor_id, uint8_t percentage);
    bool setSubdivision(uint32_t motor_id, uint8_t subdivision);
    bool restoreDefaults(uint32_t motor_id);
    bool setHomeParameters(uint32_t motor_id, uint8_t trigger_level, uint8_t direction, uint16_t speed_rpm, uint8_t enable_limit);
    bool goHome(uint32_t motor_id);
    bool setZeroPosition(uint32_t motor_id);
    bool setLimitPortRemap(uint32_t motor_id, bool enable);
    
    // =================================================================
    // MKS Manual Commands - Motor Control (Section 6.2, 6.3, 6.4)
    // =================================================================
    
    bool queryMotorStatus(uint32_t motor_id);
    bool enableMotor(uint32_t motor_id, bool enable);
    bool emergencyStop(uint32_t motor_id);
    bool setVelocity(uint32_t motor_id, int16_t speed_rpm, uint8_t acceleration);
    
    // =================================================================
    // MKS Manual Commands - Position Control (Section 6.5-6.8)
    // =================================================================
    
    bool setRelativePositionByPulses(uint32_t motor_id, uint32_t pulses, uint16_t speed_rpm, uint8_t acceleration, bool clockwise);
    bool setAbsolutePositionByPulses(uint32_t motor_id, int32_t abs_pulses, uint16_t speed_rpm, uint8_t acceleration);
    bool setRelativePositionByAxis(uint32_t motor_id, int32_t rel_axis, uint16_t speed_rpm, uint8_t acceleration);
    bool setAbsolutePositionByAxis(uint32_t motor_id, int32_t abs_axis, uint16_t speed_rpm, uint8_t acceleration);
    
    // =================================================================
    // MKS Manual Commands - Read Parameters (Section 5.1)
    // =================================================================
    
    bool requestEncoderReading(uint32_t motor_id);
    bool requestSpeedReading(uint32_t motor_id);
    bool requestIOStatus(uint32_t motor_id);
    
    // CAN message processing
    
    /**
     * @brief Callback type for CAN response handling
     * @param motor_id Motor CAN ID
     * @param frame Parsed CAN frame
     */
    using ResponseCallback = std::function<void(uint32_t motor_id, const CANFrame& frame)>;
    
    /**
     * @brief Register callback for CAN responses
     * @param callback Function to call when a response is received
     */
    void registerResponseCallback(const ResponseCallback& callback);
    
    /**
     * @brief Process incoming CAN message (ROS2 format)
     * @param msg CAN frame message
     */
    void processCANMessage(const can_msgs::msg::Frame& msg);

private:
    /**
     * @brief Get or create driver for specific motor ID
     * @param motor_id Motor CAN ID
     * @return Shared pointer to driver
     */
    std::shared_ptr<mks::MksDriver> getDriver(uint32_t motor_id);
    
    /**
     * @brief Process incoming CAN frame (mks format)
     * @param frame CAN frame
     */
    void processCANFrame(const mks::CanFrame& frame);
    
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<mks::ICanTransport> transport_;
    
    std::mutex drivers_mutex_;
    std::unordered_map<uint32_t, std::shared_ptr<mks::MksDriver>> drivers_;
    
    std::mutex callback_mutex_;
    ResponseCallback response_callback_;
};

} // namespace arctos_motor_driver

#endif // ARCTOS_MOTOR_DRIVER_MKS_MOTOR_DRIVER_HPP_
