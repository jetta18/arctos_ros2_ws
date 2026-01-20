#ifndef ARCTOS_MOTOR_DRIVER_MKS_MOTOR_MANAGER_HPP_
#define ARCTOS_MOTOR_DRIVER_MKS_MOTOR_MANAGER_HPP_

#include "mks_motor_driver.hpp"
#include "mks_motor_types.hpp"
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>
#include <mutex>

namespace arctos_motor_driver {

/**
 * @brief Motor Manager for handling multiple MKS motors
 * 
 * This class manages the configuration and state of multiple motors.
 * It separates motor management logic from the pure MKS protocol commands.
 */
class MKSMotorManager {
public:
    /**
     * @brief Constructor
     * @param motor_driver Shared pointer to the motor driver instance
     */
    explicit MKSMotorManager(std::shared_ptr<MKSMotorDriver> motor_driver);
    
    /**
     * @brief Destructor
     */
    ~MKSMotorManager();
    
    // Motor Configuration Management
    
    /**
     * @brief Add a motor/joint to the manager
     * @param joint_name Name of the joint
     * @param config Motor configuration
     * @return true if added successfully
     */
    bool addMotor(const std::string& joint_name, const MotorConfig& config);
    
    /**
     * @brief Remove a motor/joint from the manager
     * @param joint_name Name of the joint
     */
    void removeMotor(const std::string& joint_name);
    
    /**
     * @brief Get list of configured joints
     * @return Vector of joint names
     */
    std::vector<std::string> getJointNames() const;
    
    /**
     * @brief Get motor configuration
     * @param joint_name Name of the joint
     * @return Pointer to config (nullptr if not found)
     */
    const MotorConfig* getMotorConfig(const std::string& joint_name) const;
    
    /**
     * @brief Get motor ID by joint name
     * @param joint_name Name of the joint
     * @return Motor ID (0 if not found)
     */
    uint32_t getMotorId(const std::string& joint_name) const;
    
    /**
     * @brief Find joint name by motor CAN ID
     * @param motor_id CAN ID to look up
     * @return Joint name (empty if not found)
     */
    std::string findJointByMotorId(uint32_t motor_id) const;
    
    // Motor State Management
    
    /**
     * @brief Update motor state from CAN response
     * @param motor_id CAN ID of the motor
     * @param encoder Encoder data
     */
    void updateEncoderData(uint32_t motor_id, const EncoderData& encoder);
    
    /**
     * @brief Update motor speed from CAN response
     * @param motor_id CAN ID of the motor
     * @param speed Speed data
     */
    void updateSpeedData(uint32_t motor_id, const SpeedData& speed);
    
    /**
     * @brief Update IO status from CAN response
     * @param motor_id CAN ID of the motor
     * @param io_status IO status data
     */
    void updateIOStatus(uint32_t motor_id, const IOStatus& io_status);
    
    /**
     * @brief Update motor status from CAN response
     * @param motor_id CAN ID of the motor
     * @param motor_status Motor status data
     */
    void updateMotorStatus(uint32_t motor_id, const MKSMotorStatus& motor_status);
    
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
    
    /**
     * @brief Get the underlying motor driver
     * @return Shared pointer to motor driver
     */
    std::shared_ptr<MKSMotorDriver> getMotorDriver() const { return motor_driver_; }

private:
    std::shared_ptr<MKSMotorDriver> motor_driver_;
    rclcpp::Node::SharedPtr node_;
    
    // Motor management
    mutable std::mutex motors_mutex_;
    std::unordered_map<std::string, MotorConfig> motor_configs_;
    std::unordered_map<std::string, MotorState> motor_states_;
    std::unordered_map<uint32_t, std::string> id_to_joint_map_;  // CAN ID -> joint name
    
    /**
     * @brief Update joint state from motor data
     * @param joint_name Name of the joint
     */
    void updateJointFromMotorData(const std::string& joint_name);
    
    /**
     * @brief Handle incoming CAN response
     * @param motor_id Motor CAN ID
     * @param frame CAN frame
     */
    void handleCANResponse(uint32_t motor_id, const CANFrame& frame);
    
    /**
     * @brief Get motor config by motor ID
     * @param motor_id Motor CAN ID
     * @return Pointer to config (nullptr if not found)
     */
    const MotorConfig* getMotorConfigById(uint32_t motor_id) const;
};

} // namespace arctos_motor_driver

#endif // ARCTOS_MOTOR_DRIVER_MKS_MOTOR_MANAGER_HPP_
