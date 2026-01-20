#include "arctos_motor_driver/mks_motor_manager.hpp"
#include <algorithm>

namespace arctos_motor_driver {

MKSMotorManager::MKSMotorManager(std::shared_ptr<MKSMotorDriver> motor_driver)
    : motor_driver_(motor_driver), node_(motor_driver->getNode()) {
    
    // Register callback to receive CAN responses
    motor_driver_->registerResponseCallback(
        [this](uint32_t motor_id, const CANFrame& frame) {
            this->handleCANResponse(motor_id, frame);
        });
    
    RCLCPP_INFO(node_->get_logger(), "MKSMotorManager initialized");
}

MKSMotorManager::~MKSMotorManager() {
    RCLCPP_INFO(node_->get_logger(), "MKSMotorManager destroyed");
}

bool MKSMotorManager::addMotor(const std::string& joint_name, const MotorConfig& config) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    // Check if joint already exists
    if (motor_configs_.find(joint_name) != motor_configs_.end()) {
        RCLCPP_WARN(node_->get_logger(), "Joint '%s' already exists, updating config", joint_name.c_str());
    }
    
    // Check if motor ID is already used
    for (const auto& [existing_joint, existing_config] : motor_configs_) {
        if (existing_config.motor_id == config.motor_id && existing_joint != joint_name) {
            RCLCPP_ERROR(node_->get_logger(), "Motor ID %u already used by joint '%s'", 
                         config.motor_id, existing_joint.c_str());
            return false;
        }
    }
    
    // Add motor configuration
    motor_configs_[joint_name] = config;
    motor_states_[joint_name] = MotorState();
    id_to_joint_map_[config.motor_id] = joint_name;
    
    RCLCPP_INFO(node_->get_logger(), "Added motor: joint='%s' id=%u type=%s gear=%.2f inverted=%s",
                joint_name.c_str(), config.motor_id, config.hardware_type.c_str(),
                config.gear_ratio, config.inverted ? "YES" : "NO");
    
    return true;
}

void MKSMotorManager::removeMotor(const std::string& joint_name) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto config_it = motor_configs_.find(joint_name);
    if (config_it != motor_configs_.end()) {
        uint32_t motor_id = config_it->second.motor_id;
        motor_configs_.erase(config_it);
        motor_states_.erase(joint_name);
        id_to_joint_map_.erase(motor_id);
        
        RCLCPP_INFO(node_->get_logger(), "Removed motor: joint='%s' id=%u", joint_name.c_str(), motor_id);
    }
}

std::vector<std::string> MKSMotorManager::getJointNames() const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    std::vector<std::string> names;
    names.reserve(motor_configs_.size());
    
    for (const auto& [joint_name, config] : motor_configs_) {
        names.push_back(joint_name);
    }
    
    return names;
}

const MotorConfig* MKSMotorManager::getMotorConfig(const std::string& joint_name) const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto it = motor_configs_.find(joint_name);
    if (it != motor_configs_.end()) {
        return &it->second;
    }
    
    return nullptr;
}

uint32_t MKSMotorManager::getMotorId(const std::string& joint_name) const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto it = motor_configs_.find(joint_name);
    if (it != motor_configs_.end()) {
        return it->second.motor_id;
    }
    
    return 0;
}

std::string MKSMotorManager::findJointByMotorId(uint32_t motor_id) const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto it = id_to_joint_map_.find(motor_id);
    if (it != id_to_joint_map_.end()) {
        return it->second;
    }
    
    return "";
}

void MKSMotorManager::updateEncoderData(uint32_t motor_id, const EncoderData& encoder) {
    std::string joint_name = findJointByMotorId(motor_id);
    if (joint_name.empty()) return;
    
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    motor_states_[joint_name].encoder = encoder;
    motor_states_[joint_name].last_update = node_->get_clock()->now();
    
    // Update joint state
    updateJointFromMotorData(joint_name);
}

void MKSMotorManager::updateSpeedData(uint32_t motor_id, const SpeedData& speed) {
    std::string joint_name = findJointByMotorId(motor_id);
    if (joint_name.empty()) return;
    
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    motor_states_[joint_name].speed = speed;
    motor_states_[joint_name].last_update = node_->get_clock()->now();
    
    // Update joint velocity
    auto config_it = motor_configs_.find(joint_name);
    if (config_it != motor_configs_.end()) {
        motor_states_[joint_name].joint_velocity = 
            speed.toJointVelocity(config_it->second.gear_ratio, config_it->second.inverted);
    }
}

void MKSMotorManager::updateIOStatus(uint32_t motor_id, const IOStatus& io_status) {
    std::string joint_name = findJointByMotorId(motor_id);
    if (joint_name.empty()) return;
    
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    motor_states_[joint_name].io_status = io_status;
    motor_states_[joint_name].last_update = node_->get_clock()->now();
}

void MKSMotorManager::updateMotorStatus(uint32_t motor_id, const MKSMotorStatus& motor_status) {
    std::string joint_name = findJointByMotorId(motor_id);
    if (joint_name.empty()) return;
    
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    motor_states_[joint_name].motor_status = motor_status;
    motor_states_[joint_name].last_update = node_->get_clock()->now();
}

double MKSMotorManager::getJointPosition(const std::string& joint_name) const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto state_it = motor_states_.find(joint_name);
    if (state_it != motor_states_.end()) {
        return state_it->second.joint_position;
    }
    
    return 0.0;
}

double MKSMotorManager::getJointVelocity(const std::string& joint_name) const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto state_it = motor_states_.find(joint_name);
    if (state_it != motor_states_.end()) {
        return state_it->second.joint_velocity;
    }
    
    return 0.0;
}

bool MKSMotorManager::isJointDataValid(const std::string& joint_name, double max_age_seconds) const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto state_it = motor_states_.find(joint_name);
    if (state_it == motor_states_.end()) {
        return false;
    }
    
    const MotorState& state = state_it->second;
    if (!state.data_valid) {
        return false;
    }
    
    auto now = node_->get_clock()->now();
    double age_seconds = (now - state.last_update).seconds();
    
    return age_seconds <= max_age_seconds;
}

const MotorState* MKSMotorManager::getMotorState(const std::string& joint_name) const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto state_it = motor_states_.find(joint_name);
    if (state_it != motor_states_.end()) {
        return &state_it->second;
    }
    
    return nullptr;
}

void MKSMotorManager::updateJointFromMotorData(const std::string& joint_name) {
    // This function should be called with motors_mutex_ already locked
    
    auto config_it = motor_configs_.find(joint_name);
    auto state_it = motor_states_.find(joint_name);
    
    if (config_it == motor_configs_.end() || state_it == motor_states_.end()) {
        return;
    }
    
    const MotorConfig& config = config_it->second;
    MotorState& state = state_it->second;
    
    // Update joint position from encoder data
    if (state.encoder.is_valid) {
        state.joint_position = state.encoder.toJointAngle(config.gear_ratio, config.inverted);
        state.data_valid = true;
    }
    
    // Update joint velocity from speed data
    if (state.speed.is_valid) {
        state.joint_velocity = state.speed.toJointVelocity(config.gear_ratio, config.inverted);
    }
}

const MotorConfig* MKSMotorManager::getMotorConfigById(uint32_t motor_id) const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto it = id_to_joint_map_.find(motor_id);
    if (it != id_to_joint_map_.end()) {
        const std::string& joint_name = it->second;
        auto config_it = motor_configs_.find(joint_name);
        if (config_it != motor_configs_.end()) {
            return &config_it->second;
        }
    }
    
    return nullptr;
}

void MKSMotorManager::handleCANResponse(uint32_t motor_id, const CANFrame& frame) {
    // Check if this is a valid response frame
    if (frame.dlc == 0) {
        return;
    }
    
    uint8_t command = frame.data[0];
    
    // Parse response based on command byte
    switch (command) {
        case 0x31: {  // Encoder reading response
            EncoderData encoder = EncoderData::fromCANFrame(frame);
            if (encoder.is_valid) {
                encoder.timestamp = node_->get_clock()->now();
                updateEncoderData(motor_id, encoder);
                RCLCPP_DEBUG(node_->get_logger(), 
                             "Motor %u encoder: raw=%ld, angle=%.2f°", 
                             motor_id, encoder.raw_value, encoder.angle_degrees);
            }
            break;
        }
        
        case 0x32: {  // Speed reading response
            SpeedData speed = SpeedData::fromCANFrame(frame);
            if (speed.is_valid) {
                speed.timestamp = node_->get_clock()->now();
                updateSpeedData(motor_id, speed);
                RCLCPP_DEBUG(node_->get_logger(), 
                             "Motor %u speed: %d RPM (%.2f rad/s)", 
                             motor_id, speed.rpm, speed.rad_per_sec);
            }
            break;
        }
        
        case 0x34: {  // IO status response
            const MotorConfig* config = getMotorConfigById(motor_id);
            std::string hw_type = config ? config->hardware_type : "MKS_42D";
            bool limit_remap = config ? config->limit_remap_enabled : false;
            
            IOStatus io = IOStatus::fromCANFrame(frame, hw_type, limit_remap);
            if (io.is_valid) {
                io.timestamp = node_->get_clock()->now();
                updateIOStatus(motor_id, io);
                RCLCPP_DEBUG(node_->get_logger(), 
                             "Motor %u IO: IN1=%d IN2=%d OUT1=%d OUT2=%d", 
                             motor_id, io.in1, io.in2, io.out1, io.out2);
            }
            break;
        }
        
        case 0xF1: {  // Motor status query response
            MKSMotorStatus status = MKSMotorStatus::fromQueryResponse(frame);
            if (status.is_valid) {
                status.timestamp = node_->get_clock()->now();
                updateMotorStatus(motor_id, status);
                RCLCPP_DEBUG(node_->get_logger(), 
                             "Motor %u status: %s", 
                             motor_id, status.getStatusDescription().c_str());
            }
            break;
        }
        
        case 0xF3:    // Enable/disable response
        case 0x3A: {  // Enable status response
            MKSMotorStatus status = MKSMotorStatus::fromEnableResponse(frame);
            if (status.is_valid) {
                status.timestamp = node_->get_clock()->now();
                updateMotorStatus(motor_id, status);
                RCLCPP_DEBUG(node_->get_logger(), 
                             "Motor %u enable status: %s", 
                             motor_id, status.enabled ? "ENABLED" : "DISABLED");
            }
            break;
        }
        
        default:
            // Unknown or unhandled response
            RCLCPP_DEBUG(node_->get_logger(), 
                         "Motor %u: Unhandled response command 0x%02X", 
                         motor_id, command);
            break;
    }
}

} // namespace arctos_motor_driver
