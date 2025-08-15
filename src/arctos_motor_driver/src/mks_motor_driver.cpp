#include "arctos_motor_driver/mks_motor_driver.hpp"
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>

namespace arctos_motor_driver {

MKSMotorDriver::MKSMotorDriver(rclcpp::Node::SharedPtr node, const std::string& can_interface_name)
    : node_(node), messages_processed_(0), messages_ignored_(0),
      encoder_updates_(0), speed_updates_(0), io_updates_(0), status_updates_(0) {

    can_bridge_ = std::make_shared<SocketCANBridge>(node_);
    if (!can_bridge_->open(can_interface_name)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open CAN interface '%s'", can_interface_name.c_str());
        throw std::runtime_error("Failed to open CAN interface");
    }

    can_protocol_ = std::make_unique<CANProtocol>(node_, can_bridge_);

    can_bridge_->register_receive_callback(
        [this](const can_msgs::msg::Frame& msg) {
            this->processCANMessage(msg);
        });

    // Initialize tuning params
    try {
        // Optional ROS parameter to experiment with small TX gaps (microseconds)
        int delay_us = node_->declare_parameter<int>("mks_inter_request_delay_us", 150);
        inter_request_delay_us_.store(delay_us);
        if (delay_us > 0) {
            RCLCPP_INFO(node_->get_logger(), "Using inter-request delay: %d us", delay_us);
        }
    } catch (const std::exception&){ /* parameter may already exist in some contexts */ }

    RCLCPP_INFO(node_->get_logger(), "MKSMotorDriver initialized with direct CAN access on '%s'", can_interface_name.c_str());
}

MKSMotorDriver::~MKSMotorDriver() {
    // Ensure background polling is stopped before destruction
    stopPolling();
    RCLCPP_INFO(node_->get_logger(), "MKSMotorDriver destroyed. %s", getStatistics().c_str());
}

bool MKSMotorDriver::addMotor(const std::string& joint_name, const MotorConfig& config) {
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

void MKSMotorDriver::removeMotor(const std::string& joint_name) {
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

std::vector<std::string> MKSMotorDriver::getJointNames() const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    std::vector<std::string> names;
    names.reserve(motor_configs_.size());
    
    for (const auto& [joint_name, config] : motor_configs_) {
        names.push_back(joint_name);
    }
    
    return names;
}

bool MKSMotorDriver::initializeMotors() {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    bool all_success = true;
    
    for (const auto& [joint_name, config] : motor_configs_) {
        RCLCPP_INFO(node_->get_logger(), "Initializing motor for joint '%s' (ID %u)", 
                    joint_name.c_str(), config.motor_id);
        
        // Set work mode from configuration
        if (!can_protocol_->setWorkMode(config.motor_id, config.work_mode)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set work mode %d for joint '%s'", config.work_mode, joint_name.c_str());
            all_success = false;
        } else {
            RCLCPP_INFO(node_->get_logger(), "Set work mode %d for joint '%s' (motor %d)", 
                       config.work_mode, joint_name.c_str(), config.motor_id);
        }
        
        // Set working current
        if (!can_protocol_->setWorkingCurrent(config.motor_id, config.working_current)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set working current for joint '%s'", joint_name.c_str());
            all_success = false;
        }
        
        // Set limit port remap if needed
        if (config.limit_remap_enabled) {
            if (!can_protocol_->setLimitPortRemap(config.motor_id, true)) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to set limit port remap for joint '%s'", joint_name.c_str());
                all_success = false;
            }
        }
        
        // Small delay between commands
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    RCLCPP_INFO(node_->get_logger(), "Motor initialization %s", all_success ? "completed successfully" : "had errors");
    return all_success;
}

bool MKSMotorDriver::enableMotor(const std::string& joint_name, bool enable) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto config_it = motor_configs_.find(joint_name);
    if (config_it == motor_configs_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint '%s' not found", joint_name.c_str());
        return false;
    }
    
    bool success = can_protocol_->enableMotor(config_it->second.motor_id, enable);
    if (success) {
        RCLCPP_INFO(node_->get_logger(), "Motor for joint '%s' %s", 
                    joint_name.c_str(), enable ? "enabled" : "disabled");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to %s motor for joint '%s'", 
                     enable ? "enable" : "disable", joint_name.c_str());
    }
    
    return success;
}

bool MKSMotorDriver::enableAllMotors(bool enable) {
    auto joint_names = getJointNames();
    bool all_success = true;
    
    for (const auto& joint_name : joint_names) {
        if (!enableMotor(joint_name, enable)) {
            all_success = false;
        }
        // Small delay between commands
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    return all_success;
}

bool MKSMotorDriver::setZeroPosition(const std::string& joint_name) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto config_it = motor_configs_.find(joint_name);
    if (config_it == motor_configs_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint '%s' not found", joint_name.c_str());
        return false;
    }
    
    bool success = can_protocol_->setZeroPosition(config_it->second.motor_id);
    if (success) {
        RCLCPP_INFO(node_->get_logger(), "Set zero position for joint '%s'", joint_name.c_str());
        // Reset our stored position
        motor_states_[joint_name].joint_position = 0.0;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set zero position for joint '%s'", joint_name.c_str());
    }
    
    return success;
}

bool MKSMotorDriver::emergencyStopAll() {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    bool all_success = true;
    
    RCLCPP_WARN(node_->get_logger(), "Emergency stop triggered for all motors!");
    
    for (const auto& [joint_name, config] : motor_configs_) {
        if (!can_protocol_->emergencyStop(config.motor_id)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to emergency stop motor for joint '%s'", joint_name.c_str());
            all_success = false;
        } else {
            RCLCPP_INFO(node_->get_logger(), "Emergency stop sent to joint '%s'", joint_name.c_str());
        }
    }
    
    return all_success;
}

bool MKSMotorDriver::calibrateMotor(const std::string& joint_name) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto config_it = motor_configs_.find(joint_name);
    if (config_it == motor_configs_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint '%s' not found", joint_name.c_str());
        return false;
    }
    
    const MotorConfig& config = config_it->second;
    
    RCLCPP_WARN(node_->get_logger(), "Calibrating motor for joint '%s' - ensure motor is unloaded!", joint_name.c_str());
    
    if (!can_protocol_->calibrateMotor(config.motor_id)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to send calibration command to joint '%s'", joint_name.c_str());
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Calibration command sent to joint '%s'", joint_name.c_str());
    return true;
}

bool MKSMotorDriver::setHomeParameters(const std::string& joint_name, uint8_t trigger_level, uint8_t direction, double speed) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto config_it = motor_configs_.find(joint_name);
    if (config_it == motor_configs_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint '%s' not found", joint_name.c_str());
        return false;
    }
    
    const MotorConfig& config = config_it->second;
    
    // Convert rad/s to RPM
    double speed_rpm = std::abs(speed) * 60.0 / (2.0 * M_PI) * config.gear_ratio;
    uint16_t speed_rpm_int = static_cast<uint16_t>(std::min(speed_rpm, static_cast<double>(MKSConstants::MAX_SPEED)));
    
    if (!can_protocol_->setHomeParameters(config.motor_id, trigger_level, direction, speed_rpm_int)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set home parameters for joint '%s'", joint_name.c_str());
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Set home parameters for joint '%s': trigger=%u, dir=%u, speed=%.2f RPM", 
                joint_name.c_str(), trigger_level, direction, speed_rpm);
    return true;
}

bool MKSMotorDriver::goHome(const std::string& joint_name) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto config_it = motor_configs_.find(joint_name);
    if (config_it == motor_configs_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint '%s' not found", joint_name.c_str());
        return false;
    }
    
    const MotorConfig& config = config_it->second;
    
    if (!can_protocol_->goHome(config.motor_id)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to start homing for joint '%s'", joint_name.c_str());
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Homing started for joint '%s'", joint_name.c_str());
    return true;
}

bool MKSMotorDriver::homeAllMotors() {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    bool all_success = true;
    
    RCLCPP_INFO(node_->get_logger(), "Starting homing sequence for all motors");
    
    for (const auto& [joint_name, config] : motor_configs_) {
        if (!can_protocol_->goHome(config.motor_id)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to start homing for joint '%s'", joint_name.c_str());
            all_success = false;
        } else {
            RCLCPP_INFO(node_->get_logger(), "Homing started for joint '%s'", joint_name.c_str());
            // Add small delay between homing commands
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    return all_success;
}

bool MKSMotorDriver::setHoldingCurrentPercentage(const std::string& joint_name, uint8_t percentage) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto config_it = motor_configs_.find(joint_name);
    if (config_it == motor_configs_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint '%s' not found", joint_name.c_str());
        return false;
    }
    
    const MotorConfig& config = config_it->second;
    
    // Convert percentage to MKS format (0-8 for 10%-90%)
    uint8_t mks_percentage = 0;
    if (percentage >= 10 && percentage <= 90) {
        mks_percentage = (percentage / 10) - 1;  // 10% -> 0, 20% -> 1, ..., 90% -> 8
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Invalid holding current percentage %u for joint '%s' (must be 10-90%%)", 
                     percentage, joint_name.c_str());
        return false;
    }
    
    if (!can_protocol_->setHoldingCurrentPercentage(config.motor_id, mks_percentage)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set holding current for joint '%s'", joint_name.c_str());
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Set holding current for joint '%s': %u%%", joint_name.c_str(), percentage);
    return true;
}

bool MKSMotorDriver::setSubdivision(const std::string& joint_name, uint8_t subdivision) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto config_it = motor_configs_.find(joint_name);
    if (config_it == motor_configs_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint '%s' not found", joint_name.c_str());
        return false;
    }
    
    const MotorConfig& config = config_it->second;
    
    if (!can_protocol_->setSubdivision(config.motor_id, subdivision)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set subdivision for joint '%s'", joint_name.c_str());
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Set subdivision for joint '%s': %u", joint_name.c_str(), subdivision);
    return true;
}

bool MKSMotorDriver::restoreDefaults(const std::string& joint_name) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto config_it = motor_configs_.find(joint_name);
    if (config_it == motor_configs_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint '%s' not found", joint_name.c_str());
        return false;
    }
    
    const MotorConfig& config = config_it->second;
    
    RCLCPP_WARN(node_->get_logger(), "Restoring factory defaults for joint '%s' - motor will reboot!", joint_name.c_str());
    
    if (!can_protocol_->restoreDefaults(config.motor_id)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to restore defaults for joint '%s'", joint_name.c_str());
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Factory defaults restore command sent to joint '%s'", joint_name.c_str());
    return true;
}

bool MKSMotorDriver::setJointRelativePositionByPulses(const std::string& joint_name, double relative_position, double speed) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto config_it = motor_configs_.find(joint_name);
    if (config_it == motor_configs_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint '%s' not found", joint_name.c_str());
        return false;
    }
    
    const MotorConfig& config = config_it->second;
    
    // Apply inversion to match encoder feedback behavior
    double adjusted_position = config.inverted ? -relative_position : relative_position;
    
    // Convert to encoder pulses: radians -> degrees -> encoder steps
    double degrees = adjusted_position * (180.0 / M_PI);
    int32_t encoder_pulses = static_cast<int32_t>(std::abs(degrees * config.gear_ratio * (16384.0 / 360.0)));
    bool clockwise = (adjusted_position >= 0);
    
    // Convert speed from joint rad/s to motor RPM (apply gear ratio)
    double speed_rpm = std::abs(speed) * (30.0 / M_PI) * config.gear_ratio;
    speed_rpm = std::min(speed_rpm, 1500.0);
    uint16_t speed_rpm_int = static_cast<uint16_t>(speed_rpm);
    
    RCLCPP_DEBUG(node_->get_logger(), "Mode 1 - Relative position by pulses for '%s': %.3f rad -> %d pulses %s at %.1f RPM",
                 joint_name.c_str(), relative_position, encoder_pulses, clockwise ? "CW" : "CCW", speed_rpm);
    
    return can_protocol_->setRelativePositionByPulses(config.motor_id, encoder_pulses, speed_rpm_int, 10, clockwise);
}

bool MKSMotorDriver::setJointAbsolutePositionByPulses(const std::string& joint_name, double absolute_position, double speed) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto config_it = motor_configs_.find(joint_name);
    if (config_it == motor_configs_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint '%s' not found", joint_name.c_str());
        return false;
    }
    
    const MotorConfig& config = config_it->second;
    
    // Apply inversion to match encoder feedback behavior
    double adjusted_position = config.inverted ? -absolute_position : absolute_position;
    
    // Convert to encoder pulses: radians -> degrees -> encoder steps
    double angle_degrees = adjusted_position * (180.0 / M_PI);
    int32_t encoder_pulses = static_cast<int32_t>(angle_degrees * config.gear_ratio * (16384.0 / 360.0));
    
    // Clamp to valid range for 24-bit signed integer
    encoder_pulses = std::max(-8388607, std::min(8388607, encoder_pulses));
    
    // Convert speed from rad/s to RPM
    double speed_rpm = std::abs(speed) * (30.0 / M_PI) * config.gear_ratio;
    speed_rpm = std::min(speed_rpm, 1500.0);
    uint16_t speed_rpm_int = static_cast<uint16_t>(speed_rpm);
    
    RCLCPP_DEBUG(node_->get_logger(), "Mode 2 - Absolute position by pulses for '%s': %.3f rad -> %d pulses at %.1f RPM",
                 joint_name.c_str(), absolute_position, encoder_pulses, speed_rpm);
    
    return can_protocol_->setAbsolutePositionByPulses(config.motor_id, encoder_pulses, speed_rpm_int, 10);
}

bool MKSMotorDriver::setJointRelativePositionByAxis(const std::string& joint_name, double relative_position, double speed) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto config_it = motor_configs_.find(joint_name);
    if (config_it == motor_configs_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint '%s' not found", joint_name.c_str());
        return false;
    }
    
    const MotorConfig& config = config_it->second;
    
    // Apply inversion to match encoder feedback behavior
    double adjusted_position = config.inverted ? -relative_position : relative_position;
    
    // Convert to encoder axis units (direct encoder values)
    double degrees = adjusted_position * (180.0 / M_PI);
    int32_t axis_units = static_cast<int32_t>(degrees * config.gear_ratio * (16384.0 / 360.0));
    
    // Clamp to valid range for 24-bit signed integer
    axis_units = std::max(-8388607, std::min(8388607, axis_units));
    
    // Convert speed from rad/s to RPM
    double speed_rpm = std::abs(speed) * (30.0 / M_PI) * config.gear_ratio;
    speed_rpm = std::min(speed_rpm, 1500.0);
    uint16_t speed_rpm_int = static_cast<uint16_t>(speed_rpm);
    
    RCLCPP_DEBUG(node_->get_logger(), "Mode 3 - Relative position by axis for '%s': %.3f rad -> %d axis units at %.1f RPM",
                 joint_name.c_str(), relative_position, axis_units, speed_rpm);
    
    return can_protocol_->setRelativePositionByAxis(config.motor_id, axis_units, speed_rpm_int, 245);
}

bool MKSMotorDriver::setJointAbsolutePositionByAxis(const std::string& joint_name, double absolute_position, double speed) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto config_it = motor_configs_.find(joint_name);
    if (config_it == motor_configs_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint '%s' not found", joint_name.c_str());
        return false;
    }
    
    const MotorConfig& config = config_it->second;
    
    // Apply inversion to match encoder feedback behavior
    double adjusted_position = config.inverted ? -absolute_position : absolute_position;
    
    // Convert to encoder axis units (direct encoder values)
    double degrees = adjusted_position * (180.0 / M_PI);
    int32_t axis_units = static_cast<int32_t>(degrees * config.gear_ratio * (16384.0 / 360.0));
    
    // Clamp to valid range for 24-bit signed integer
    axis_units = std::max(-8388607, std::min(8388607, axis_units));
    
    // Convert speed from joint rad/s to motor RPM (apply gear ratio)
    double speed_rpm = std::abs(speed) * (30.0 / M_PI) * config.gear_ratio;
    speed_rpm = std::min(speed_rpm, 3000.0);
    uint16_t speed_rpm_int = static_cast<uint16_t>(speed_rpm);
    
    RCLCPP_DEBUG(node_->get_logger(), "Mode 4 - Absolute position by axis for '%s': %.3f rad -> %d axis units at %.1f RPM",
                 joint_name.c_str(), absolute_position, axis_units, speed_rpm);
    
    return can_protocol_->setAbsolutePositionByAxis(config.motor_id, axis_units, speed_rpm_int, 220);
}

bool MKSMotorDriver::setJointVelocity(const std::string& joint_name, double velocity) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto config_it = motor_configs_.find(joint_name);
    if (config_it == motor_configs_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint '%s' not found", joint_name.c_str());
        return false;
    }
    
    const MotorConfig& config = config_it->second;
    
    // Apply inversion if configured
    double adjusted_velocity = velocity;
    if (config.inverted) {
        adjusted_velocity = -adjusted_velocity;
    }
    
    // Convert from rad/s to RPM, accounting for gear ratio
    double velocity_rpm = adjusted_velocity * (30.0 / M_PI) * config.gear_ratio;
    
    // Limit to safe velocity range
    velocity_rpm = std::max(-1000.0, std::min(1000.0, velocity_rpm));
    
    int16_t velocity_rpm_int = static_cast<int16_t>(velocity_rpm);
    
    RCLCPP_DEBUG(node_->get_logger(), "Setting velocity for joint '%s': %.3f rad/s -> %d RPM",
                 joint_name.c_str(), velocity, velocity_rpm_int);
    
    return can_protocol_->setVelocity(config.motor_id, velocity_rpm_int);
}

void MKSMotorDriver::updateJointStates() {
    auto joint_names = getJointNames();
    
    for (const auto& joint_name : joint_names) {
        std::lock_guard<std::mutex> lock(motors_mutex_);
        auto config_it = motor_configs_.find(joint_name);
        if (config_it != motor_configs_.end()) {
            uint32_t motor_id = config_it->second.motor_id;
            
            // Request encoder data (most important)
            (void)can_protocol_->requestEncoderReading(motor_id);
            // Optional small pause between TX to avoid bus congestion on some hardware
            int us = inter_request_delay_us_.load();
            if (us > 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(us));
            }

            // Request IO status (for limit switches)
            // can_protocol_->requestIOStatus(motor_id);
        }
    }
}

void MKSMotorDriver::startPolling(double hz) {
    // Clamp rate to a reasonable range
    if (!(hz > 0.0)) hz = 200.0;
    if (hz < 1.0) hz = 1.0;
    if (hz > 1000.0) hz = 1000.0;
    polling_rate_hz_ = hz;

    if (polling_running_.load()) {
        // Already running; just update the rate
        RCLCPP_DEBUG(node_->get_logger(), "Polling already running, updating rate to %.1f Hz", hz);
        return;
    }

    polling_running_.store(true);
    polling_thread_ = std::thread(&MKSMotorDriver::pollingLoop, this);
    RCLCPP_INFO(node_->get_logger(), "Started background polling at %.1f Hz", hz);
}

void MKSMotorDriver::stopPolling() {
    if (polling_running_.exchange(false)) {
        if (polling_thread_.joinable()) {
            polling_thread_.join();
        }
        RCLCPP_INFO(node_->get_logger(), "Stopped background polling");
    }
}

void MKSMotorDriver::pollingLoop() {
    using clock = std::chrono::steady_clock;
    auto next = clock::now();

    while (polling_running_.load()) {
        // Perform one polling cycle
        try {
            updateJointStates();
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "Polling loop exception: %s", e.what());
        }

        // Compute next wakeup based on current rate
        double hz = polling_rate_hz_;
        if (!(hz > 0.0)) hz = 200.0;
        if (hz < 1.0) hz = 1.0;
        if (hz > 1000.0) hz = 1000.0;
        auto period = std::chrono::duration<double>(1.0 / hz);
        next = clock::now() + std::chrono::duration_cast<clock::duration>(period);

        std::this_thread::sleep_until(next);
    }
}

double MKSMotorDriver::getJointPosition(const std::string& joint_name) const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto state_it = motor_states_.find(joint_name);
    if (state_it != motor_states_.end()) {
        return state_it->second.joint_position;
    }
    
    return 0.0;
}

double MKSMotorDriver::getJointVelocity(const std::string& joint_name) const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto state_it = motor_states_.find(joint_name);
    if (state_it != motor_states_.end()) {
        return state_it->second.joint_velocity;
    }
    
    return 0.0;
}

bool MKSMotorDriver::isJointDataValid(const std::string& joint_name, double max_age_seconds) const {
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

const MotorState* MKSMotorDriver::getMotorState(const std::string& joint_name) const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto state_it = motor_states_.find(joint_name);
    if (state_it != motor_states_.end()) {
        return &state_it->second;
    }
    
    return nullptr;
}

void MKSMotorDriver::processCANMessage(const can_msgs::msg::Frame& msg) {
    // Convert ROS message to CANFrame
    CANFrame frame = CANFrame::fromRosMessage(msg);
    
    // Update statistics
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        messages_processed_++;
    }
    
    // Find which joint this motor belongs to
    std::string joint_name = findJointByMotorId(msg.id);
    if (joint_name.empty()) {
        // Not one of our motors, ignore
        std::lock_guard<std::mutex> lock(stats_mutex_);
        messages_ignored_++;
        return;
    }
    
    // Log the message for debugging
    logCANMessage(frame, "RX", "from motor " + std::to_string(msg.id));
    
    // Validate frame
    if (!frame.isValid()) {
        RCLCPP_WARN(node_->get_logger(), "Invalid CAN frame from motor %u: %s", 
                    msg.id, frame.toString().c_str());
        return;
    }
    
    // Process based on command type
    uint8_t command = frame.getCommand();
    switch (command) {
        case MKSCommands::READ_ENCODER:
            processEncoderResponse(msg.id, frame);
            break;
            
        case MKSCommands::READ_SPEED:
            processSpeedResponse(msg.id, frame);
            break;
            
        case MKSCommands::READ_IO_STATUS:
            processIOStatusResponse(msg.id, frame);
            break;
            
        case MKSCommands::ENABLE_MOTOR:
        case MKSCommands::READ_ENABLE_STATUS:
        case MKSCommands::QUERY_STATUS:
            processMotorStatusResponse(msg.id, frame);
            break;
            
        default:
            RCLCPP_DEBUG(node_->get_logger(), "Unhandled command 0x%02X from motor %u", command, msg.id);
            break;
    }
}

void MKSMotorDriver::processEncoderResponse(uint32_t motor_id, const CANFrame& frame) {
    std::string joint_name = findJointByMotorId(motor_id);
    if (joint_name.empty()) return;
    
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    // Extract encoder data
    EncoderData encoder_data = EncoderData::fromCANFrame(frame);
    
    if (encoder_data.is_valid) {
        motor_states_[joint_name].encoder = encoder_data;
        motor_states_[joint_name].last_update = node_->get_clock()->now();
        
        // Update joint state
        updateJointFromMotorData(joint_name);
        
        // Update statistics
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        encoder_updates_++;
        
        RCLCPP_DEBUG(node_->get_logger(), "Encoder update for joint '%s': %s -> joint_pos=%.6f rad", 
                     joint_name.c_str(), encoder_data.toString().c_str(), 
                     motor_states_[joint_name].joint_position);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Invalid encoder data from motor %u: %s", 
                    motor_id, encoder_data.toString().c_str());
    }
}

void MKSMotorDriver::processSpeedResponse(uint32_t motor_id, const CANFrame& frame) {
    std::string joint_name = findJointByMotorId(motor_id);
    if (joint_name.empty()) return;
    
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    // Extract speed data
    SpeedData speed_data = SpeedData::fromCANFrame(frame);
    
    if (speed_data.is_valid) {
        motor_states_[joint_name].speed = speed_data;
        motor_states_[joint_name].last_update = node_->get_clock()->now();
        
        // Update joint velocity
        auto config_it = motor_configs_.find(joint_name);
        if (config_it != motor_configs_.end()) {
            motor_states_[joint_name].joint_velocity = 
                speed_data.toJointVelocity(config_it->second.gear_ratio, config_it->second.inverted);
        }
        
        // Update statistics
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        speed_updates_++;
        
        RCLCPP_DEBUG(node_->get_logger(), "Speed update for joint '%s': %s -> joint_vel=%.6f rad/s", 
                     joint_name.c_str(), speed_data.toString().c_str(), 
                     motor_states_[joint_name].joint_velocity);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Invalid speed data from motor %u: %s", 
                    motor_id, speed_data.toString().c_str());
    }
}

void MKSMotorDriver::processIOStatusResponse(uint32_t motor_id, const CANFrame& frame) {
    std::string joint_name = findJointByMotorId(motor_id);
    if (joint_name.empty()) return;
    
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto config_it = motor_configs_.find(joint_name);
    if (config_it == motor_configs_.end()) return;
    
    // Extract IO status
    IOStatus io_status = IOStatus::fromCANFrame(frame, config_it->second.hardware_type, 
                                                config_it->second.limit_remap_enabled);
    
    if (io_status.is_valid) {
        motor_states_[joint_name].io_status = io_status;
        motor_states_[joint_name].last_update = node_->get_clock()->now();
        
        // Update statistics
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        io_updates_++;
        
        RCLCPP_DEBUG(node_->get_logger(), "IO status update for joint '%s': %s", 
                     joint_name.c_str(), io_status.toString().c_str());
        
        // Log limit switch activations
        if (io_status.limit_left || io_status.limit_right) {
            RCLCPP_WARN(node_->get_logger(), "Limit switch triggered for joint '%s': %s", 
                        joint_name.c_str(), io_status.toString().c_str());
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "Invalid IO status from motor %u: %s", 
                    motor_id, io_status.toString().c_str());
    }
}

void MKSMotorDriver::processMotorStatusResponse(uint32_t motor_id, const CANFrame& frame) {
    std::string joint_name = findJointByMotorId(motor_id);
    if (joint_name.empty()) return;
    
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    // Extract motor status based on command type
    MKSMotorStatus motor_status;
    uint8_t command = frame.getCommand();
    
    if (command == MKSCommands::ENABLE_MOTOR || command == MKSCommands::READ_ENABLE_STATUS) {
        motor_status = MKSMotorStatus::fromEnableResponse(frame);
    } else if (command == MKSCommands::QUERY_STATUS) {
        motor_status = MKSMotorStatus::fromQueryResponse(frame);
    }
    
    if (motor_status.is_valid) {
        motor_states_[joint_name].motor_status = motor_status;
        motor_states_[joint_name].last_update = node_->get_clock()->now();
        
        // Update statistics
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        status_updates_++;
        
        RCLCPP_DEBUG(node_->get_logger(), "Motor status update for joint '%s': %s", 
                     joint_name.c_str(), motor_status.toString().c_str());
        
        // Log important status changes
        if (motor_status.error) {
            RCLCPP_ERROR(node_->get_logger(), "Motor error for joint '%s': %s", 
                         joint_name.c_str(), motor_status.getStatusDescription().c_str());
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "Invalid motor status from motor %u", motor_id);
    }
}

void MKSMotorDriver::updateJointFromMotorData(const std::string& joint_name) {
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
        double raw_angle = state.encoder.angle_radians;
        state.joint_position = state.encoder.toJointAngle(config.gear_ratio, config.inverted);
        state.data_valid = true;
        
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                            "Joint '%s': raw_encoder=%.3f rad, inverted=%s, joint_pos=%.3f rad",
                            joint_name.c_str(), raw_angle, config.inverted ? "YES" : "NO", state.joint_position);
    }
    
    // Update joint velocity from speed data
    if (state.speed.is_valid) {
        state.joint_velocity = state.speed.toJointVelocity(config.gear_ratio, config.inverted);
    }
}

std::string MKSMotorDriver::findJointByMotorId(uint32_t motor_id) const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto it = id_to_joint_map_.find(motor_id);
    if (it != id_to_joint_map_.end()) {
        return it->second;
    }
    
    return "";
}

void MKSMotorDriver::logCANMessage(const CANFrame& frame, const std::string& direction, 
                                   const std::string& description) const {
    std::string desc_str = description.empty() ? "" : " (" + description + ")";
    RCLCPP_DEBUG(node_->get_logger(), "CAN %s: %s%s", 
                 direction.c_str(), frame.toString().c_str(), desc_str.c_str());
}

std::string MKSMotorDriver::getStatistics() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    
    std::ostringstream oss;
    oss << "MKSMotorDriver Stats: "
        << "processed=" << messages_processed_
        << " ignored=" << messages_ignored_
        << " encoder=" << encoder_updates_
        << " speed=" << speed_updates_
        << " io=" << io_updates_
        << " status=" << status_updates_;
    
    if (can_protocol_) {
        oss << " sent=" << can_protocol_->getFramesSent()
            << " failures=" << can_protocol_->getSendFailures();
    }
    
    return oss.str();
}

void MKSMotorDriver::resetStatistics() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    messages_processed_ = 0;
    messages_ignored_ = 0;
    encoder_updates_ = 0;
    speed_updates_ = 0;
    io_updates_ = 0;
    status_updates_ = 0;
    
    if (can_protocol_) {
        can_protocol_->resetStatistics();
    }
}

} // namespace arctos_motor_driver
