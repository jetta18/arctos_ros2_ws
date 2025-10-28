#include "arctos_motor_driver/mks_motor_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <deque>

namespace arctos_motor_driver {

// ROS2-to-MKS transport adapter
class ROS2CanTransport : public mks::ICanTransport {
public:
    explicit ROS2CanTransport(rclcpp::Node::SharedPtr node)
        : node_(std::move(node)) {}

    ~ROS2CanTransport() override { close(); }

    bool open(const std::string& ifname) override {
        // Use the underlying SocketCanTransport from mks_motor_driver
        core_transport_ = std::make_shared<mks::SocketCanTransport>();
        if (!core_transport_->open(ifname)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open CAN interface '%s'", ifname.c_str());
            return false;
        }

        running_.store(true, std::memory_order_release);
        receive_thread_ = std::thread(&ROS2CanTransport::receiveLoop, this);

        RCLCPP_INFO(node_->get_logger(), "CAN transport opened on '%s'", ifname.c_str());
        return true;
    }

    void close() override {
        if (running_.exchange(false, std::memory_order_acq_rel)) {
            queue_cv_.notify_all();
            if (receive_thread_.joinable()) {
                receive_thread_.join();
            }
        }
        if (core_transport_) {
            core_transport_->close();
            core_transport_.reset();
        }

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            frame_queue_.clear();
        }
    }

    bool write(const mks::CanFrame& frame) override {
        if (!core_transport_) return false;
        return core_transport_->write(frame);
    }

    std::optional<mks::CanFrame> read(int timeout_ms) override {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        if (timeout_ms < 0) {
            queue_cv_.wait(lock, [this] { return !frame_queue_.empty() || !running_.load(); });
        } else {
            if (!queue_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                    [this] { return !frame_queue_.empty() || !running_.load(); })) {
                return std::nullopt;
            }
        }

        if (frame_queue_.empty()) {
            return std::nullopt;
        }

        mks::CanFrame frame = frame_queue_.front();
        frame_queue_.pop_front();
        return frame;
    }

    void registerCallback(std::function<void(const mks::CanFrame&)> cb) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callback_ = std::move(cb);
    }

private:
    void receiveLoop() {
        while (running_.load(std::memory_order_acquire)) {
            auto frame = core_transport_->read(100);
            if (!frame) {
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                frame_queue_.push_back(*frame);
            }
            queue_cv_.notify_all();

            std::lock_guard<std::mutex> cb_lock(callback_mutex_);
            if (callback_) {
                callback_(*frame);
            }
        }
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<mks::SocketCanTransport> core_transport_;
    std::thread receive_thread_;
    std::atomic<bool> running_{false};

    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::deque<mks::CanFrame> frame_queue_;

    std::mutex callback_mutex_;
    std::function<void(const mks::CanFrame&)> callback_;
};

MKSMotorDriver::MKSMotorDriver(rclcpp::Node::SharedPtr node, const std::string& can_interface_name)
    : node_(node) {
    
    // Create ROS2-aware CAN transport
    auto ros_transport = std::make_shared<ROS2CanTransport>(node_);
    if (!ros_transport->open(can_interface_name)) {
        throw std::runtime_error("Failed to open CAN interface");
    }
    
    // Register callback for incoming messages
    ros_transport->registerCallback([this](const mks::CanFrame& frame) {
        this->processCANFrame(frame);
    });
    
    transport_ = ros_transport;
    
    // Create per-motor-ID drivers (we'll manage them dynamically)
    // For now, create a default driver with ID 0x01
    drivers_[0x01] = std::make_shared<mks::MksDriver>(transport_, 0x01);
    
    RCLCPP_INFO(node_->get_logger(), "MKSMotorDriver initialized on CAN interface '%s'", can_interface_name.c_str());
}

MKSMotorDriver::~MKSMotorDriver() {
    drivers_.clear();
    if (transport_) {
        transport_->close();
    }
    RCLCPP_INFO(node_->get_logger(), "MKSMotorDriver destroyed");
}

std::shared_ptr<mks::MksDriver> MKSMotorDriver::getDriver(uint32_t motor_id) {
    std::lock_guard<std::mutex> lock(drivers_mutex_);
    auto it = drivers_.find(motor_id);
    if (it == drivers_.end()) {
        // Create new driver for this motor ID
        auto driver = std::make_shared<mks::MksDriver>(transport_, static_cast<uint16_t>(motor_id));
        drivers_[motor_id] = driver;
        RCLCPP_DEBUG(node_->get_logger(), "Created driver for motor ID 0x%02X", motor_id);
        return driver;
    }
    return it->second;
}

// Configuration Commands
bool MKSMotorDriver::calibrateMotor(uint32_t motor_id) {
    auto driver = getDriver(motor_id);
    auto result = driver->calibrate();
    return result && *result == mks::Result::Success;
}

bool MKSMotorDriver::setWorkMode(uint32_t motor_id, uint8_t mode) {
    auto driver = getDriver(motor_id);
    auto result = driver->set_mode(static_cast<mks::WorkMode>(mode));
    return result && *result == mks::Result::Success;
}

bool MKSMotorDriver::setWorkingCurrent(uint32_t motor_id, uint16_t current_ma) {
    auto driver = getDriver(motor_id);
    auto result = driver->set_work_current_ma(current_ma);
    return result && *result == mks::Result::Success;
}

bool MKSMotorDriver::setHoldingCurrentPercentage(uint32_t motor_id, uint8_t percentage) {
    auto driver = getDriver(motor_id);
    auto result = driver->set_hold_current_percent(percentage);
    return result && *result == mks::Result::Success;
}

bool MKSMotorDriver::setSubdivision(uint32_t motor_id, uint8_t subdivision) {
    auto driver = getDriver(motor_id);
    auto result = driver->set_microstep(subdivision);
    return result && *result == mks::Result::Success;
}

bool MKSMotorDriver::restoreDefaults(uint32_t motor_id) {
    auto driver = getDriver(motor_id);
    auto result = driver->restore_defaults();
    return result && *result == mks::Result::Success;
}

// Homing Commands
bool MKSMotorDriver::setHomeParameters(uint32_t motor_id, uint8_t trigger_level, uint8_t direction, uint16_t speed_rpm, uint8_t enable_limit) {
    auto driver = getDriver(motor_id);
    mks::Direction dir = (direction == 0) ? mks::Direction::CW : mks::Direction::CCW;
    auto result = driver->set_home_params(trigger_level, dir, speed_rpm, enable_limit != 0);
    return result && *result == mks::Result::Success;
}

bool MKSMotorDriver::goHome(uint32_t motor_id) {
    auto driver = getDriver(motor_id);
    auto result = driver->go_home();
    return result && *result == mks::Result::Success;
}

bool MKSMotorDriver::setZeroPosition(uint32_t motor_id) {
    auto driver = getDriver(motor_id);
    auto result = driver->set_current_axis_zero();
    return result && *result == mks::Result::Success;
}

bool MKSMotorDriver::setLimitPortRemap(uint32_t motor_id, bool enable) {
    auto driver = getDriver(motor_id);
    auto result = driver->set_limit_remap(enable);
    return result && *result == mks::Result::Success;
}

// Motor Control Commands
bool MKSMotorDriver::queryMotorStatus(uint32_t motor_id) {
    auto driver = getDriver(motor_id);
    auto status = driver->query_status();
    return status.has_value();
}

bool MKSMotorDriver::enableMotor(uint32_t motor_id, bool enable) {
    auto driver = getDriver(motor_id);
    auto result = driver->enable_driver(enable);
    return result && *result == mks::Result::Success;
}

bool MKSMotorDriver::emergencyStop(uint32_t motor_id) {
    auto driver = getDriver(motor_id);
    auto result = driver->emergency_stop();
    return result && *result == mks::Result::Success;
}

bool MKSMotorDriver::setVelocity(uint32_t motor_id, int16_t speed_rpm, uint8_t acceleration) {
    auto driver = getDriver(motor_id);
    mks::Direction dir = (speed_rpm >= 0) ? mks::Direction::CCW : mks::Direction::CW;
    mks::SpeedAcc sa{static_cast<uint16_t>(std::abs(speed_rpm)), acceleration};
    auto result = driver->run_speed(dir, sa);
    return result && *result == mks::Result::Success;
}

// Position Control Commands
bool MKSMotorDriver::setRelativePositionByPulses(uint32_t motor_id, uint32_t pulses, uint16_t speed_rpm, uint8_t acceleration, bool clockwise) {
    auto driver = getDriver(motor_id);
    mks::Direction dir = clockwise ? mks::Direction::CW : mks::Direction::CCW;
    mks::SpeedAcc sa{speed_rpm, acceleration};
    auto status = driver->move_relative_pulses(dir, sa, pulses);
    return status && (*status == 1 || *status == 2); // 1=starting, 2=complete
}

bool MKSMotorDriver::setAbsolutePositionByPulses(uint32_t motor_id, int32_t abs_pulses, uint16_t speed_rpm, uint8_t acceleration) {
    auto driver = getDriver(motor_id);
    mks::SpeedAcc sa{speed_rpm, acceleration};
    auto status = driver->move_absolute_pulses(sa, abs_pulses);
    return status && (*status == 1 || *status == 2);
}

bool MKSMotorDriver::setRelativePositionByAxis(uint32_t motor_id, int32_t rel_axis, uint16_t speed_rpm, uint8_t acceleration) {
    auto driver = getDriver(motor_id);
    mks::SpeedAcc sa{speed_rpm, acceleration};
    auto status = driver->move_relative_axis(sa, rel_axis);
    return status && (*status == 1 || *status == 2);
}

bool MKSMotorDriver::setAbsolutePositionByAxis(uint32_t motor_id, int32_t abs_axis, uint16_t speed_rpm, uint8_t acceleration) {
    auto driver = getDriver(motor_id);
    mks::SpeedAcc sa{speed_rpm, acceleration};
    auto status = driver->move_absolute_axis(sa, abs_axis);
    return status && (*status == 1 || *status == 2);
}

// Read Parameter Commands
bool MKSMotorDriver::requestEncoderReading(uint32_t motor_id) {
    auto driver = getDriver(motor_id);
    auto encoder = driver->read_encoder_addition();
    return encoder.has_value();
}

bool MKSMotorDriver::requestSpeedReading(uint32_t motor_id) {
    auto driver = getDriver(motor_id);
    auto speed = driver->read_speed_rpm();
    return speed.has_value();
}

bool MKSMotorDriver::requestIOStatus(uint32_t motor_id) {
    auto driver = getDriver(motor_id);
    auto io = driver->read_io_status();
    return io.has_value();
}

// CAN Message Processing
void MKSMotorDriver::registerResponseCallback(const ResponseCallback& callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    response_callback_ = callback;
    RCLCPP_DEBUG(node_->get_logger(), "Response callback registered");
}

void MKSMotorDriver::processCANMessage(const can_msgs::msg::Frame& msg) {
    mks::CanFrame frame{};
    frame.id = msg.id;
    frame.dlc = msg.dlc;
    std::copy(msg.data.begin(), msg.data.begin() + std::min(8, static_cast<int>(msg.dlc)), frame.data);
    processCANFrame(frame);
}

void MKSMotorDriver::processCANFrame(const mks::CanFrame& frame) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (response_callback_) {
        // Convert mks::CanFrame to legacy CANFrame for compatibility
        CANFrame legacy_frame{};
        legacy_frame.id = frame.id;
        legacy_frame.dlc = frame.dlc;
        std::copy(std::begin(frame.data), std::begin(frame.data) + frame.dlc, legacy_frame.data.begin());
        
        response_callback_(frame.id, legacy_frame);
    }
    
    RCLCPP_DEBUG(node_->get_logger(), "Received CAN frame: ID=0x%X, DLC=%u, CMD=0x%02X", 
                 frame.id, frame.dlc, frame.dlc > 0 ? frame.data[0] : 0);
}

} // namespace arctos_motor_driver
