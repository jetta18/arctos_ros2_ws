#include <memory>
#include <string>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "arctos_motor_driver/mks_motor_driver.hpp"
#include "arctos_motor_driver/mks_data_types.hpp"
#include "arctos_motor_driver/srv/mks_motor_config.hpp"
#include "arctos_motor_driver/srv/mks_motor_read.hpp"
#include "arctos_motor_driver/srv/mks_connection.hpp"

using namespace arctos_motor_driver;

class MKSConfigService : public rclcpp::Node {
public:
    MKSConfigService()
        : Node("mks_config_service"),
          response_received_(false),
          waiting_motor_id_(0),
          driver_initialized_(false) {
        
        this->declare_parameter("can_interface", "can0");
        this->declare_parameter("response_timeout_ms", 500);
    }
    
    void initialize() {
        // Don't initialize CAN driver here - wait for explicit connection
        
        config_service_ = this->create_service<arctos_motor_driver::srv::MKSMotorConfig>(
            "mks_motor_config",
            std::bind(&MKSConfigService::handleConfigRequest, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        read_service_ = this->create_service<arctos_motor_driver::srv::MKSMotorRead>(
            "mks_motor_read",
            std::bind(&MKSConfigService::handleReadRequest, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        connection_service_ = this->create_service<arctos_motor_driver::srv::MKSConnection>(
            "mks_connection",
            std::bind(&MKSConfigService::handleConnectionRequest, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        RCLCPP_INFO(this->get_logger(), "MKS Config Service ready (CAN not connected)");
    }

private:
    void handleConfigRequest(
        const std::shared_ptr<arctos_motor_driver::srv::MKSMotorConfig::Request> request,
        std::shared_ptr<arctos_motor_driver::srv::MKSMotorConfig::Response> response) {
        
        RCLCPP_INFO(this->get_logger(), "Config request: motor_id=%u, command=%s", 
                    request->motor_id, request->command_type.c_str());
        
        if (!driver_initialized_) {
            response->success = false;
            response->message = "Not connected to CAN interface";
            response->status_code = 2;
            RCLCPP_WARN(this->get_logger(), "Config request rejected: CAN not connected");
            return;
        }
        
        bool success = false;
        std::string message = "Unknown command";
        
        try {
            if (request->command_type == "calibrate") {
                success = driver_->calibrateMotor(request->motor_id);
                message = success ? "Motor calibration started" : "Failed to start calibration";
                
            } else if (request->command_type == "set_work_mode") {
                success = driver_->setWorkMode(request->motor_id, request->work_mode);
                message = success ? "Work mode set successfully" : "Failed to set work mode";
                
            } else if (request->command_type == "set_current") {
                success = driver_->setWorkingCurrent(request->motor_id, request->working_current_ma);
                message = success ? "Working current set successfully" : "Failed to set working current";
                
            } else if (request->command_type == "set_holding_current") {
                success = driver_->setHoldingCurrentPercentage(request->motor_id, request->holding_current_pct);
                message = success ? "Holding current set successfully" : "Failed to set holding current";
                
            } else if (request->command_type == "set_subdivision") {
                success = driver_->setSubdivision(request->motor_id, request->subdivision);
                message = success ? "Subdivision set successfully" : "Failed to set subdivision";
                
            } else if (request->command_type == "restore_defaults") {
                success = driver_->restoreDefaults(request->motor_id);
                message = success ? "Defaults restored successfully" : "Failed to restore defaults";
                
            } else if (request->command_type == "set_home_params") {
                success = driver_->setHomeParameters(
                    request->motor_id,
                    request->home_trigger_level,
                    request->home_direction,
                    request->home_speed_rpm,
                    request->home_enable_limit ? 1 : 0
                );
                message = success ? "Home parameters set successfully" : "Failed to set home parameters";
                
            } else if (request->command_type == "go_home") {
                success = driver_->goHome(request->motor_id);
                message = success ? "Homing started" : "Failed to start homing";
                
            } else if (request->command_type == "set_zero") {
                success = driver_->setZeroPosition(request->motor_id);
                message = success ? "Zero position set successfully" : "Failed to set zero position";
                
            } else if (request->command_type == "set_limit_remap") {
                success = driver_->setLimitPortRemap(request->motor_id, request->limit_remap_enable);
                message = success ? "Limit port remap set successfully" : "Failed to set limit port remap";
                
            } else if (request->command_type == "enable") {
                success = driver_->enableMotor(request->motor_id, request->motor_enable);
                message = success ? (request->motor_enable ? "Motor enabled" : "Motor disabled") : "Failed to enable/disable motor";
                
            } else if (request->command_type == "query_status") {
                success = driver_->queryMotorStatus(request->motor_id);
                message = success ? "Status query sent" : "Failed to query status";
            }
            
        } catch (const std::exception& e) {
            success = false;
            message = std::string("Exception: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Exception in config request: %s", e.what());
        }
        
        response->success = success;
        response->message = message;
        response->status_code = 0;
        
        RCLCPP_INFO(this->get_logger(), "Config response: success=%d, message=%s", success, message.c_str());
    }
    
    void handleReadRequest(
        const std::shared_ptr<arctos_motor_driver::srv::MKSMotorRead::Request> request,
        std::shared_ptr<arctos_motor_driver::srv::MKSMotorRead::Response> response) {
        
        RCLCPP_INFO(this->get_logger(), "Read request: motor_id=%u, read_type=%s", 
                    request->motor_id, request->read_type.c_str());
        
        if (!driver_initialized_) {
            response->success = false;
            response->message = "Not connected to CAN interface";
            return;
        }
        
        bool success = false;
        std::string message = "Unknown read type";
        
        try {
            // Reset response state
            {
                std::lock_guard<std::mutex> lock(response_mutex_);
                response_received_ = false;
                waiting_motor_id_ = request->motor_id;
                last_frame_ = CANFrame();
            }
            
            // Send CAN request
            if (request->read_type == "encoder") {
                success = driver_->requestEncoderReading(request->motor_id);
                
            } else if (request->read_type == "speed") {
                success = driver_->requestSpeedReading(request->motor_id);
                
            } else if (request->read_type == "io_status") {
                success = driver_->requestIOStatus(request->motor_id);
                
            } else if (request->read_type == "motor_status") {
                success = driver_->queryMotorStatus(request->motor_id);
            }
            
            if (!success) {
                response->success = false;
                response->message = "Failed to send CAN request";
                return;
            }
            
            // Wait for CAN response with timeout
            std::unique_lock<std::mutex> lock(response_mutex_);
            bool received = response_cv_.wait_for(
                lock,
                std::chrono::milliseconds(response_timeout_ms_),
                [this]() { return response_received_; }
            );
            
            if (!received) {
                response->success = false;
                response->message = "Timeout waiting for motor response";
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for motor %u response", request->motor_id);
                return;
            }
            
            // Parse response based on type
            if (request->read_type == "encoder") {
                EncoderData encoder = EncoderData::fromCANFrame(last_frame_);
                if (encoder.is_valid) {
                    response->success = true;
                    response->message = "Encoder data received";
                    response->encoder_raw_value = encoder.raw_value;
                    response->encoder_angle_degrees = encoder.angle_degrees;
                    response->encoder_angle_radians = encoder.angle_radians;
                    RCLCPP_INFO(this->get_logger(), "Encoder: raw=%ld, deg=%.2f, rad=%.4f",
                               encoder.raw_value, encoder.angle_degrees, encoder.angle_radians);
                } else {
                    response->success = false;
                    response->message = "Invalid encoder data received";
                }
                
            } else if (request->read_type == "speed") {
                SpeedData speed = SpeedData::fromCANFrame(last_frame_);
                if (speed.is_valid) {
                    response->success = true;
                    response->message = "Speed data received";
                    response->speed_rpm = speed.rpm;
                    response->speed_rad_per_sec = speed.rad_per_sec;
                } else {
                    response->success = false;
                    response->message = "Invalid speed data received";
                }
                
            } else if (request->read_type == "io_status") {
                IOStatus io = IOStatus::fromCANFrame(last_frame_, "MKS_42D", false);
                if (io.is_valid) {
                    response->success = true;
                    response->message = "IO status received";
                    response->io_in1 = io.in1;
                    response->io_in2 = io.in2;
                    response->io_out1 = io.out1;
                    response->io_out2 = io.out2;
                    response->limit_left = io.limit_left;
                    response->limit_right = io.limit_right;
                    response->stall_detected = io.stall_detected;
                } else {
                    response->success = false;
                    response->message = "Invalid IO status received";
                }
            }
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Exception: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Exception in read request: %s", e.what());
        }
        
        RCLCPP_INFO(this->get_logger(), "Read response: success=%d, message=%s", 
                   response->success, response->message.c_str());
    }
    
    void handleConnectionRequest(
        const std::shared_ptr<arctos_motor_driver::srv::MKSConnection::Request> request,
        std::shared_ptr<arctos_motor_driver::srv::MKSConnection::Response> response) {
        
        if (request->connect) {
            // Connect request
            if (driver_initialized_) {
                response->success = true;
                response->message = "Already connected";
                response->status_code = 0;
                return;
            }
            
            try {
                std::string can_interface = this->get_parameter("can_interface").as_string();
                response_timeout_ms_ = this->get_parameter("response_timeout_ms").as_int();
                
                driver_ = std::make_shared<MKSMotorDriver>(
                    this->shared_from_this(),
                    can_interface
                );
                
                // Register callback for CAN responses
                driver_->registerResponseCallback(
                    std::bind(&MKSConfigService::handleCANResponse, this, 
                             std::placeholders::_1, std::placeholders::_2)
                );
                
                driver_initialized_ = true;
                response->success = true;
                response->message = "Successfully connected to CAN interface";
                response->status_code = 0;
                
                RCLCPP_INFO(this->get_logger(), "MKS Motor Driver connected on %s", can_interface.c_str());
            } catch (const std::exception& e) {
                driver_initialized_ = false;
                response->success = false;
                response->message = std::string("Failed to connect: ") + e.what();
                response->status_code = 1;
                
                RCLCPP_ERROR(this->get_logger(), "Failed to connect to CAN interface: %s", e.what());
            }
        } else {
            // Disconnect request
            if (!driver_initialized_) {
                response->success = true;
                response->message = "Already disconnected";
                response->status_code = 0;
                return;
            }
            
            try {
                driver_.reset();
                driver_initialized_ = false;
                response->success = true;
                response->message = "Successfully disconnected from CAN interface";
                response->status_code = 0;
                
                RCLCPP_INFO(this->get_logger(), "MKS Motor Driver disconnected");
            } catch (const std::exception& e) {
                response->success = false;
                response->message = std::string("Failed to disconnect: ") + e.what();
                response->status_code = 1;
                
                RCLCPP_ERROR(this->get_logger(), "Failed to disconnect: %s", e.what());
            }
        }
    }
    
    void handleCANResponse(uint32_t motor_id, const CANFrame& frame) {
        std::lock_guard<std::mutex> lock(response_mutex_);
        
        // Only process if we're waiting for this motor's response
        if (motor_id == waiting_motor_id_ && !response_received_) {
            last_frame_ = frame;
            response_received_ = true;
            response_cv_.notify_one();
            
            RCLCPP_DEBUG(this->get_logger(), "Received CAN response from motor %u, cmd=0x%02X",
                        motor_id, frame.data[0]);
        }
    }
    
    std::shared_ptr<MKSMotorDriver> driver_;
    rclcpp::Service<arctos_motor_driver::srv::MKSMotorConfig>::SharedPtr config_service_;
    rclcpp::Service<arctos_motor_driver::srv::MKSMotorRead>::SharedPtr read_service_;
    rclcpp::Service<arctos_motor_driver::srv::MKSConnection>::SharedPtr connection_service_;
    
    // Connection state
    bool driver_initialized_;
    
    // Response synchronization
    std::mutex response_mutex_;
    std::condition_variable response_cv_;
    bool response_received_;
    uint32_t waiting_motor_id_;
    CANFrame last_frame_;
    int response_timeout_ms_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<MKSConfigService>();
        node->initialize();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("mks_config_service"), "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
