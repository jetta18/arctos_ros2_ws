#include "arctos_hardware_interface/arctos_mks_hardware_interface.hpp"
#include "arctos_hardware_interface/constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>

namespace arctos_hardware_interface
{

bool ArctosMKSHardwareInterface::readInitialEncoderValues(int timeout_ms)
{
  if (!motor_manager_ || !motor_driver_) {
    RCLCPP_ERROR(rclcpp::get_logger("ArctosMKSHardwareInterface"),
                 "Motor manager or driver not initialized");
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"),
              "Sending encoder read requests to all motors...");
  
  for (const std::string& joint_name : joint_names_) {
    uint32_t motor_id = motor_manager_->getMotorId(joint_name);
    if (motor_id == 0) {
      RCLCPP_ERROR(rclcpp::get_logger("ArctosMKSHardwareInterface"),
                   "Invalid motor ID for joint: %s", joint_name.c_str());
      return false;
    }
    
    if (!motor_driver_->requestEncoderReading(motor_id)) {
      RCLCPP_ERROR(rclcpp::get_logger("ArctosMKSHardwareInterface"),
                   "Failed to send encoder request to motor %u", motor_id);
      return false;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(constants::kBetweenEncoderRequestsMs));
  }
  
  RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"),
              "Waiting for encoder responses (timeout: %d ms)...", timeout_ms);
  
  auto start_time = std::chrono::steady_clock::now();
  int retry_count = 0;
  const int max_retries = constants::kEncoderMaxRetries;
  
  while (retry_count < max_retries) {
    rclcpp::spin_some(can_node_);
    
    bool all_valid = true;
    for (const std::string& joint_name : joint_names_) {
      if (!motor_manager_->isJointDataValid(joint_name, constants::kEncoderValidMaxAgeSec)) {
        all_valid = false;
        break;
      }
    }
    
    if (all_valid) {
      RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"),
                  "✓ All encoder values received successfully");
      return true;
    }
    
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time).count();
    
    if (elapsed > timeout_ms) {
      if (retry_count < max_retries - 1) {
        RCLCPP_WARN(rclcpp::get_logger("ArctosMKSHardwareInterface"),
                    "Timeout waiting for encoder responses. Retry %d/%d...",
                    retry_count + 1, max_retries);
        
        for (const std::string& joint_name : joint_names_) {
          if (!motor_manager_->isJointDataValid(joint_name, constants::kEncoderValidMaxAgeSec)) {
            uint32_t motor_id = motor_manager_->getMotorId(joint_name);
            motor_driver_->requestEncoderReading(motor_id);
            std::this_thread::sleep_for(std::chrono::milliseconds(constants::kBetweenEncoderRequestsMs));
          }
        }
        
        start_time = std::chrono::steady_clock::now();
        retry_count++;
      } else {
        break;
      }
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(constants::kEncoderLoopSleepMs));
  }
  
  RCLCPP_ERROR(rclcpp::get_logger("ArctosMKSHardwareInterface"),
               "Failed to read encoder values after %d retries", max_retries);
  
  for (const std::string& joint_name : joint_names_) {
    if (!motor_manager_->isJointDataValid(joint_name, constants::kEncoderValidMaxAgeSec)) {
      uint32_t motor_id = motor_manager_->getMotorId(joint_name);
      RCLCPP_ERROR(rclcpp::get_logger("ArctosMKSHardwareInterface"),
                   "  ❌ Joint '%s' (motor %u): No valid encoder data",
                   joint_name.c_str(), motor_id);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("ArctosMKSHardwareInterface"),
                  "  ✓ Joint '%s': %.4f rad",
                  joint_name.c_str(), motor_manager_->getJointPosition(joint_name));
    }
  }
  
  return false;
}

} // namespace arctos_hardware_interface
