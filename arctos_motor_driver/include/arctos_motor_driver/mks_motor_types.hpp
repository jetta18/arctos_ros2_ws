#ifndef ARCTOS_MOTOR_DRIVER_MKS_MOTOR_TYPES_HPP_
#define ARCTOS_MOTOR_DRIVER_MKS_MOTOR_TYPES_HPP_

#include <cstdint>
#include <string>

#include "rclcpp/time.hpp"
#include "arctos_motor_driver/mks_data_types.hpp"

namespace arctos_motor_driver {

/**
 * @brief Motor configuration for a single joint.
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
 * @brief Current state of a motor/joint.
 */
struct MotorState {
    EncoderData encoder;        ///< Latest encoder data
    SpeedData speed;            ///< Latest speed data
    IOStatus io_status;         ///< Latest IO status
    MKSMotorStatus motor_status;///< Latest motor status

    double joint_position;      ///< Joint position in radians
    double joint_velocity;      ///< Joint velocity in rad/s

    rclcpp::Time last_update;   ///< Timestamp of last update
    rclcpp::Time last_command;  ///< Timestamp of last command
    bool data_valid;            ///< Whether the cached data is valid

    MotorState()
        : joint_position(0.0), joint_velocity(0.0), data_valid(false) {}
};

} // namespace arctos_motor_driver

#endif // ARCTOS_MOTOR_DRIVER_MKS_MOTOR_TYPES_HPP_
