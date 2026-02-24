#pragma once

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"

namespace arctos_controller
{

/**
 * ArctosSegmentController
 *
 * Lightweight controller that claims state interfaces for the arm joints
 * and the trajectory_hw/system_state interface. This keeps the HW interface
 * active in controller_manager.
 *
 * Trajectory execution is handled by the arctos_bridge node's
 * FollowJointTrajectory action server. MoveIt sends goals directly
 * to /arctos_controller/follow_joint_trajectory on the bridge.
 */
class ArctosSegmentController : public controller_interface::ControllerInterface
{
public:
  ArctosSegmentController();
  ~ArctosSegmentController() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<std::string> joints_;
};

}  // namespace arctos_controller
