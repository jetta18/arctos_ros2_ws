#pragma once

#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/gripper_command.hpp"

namespace arctos_controller
{

/**
 * ArctosGripperController
 *
 * Receives GripperCommand goals from MoveIt, writes the target position
 * (meters) to the Left_jaw_joint / Right_jaw_joint command interfaces.
 * The hardware interface converts meters → servo pulse and sends
 * CMD_SET_SERVO to the STM32.
 *
 * Both jaw joints are driven by a single servo (parallel gripper),
 * so the same position value is written to both command interfaces.
 */
class ArctosGripperController : public controller_interface::ControllerInterface
{
public:
  ArctosGripperController();
  ~ArctosGripperController() override;

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
  using GripperAction = control_msgs::action::GripperCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GripperAction>;

  rclcpp_action::Server<GripperAction>::SharedPtr action_server_;

  std::string joint_;
  double max_opening_m_;
  double position_tolerance_m_;

  std::shared_ptr<GoalHandle> active_goal_;
  double goal_position_;
  bool goal_active_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GripperAction::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle);

  void publish_feedback(double current_position);
};

}  // namespace arctos_controller
