#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace arctos_hardware_interface { class STM32HardwareInterface; }

namespace arctos_controller
{

/**
 * ArctosSegmentController
 *
 * Receives FollowJointTrajectory goals from MoveIt / RViz,
 * converts trajectory points from radians to steps, uploads them
 * to the STM32 via the shared protocol owned by
 * STM32HardwareInterface, and monitors execution via GET_STATE.
 *
 * Uses a single command interface ("trajectory_hw/hw_interface_ptr")
 * to obtain a pointer to the HW interface, then calls
 * lock_protocol() / get_protocol() / get_unit_converter() on it.
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
  using FJT = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FJT>;

  rclcpp_action::Server<FJT>::SharedPtr action_server_;

  std::vector<std::string> joints_;

  /* Pointer to the HW interface (obtained from command interface) */
  arctos_hardware_interface::STM32HardwareInterface * hw_interface_;

  /* Goal tracking */
  std::mutex mtx_;
  std::shared_ptr<GoalHandleFJT> active_goal_;
  uint32_t next_trajectory_id_;
  bool trajectory_running_seen_;

  /* Joint name mapping: controller joints_ order -> goal joint_names order */
  std::vector<size_t> internal_to_goal_index_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FJT::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFJT> goal_handle);
  void handle_accepted(
    const std::shared_ptr<GoalHandleFJT> goal_handle);

  bool upload_trajectory(
    const trajectory_msgs::msg::JointTrajectory & traj,
    uint32_t trajectory_id);
};

}  // namespace arctos_controller
