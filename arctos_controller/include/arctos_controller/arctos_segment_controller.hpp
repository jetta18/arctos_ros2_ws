#pragma once

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <functional>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace arctos_controller {

class ArctosSegmentController : public controller_interface::ControllerInterface {
public:
  ArctosSegmentController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using FJT = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FJT>;

  rclcpp_action::Server<FJT>::SharedPtr action_server_;

  std::vector<std::string> joints_;
  double segment_rate_hz_ = 20.0; // minimal segment sampling
  double goal_t_start_sec_ = 0.0;

  struct Segment {
    double t_from_start; // seconds
    std::vector<double> positions;
    std::vector<double> velocities; // optional, not used initially
  };

  std::mutex mtx_;
  std::shared_ptr<GoalHandleFJT> active_goal_;
  std::vector<Segment> segments_;
  size_t next_segment_idx_ = 0;

  // Ordered command interfaces aligned to joints_
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> position_cmds_ordered_;

  // Mapping from controller joints_ order to indices in the incoming goal's joint_names
  std::vector<size_t> internal_to_goal_index_;
  std::vector<std::string> goal_joint_names_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FJT::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);

  void build_segments_from_goal(const trajectory_msgs::msg::JointTrajectory & traj);
};

} // namespace arctos_controller
