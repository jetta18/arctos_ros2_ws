#include "arctos_controller/arctos_segment_controller.hpp"

#include <algorithm>
#include <chrono>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "controller_interface/helpers.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace arctos_controller {

ArctosSegmentController::ArctosSegmentController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn ArctosSegmentController::on_init() {
  // Declare expected parameters with guards so they are available early
  if (!get_node()->has_parameter("joints")) {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = "joints";
    std::vector<std::string> default_joints;
    get_node()->declare_parameter<std::vector<std::string>>("joints", default_joints, desc);
  }
  if (!get_node()->has_parameter("segment_rate_hz")) {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = "segment_rate_hz";
    get_node()->declare_parameter<double>("segment_rate_hz", segment_rate_hz_, desc);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArctosSegmentController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  joints_ = get_node()->get_parameter("joints").as_string_array();
  segment_rate_hz_ = get_node()->get_parameter("segment_rate_hz").as_double();
  if (joints_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints configured for ArctosSegmentController");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Configured ArctosSegmentController with %zu joints, segment_rate_hz=%.1f, node=%s",
              joints_.size(), segment_rate_hz_, get_node()->get_name());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArctosSegmentController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  std::lock_guard<std::mutex> lock(mtx_);
  active_goal_.reset();
  segments_.clear();
  next_segment_idx_ = 0;
  goal_t_start_sec_ = 0.0;

  // Order command interfaces to match joints_ using helper (output parameter)
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered;
  controller_interface::get_ordered_interfaces(
      command_interfaces_, joints_, hardware_interface::HW_IF_POSITION, ordered);
  if (ordered.size() != joints_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Missing position command interfaces: got %zu, expected %zu",
                 ordered.size(), joints_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  position_cmds_ordered_ = std::move(ordered);

  // Create FollowJointTrajectory action server only when active
  using namespace std::placeholders;
  const std::string action_name = std::string("/") + get_node()->get_name() + std::string("/follow_joint_trajectory");
  action_server_ = rclcpp_action::create_server<FJT>(
    get_node()->get_node_base_interface(),
    get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(),
    get_node()->get_node_waitables_interface(),
    action_name,
    std::bind(&ArctosSegmentController::handle_goal, this, _1, _2),
    std::bind(&ArctosSegmentController::handle_cancel, this, _1),
    std::bind(&ArctosSegmentController::handle_accepted, this, _1)
  );
  RCLCPP_INFO(get_node()->get_logger(), "Activated ArctosSegmentController, action server: %s",
              action_name.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArctosSegmentController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  std::lock_guard<std::mutex> lock(mtx_);
  active_goal_.reset();
  segments_.clear();
  action_server_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ArctosSegmentController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // position command interfaces for configured joints
  auto joints = get_node()->get_parameter("joints").as_string_array();
  cfg.names.reserve(joints.size());
  for (const auto & j : joints) {
    cfg.names.push_back(j + "/position");
  }
  return cfg;
}

controller_interface::InterfaceConfiguration ArctosSegmentController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::NONE;
  return cfg;
}

controller_interface::return_type ArctosSegmentController::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!active_goal_ || next_segment_idx_ >= segments_.size()) {
    return controller_interface::return_type::OK;
  }

  const double now_sec = time.seconds();
  if (goal_t_start_sec_ <= 0.0) {
    goal_t_start_sec_ = now_sec;
  }
  const double t_from_start = now_sec - goal_t_start_sec_;

  // Issue next segment command when its time has come
  while (next_segment_idx_ < segments_.size() && t_from_start >= segments_[next_segment_idx_].t_from_start) {
    const auto & seg = segments_[next_segment_idx_];
    if (seg.positions.size() == joints_.size() && position_cmds_ordered_.size() == joints_.size()) {
      for (size_t i = 0; i < joints_.size(); ++i) {
        position_cmds_ordered_[i].get().set_value(seg.positions[i]);
      }
    }
    next_segment_idx_++;
  }

  // If all segments issued, mark goal succeeded
  if (next_segment_idx_ >= segments_.size()) {
    auto result = std::make_shared<FJT::Result>();
    result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
    auto gh = active_goal_;
    // Use weak goal outside lock? Here it's fine; we are in update.
    gh->succeed(result);
    active_goal_.reset();
    segments_.clear();
  }

  return controller_interface::return_type::OK;
}

rclcpp_action::GoalResponse ArctosSegmentController::handle_goal(const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const FJT::Goal> goal) {
  // Validate joint set and build mapping from internal joints_ -> goal indices
  const auto & goal_names = goal->trajectory.joint_names;
  if (goal_names.size() != joints_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Goal joints size %zu differ from controller joints %zu", goal_names.size(), joints_.size());
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::vector<size_t> mapping;
  mapping.reserve(joints_.size());
  for (const auto & j : joints_) {
    auto it = std::find(goal_names.begin(), goal_names.end(), j);
    if (it == goal_names.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Goal does not contain required joint '%s'", j.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    mapping.push_back(static_cast<size_t>(std::distance(goal_names.begin(), it)));
  }
  // Store mapping for use when goal is accepted
  internal_to_goal_index_ = mapping;
  goal_joint_names_ = goal_names;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArctosSegmentController::handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (active_goal_ && goal_handle == active_goal_) {
    segments_.clear();
    next_segment_idx_ = 0;
    active_goal_.reset();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArctosSegmentController::handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle) {
  std::lock_guard<std::mutex> lock(mtx_);
  active_goal_ = goal_handle;
  segments_.clear();
  next_segment_idx_ = 0;
  goal_t_start_sec_ = 0.0;
  build_segments_from_goal(goal_handle->get_goal()->trajectory);
}

void ArctosSegmentController::build_segments_from_goal(const trajectory_msgs::msg::JointTrajectory & traj) {
  // Fallback: if no points, do nothing
  if (traj.points.empty()) {
    return;
  }
  if (internal_to_goal_index_.size() != joints_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Internal mapping not initialized; rejecting goal");
    return;
  }
  // Determine start time reference
  // Use trajectory.header.stamp if in the future, else start at next update
  // Here we simply rebuild relative times from first point
  const double rate_dt = segment_rate_hz_ > 0.0 ? (1.0 / segment_rate_hz_) : 0.05;

  // Build coarse segments by downsampling points at segment_rate_hz_
  double next_t = 0.0;
  for (size_t p = 0; p < traj.points.size(); ++p) {
    const auto & pt = traj.points[p];
    const double t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
    if (p == traj.points.size() - 1 || t >= next_t) {
      Segment s;
      s.t_from_start = t;
      s.positions.resize(joints_.size());
      if (pt.positions.size() != goal_joint_names_.size()) {
        RCLCPP_WARN(get_node()->get_logger(), "Point positions size %zu does not match goal joints size %zu; skipping point",
                    pt.positions.size(), goal_joint_names_.size());
        continue;
      }
      for (size_t i = 0; i < joints_.size(); ++i) {
        const size_t gi = internal_to_goal_index_[i];
        s.positions[i] = pt.positions[gi];
      }
      segments_.push_back(std::move(s));
      next_t += rate_dt;
    }
  }
  // Ensure last point is included
  const auto & last_pt = traj.points.back();
  Segment last;
  last.t_from_start = last_pt.time_from_start.sec + last_pt.time_from_start.nanosec * 1e-9;
  last.positions.resize(joints_.size());
  if (last_pt.positions.size() == goal_joint_names_.size()) {
    for (size_t i = 0; i < joints_.size(); ++i) {
      const size_t gi = internal_to_goal_index_[i];
      last.positions[i] = last_pt.positions[gi];
    }
  }
  if (segments_.empty() || segments_.back().t_from_start < last.t_from_start) {
    segments_.push_back(std::move(last));
  }

  RCLCPP_INFO(get_node()->get_logger(), "Built %zu segments from goal with %zu points", segments_.size(), traj.points.size());
}

} // namespace arctos_controller

PLUGINLIB_EXPORT_CLASS(arctos_controller::ArctosSegmentController, controller_interface::ControllerInterface)
