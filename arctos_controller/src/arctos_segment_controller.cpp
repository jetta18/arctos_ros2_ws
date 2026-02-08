#include "arctos_controller/arctos_segment_controller.hpp"
#include "arctos_hardware_interface/stm32_hardware_interface.hpp"

#include <algorithm>
#include <cstring>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arctos_controller
{

static constexpr size_t MAX_JOINTS = 6;
static constexpr size_t MAX_TRAJ_POINTS = 128;

/* ------------------------------------------------------------------ */
/* Controller lifecycle                                               */
/* ------------------------------------------------------------------ */

ArctosSegmentController::ArctosSegmentController()
: controller_interface::ControllerInterface(),
  hw_interface_(nullptr),
  next_trajectory_id_(1),
  trajectory_running_seen_(false)
{
}

ArctosSegmentController::~ArctosSegmentController() = default;

controller_interface::CallbackReturn ArctosSegmentController::on_init()
{
  auto & node = *get_node();
  if (!node.has_parameter("joints"))
  {
    node.declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArctosSegmentController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joints_ = get_node()->get_parameter("joints").as_string_array();
  if (joints_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints configured");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "Configured ArctosSegmentController: %zu joints", joints_.size());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArctosSegmentController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::lock_guard<std::mutex> lock(mtx_);
  active_goal_.reset();
  trajectory_running_seen_ = false;

  /* Retrieve the HW interface pointer from the command interface */
  hw_interface_ = nullptr;
  for (auto & ci : command_interfaces_)
  {
    if (ci.get_interface_name() == "hw_interface_ptr")
    {
      double val = ci.get_value();
      arctos_hardware_interface::STM32HardwareInterface * ptr = nullptr;
      std::memcpy(&ptr, &val, sizeof(ptr));
      hw_interface_ = ptr;
      break;
    }
  }

  if (!hw_interface_ || !hw_interface_->get_protocol())
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Could not obtain HW interface pointer from command interface");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "HW interface pointer acquired");

  using namespace std::placeholders;
  const std::string action_name =
    std::string("/") + get_node()->get_name() + "/follow_joint_trajectory";
  action_server_ = rclcpp_action::create_server<FJT>(
    get_node()->get_node_base_interface(),
    get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(),
    get_node()->get_node_waitables_interface(),
    action_name,
    std::bind(&ArctosSegmentController::handle_goal, this, _1, _2),
    std::bind(&ArctosSegmentController::handle_cancel, this, _1),
    std::bind(&ArctosSegmentController::handle_accepted, this, _1));

  RCLCPP_INFO(get_node()->get_logger(),
              "Activated ArctosSegmentController, action: %s", action_name.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArctosSegmentController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (active_goal_)
    {
      auto result = std::make_shared<FJT::Result>();
      result->error_code = FJT::Result::INVALID_GOAL;
      active_goal_->abort(result);
      active_goal_.reset();
    }
  }

  action_server_.reset();
  hw_interface_ = nullptr;

  return controller_interface::CallbackReturn::SUCCESS;
}

/* ------------------------------------------------------------------ */
/* Interface configuration                                            */
/* ------------------------------------------------------------------ */

controller_interface::InterfaceConfiguration
ArctosSegmentController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.push_back("trajectory_hw/hw_interface_ptr");
  return cfg;
}

controller_interface::InterfaceConfiguration
ArctosSegmentController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joints_)
  {
    cfg.names.push_back(j + "/position");
    cfg.names.push_back(j + "/velocity");
  }
  cfg.names.push_back("trajectory_hw/system_state");
  return cfg;
}

/* ------------------------------------------------------------------ */
/* Update (called at controller_manager rate, e.g. 100 Hz)           */
/* ------------------------------------------------------------------ */

controller_interface::return_type ArctosSegmentController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (!active_goal_)
  {
    return controller_interface::return_type::OK;
  }

  /* Read system_state from the HW interface's state interface.
   * No second GET_STATE call needed — avoids UDP response cross-talk. */
  double sys_raw = 0.0;
  for (const auto & si : state_interfaces_)
  {
    if (si.get_prefix_name() == "trajectory_hw" &&
        si.get_interface_name() == "system_state")
    {
      sys_raw = si.get_value();
      break;
    }
  }

  auto sys = static_cast<uint8_t>(sys_raw);
  constexpr uint8_t SYS_IDLE = 0;
  constexpr uint8_t SYS_TRAJ_RUNNING = 3;
  constexpr uint8_t SYS_TRAJ_COMPLETE = 4;
  constexpr uint8_t SYS_ERROR = 5;

  /* Track when the STM32 has started executing the trajectory.
   * TRAJ_COMPLETE is only held for ~1ms on the STM32 (1 orchestrator cycle),
   * so at 100Hz polling we almost always miss it. Once we've seen
   * TRAJ_RUNNING, treat IDLE or TRAJ_COMPLETE as successful completion. */
  if (sys == SYS_TRAJ_RUNNING)
  {
    trajectory_running_seen_ = true;
  }

  bool completed = (sys == SYS_TRAJ_COMPLETE) ||
                   (trajectory_running_seen_ && sys == SYS_IDLE);

  if (completed)
  {
    trajectory_running_seen_ = false;
    auto result = std::make_shared<FJT::Result>();
    result->error_code = FJT::Result::SUCCESSFUL;
    active_goal_->succeed(result);
    active_goal_.reset();
    RCLCPP_INFO(get_node()->get_logger(), "Trajectory completed successfully");
  }
  else if (sys == SYS_ERROR)
  {
    trajectory_running_seen_ = false;
    auto result = std::make_shared<FJT::Result>();
    result->error_code = FJT::Result::PATH_TOLERANCE_VIOLATED;
    active_goal_->abort(result);
    active_goal_.reset();
    RCLCPP_ERROR(get_node()->get_logger(), "STM32 reported error during trajectory");
  }

  return controller_interface::return_type::OK;
}

/* ------------------------------------------------------------------ */
/* Action server callbacks                                            */
/* ------------------------------------------------------------------ */

rclcpp_action::GoalResponse ArctosSegmentController::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const FJT::Goal> goal)
{
  const auto & goal_names = goal->trajectory.joint_names;
  if (goal_names.size() != joints_.size())
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Goal joints size %zu != controller joints %zu",
                 goal_names.size(), joints_.size());
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::vector<size_t> mapping;
  mapping.reserve(joints_.size());
  for (const auto & j : joints_)
  {
    auto it = std::find(goal_names.begin(), goal_names.end(), j);
    if (it == goal_names.end())
    {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Goal missing required joint '%s'", j.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    mapping.push_back(static_cast<size_t>(std::distance(goal_names.begin(), it)));
  }

  internal_to_goal_index_ = mapping;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArctosSegmentController::handle_cancel(
  const std::shared_ptr<GoalHandleFJT> goal_handle)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (active_goal_ && goal_handle == active_goal_)
  {
    if (hw_interface_)
    {
      hw_interface_->lock_protocol();
      auto * proto = hw_interface_->get_protocol();
      if (proto)
      {
        proto->trajectory_cancel();
      }
      hw_interface_->unlock_protocol();
    }
    active_goal_.reset();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArctosSegmentController::handle_accepted(
  const std::shared_ptr<GoalHandleFJT> goal_handle)
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (active_goal_)
  {
    auto result = std::make_shared<FJT::Result>();
    result->error_code = FJT::Result::INVALID_GOAL;
    active_goal_->abort(result);
    active_goal_.reset();
  }

  const auto & traj = goal_handle->get_goal()->trajectory;
  uint32_t traj_id = next_trajectory_id_++;

  const auto & last_pt = traj.points.back();
  double planned_duration_s = last_pt.time_from_start.sec +
    last_pt.time_from_start.nanosec / 1e9;
  RCLCPP_INFO(get_node()->get_logger(),
              "Uploading trajectory id=%u with %zu points, planned_duration=%.3fs",
              traj_id, traj.points.size(), planned_duration_s);

  if (!upload_trajectory(traj, traj_id))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to upload trajectory to STM32");
    auto result = std::make_shared<FJT::Result>();
    result->error_code = FJT::Result::INVALID_GOAL;
    goal_handle->abort(result);
    return;
  }

  trajectory_running_seen_ = false;
  active_goal_ = goal_handle;
}

/* ------------------------------------------------------------------ */
/* Trajectory upload (holds protocol lock for the entire upload)      */
/* ------------------------------------------------------------------ */

bool ArctosSegmentController::upload_trajectory(
  const trajectory_msgs::msg::JointTrajectory & traj,
  uint32_t trajectory_id)
{
  if (!hw_interface_ || traj.points.empty())
  {
    return false;
  }

  hw_interface_->lock_protocol();

  auto * proto = hw_interface_->get_protocol();
  auto * uc = hw_interface_->get_unit_converter();
  if (!proto || !uc)
  {
    hw_interface_->unlock_protocol();
    return false;
  }

  auto num_points = static_cast<uint16_t>(traj.points.size());
  if (num_points > MAX_TRAJ_POINTS)
  {
    RCLCPP_WARN(get_node()->get_logger(),
                "Trajectory has %u points, capping at %zu", num_points, MAX_TRAJ_POINTS);
    num_points = static_cast<uint16_t>(MAX_TRAJ_POINTS);
  }

  if (!proto->trajectory_begin(trajectory_id, num_points))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "TRAJ_BEGIN failed");
    hw_interface_->unlock_protocol();
    return false;
  }

  for (uint16_t i = 0; i < num_points; ++i)
  {
    const auto & pt = traj.points[i];

    arctos_hardware_interface::utils::TrajectoryPoint tp{};
    tp.time_from_start_ms = static_cast<uint32_t>(
      pt.time_from_start.sec * 1000 + pt.time_from_start.nanosec / 1000000);

    for (size_t j = 0; j < joints_.size() && j < MAX_JOINTS; ++j)
    {
      size_t gi = internal_to_goal_index_[j];

      double pos_rad = (gi < pt.positions.size()) ? pt.positions[gi] : 0.0;
      tp.positions[j] = static_cast<float>(uc->rad_to_steps(pos_rad, j));

      double vel_rps = (gi < pt.velocities.size()) ? pt.velocities[gi] : 0.0;
      tp.velocities[j] = static_cast<float>(uc->rad_per_sec_to_steps_per_sec(vel_rps, j));
    }

    if (!proto->trajectory_send_point(trajectory_id, i, tp))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "TRAJ_POINT %u failed", i);
      hw_interface_->unlock_protocol();
      return false;
    }
  }

  if (!proto->trajectory_execute(trajectory_id))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "TRAJ_EXECUTE failed");
    hw_interface_->unlock_protocol();
    return false;
  }

  hw_interface_->unlock_protocol();

  RCLCPP_INFO(get_node()->get_logger(),
              "Trajectory %u uploaded and executing (%u points)", trajectory_id, num_points);
  return true;
}

}  // namespace arctos_controller

PLUGINLIB_EXPORT_CLASS(
  arctos_controller::ArctosSegmentController,
  controller_interface::ControllerInterface)
