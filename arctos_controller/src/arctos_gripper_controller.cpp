#include "arctos_controller/arctos_gripper_controller.hpp"

#include <cmath>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arctos_controller
{

ArctosGripperController::ArctosGripperController()
: controller_interface::ControllerInterface(),
  max_opening_m_(0.015),
  position_tolerance_m_(0.001),
  goal_position_(0.0),
  goal_active_(false)
{
}

ArctosGripperController::~ArctosGripperController() = default;

controller_interface::CallbackReturn ArctosGripperController::on_init()
{
  auto & node = *get_node();
  if (!node.has_parameter("joint"))
  {
    node.declare_parameter<std::string>("joint", "Left_jaw_joint");
  }
  if (!node.has_parameter("max_opening_m"))
  {
    node.declare_parameter<double>("max_opening_m", 0.015);
  }
  if (!node.has_parameter("position_tolerance_m"))
  {
    node.declare_parameter<double>("position_tolerance_m", 0.001);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArctosGripperController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_ = get_node()->get_parameter("joint").as_string();
  max_opening_m_ = get_node()->get_parameter("max_opening_m").as_double();
  position_tolerance_m_ = get_node()->get_parameter("position_tolerance_m").as_double();

  RCLCPP_INFO(get_node()->get_logger(),
              "Configured ArctosGripperController: joint=%s, max_opening=%.4f m",
              joint_.c_str(), max_opening_m_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArctosGripperController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_goal_.reset();
  goal_active_ = false;

  using namespace std::placeholders;
  const std::string action_name =
    std::string("/") + get_node()->get_name() + "/gripper_cmd";
  action_server_ = rclcpp_action::create_server<GripperAction>(
    get_node()->get_node_base_interface(),
    get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(),
    get_node()->get_node_waitables_interface(),
    action_name,
    std::bind(&ArctosGripperController::handle_goal, this, _1, _2),
    std::bind(&ArctosGripperController::handle_cancel, this, _1),
    std::bind(&ArctosGripperController::handle_accepted, this, _1));

  RCLCPP_INFO(get_node()->get_logger(),
              "Activated ArctosGripperController, action: %s", action_name.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArctosGripperController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (active_goal_)
  {
    auto result = std::make_shared<GripperAction::Result>();
    result->reached_goal = false;
    active_goal_->abort(result);
    active_goal_.reset();
  }
  goal_active_ = false;
  action_server_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ArctosGripperController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.push_back("Left_jaw_joint/position");
  return cfg;
}

controller_interface::InterfaceConfiguration
ArctosGripperController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.push_back("Left_jaw_joint/position");
  return cfg;
}

controller_interface::return_type ArctosGripperController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!goal_active_ || !active_goal_)
  {
    return controller_interface::return_type::OK;
  }

  double current_position = 0.0;
  for (const auto & si : state_interfaces_)
  {
    if (si.get_prefix_name() == "Left_jaw_joint" &&
        si.get_interface_name() == "position")
    {
      current_position = si.get_value();
      break;
    }
  }

  double error = std::abs(current_position - goal_position_);
  bool reached = error < position_tolerance_m_;

  publish_feedback(current_position);

  if (reached)
  {
    auto result = std::make_shared<GripperAction::Result>();
    result->position = current_position;
    result->effort = 0.0;
    result->stalled = false;
    result->reached_goal = true;
    active_goal_->succeed(result);
    active_goal_.reset();
    goal_active_ = false;
    RCLCPP_INFO(get_node()->get_logger(),
                "Gripper goal reached: %.4f m", current_position);
  }

  return controller_interface::return_type::OK;
}

rclcpp_action::GoalResponse ArctosGripperController::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const GripperAction::Goal> goal)
{
  double pos = goal->command.position;
  if (pos < 0.0 || pos > max_opening_m_)
  {
    RCLCPP_WARN(get_node()->get_logger(),
                "Gripper goal position %.4f m out of range [0, %.4f]",
                pos, max_opening_m_);
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArctosGripperController::handle_cancel(
  const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Gripper goal cancelled");
  if (active_goal_)
  {
    auto result = std::make_shared<GripperAction::Result>();
    result->reached_goal = false;
    active_goal_->canceled(result);
    active_goal_.reset();
    goal_active_ = false;
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArctosGripperController::handle_accepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  if (active_goal_)
  {
    auto result = std::make_shared<GripperAction::Result>();
    result->reached_goal = false;
    active_goal_->abort(result);
    active_goal_.reset();
  }

  goal_position_ = goal_handle->get_goal()->command.position;
  double clamped = std::clamp(goal_position_, 0.0, max_opening_m_);

  for (auto & ci : command_interfaces_)
  {
    if (ci.get_interface_name() == "position")
    {
      ci.set_value(clamped);
    }
  }

  active_goal_ = goal_handle;
  goal_active_ = true;

  RCLCPP_INFO(get_node()->get_logger(),
              "Gripper goal accepted: %.4f m", clamped);
}

void ArctosGripperController::publish_feedback(double current_position)
{
  if (!active_goal_)
  {
    return;
  }
  auto feedback = std::make_shared<GripperAction::Feedback>();
  feedback->position = current_position;
  feedback->effort = 0.0;
  feedback->stalled = false;
  feedback->reached_goal = false;
  active_goal_->publish_feedback(feedback);
}

}  // namespace arctos_controller

PLUGINLIB_EXPORT_CLASS(
  arctos_controller::ArctosGripperController,
  controller_interface::ControllerInterface)
