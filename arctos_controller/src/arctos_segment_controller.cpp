#include "arctos_controller/arctos_segment_controller.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arctos_controller
{

ArctosSegmentController::ArctosSegmentController()
: controller_interface::ControllerInterface()
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
              "Configured ArctosSegmentController: %zu joints (bridge-backed)", joints_.size());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArctosSegmentController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(),
              "Activated ArctosSegmentController (trajectory via arctos_bridge)");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArctosSegmentController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ArctosSegmentController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::NONE;
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

controller_interface::return_type ArctosSegmentController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

}  // namespace arctos_controller

PLUGINLIB_EXPORT_CLASS(
  arctos_controller::ArctosSegmentController,
  controller_interface::ControllerInterface)
