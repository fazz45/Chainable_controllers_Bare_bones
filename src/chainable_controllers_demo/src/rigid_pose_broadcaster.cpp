#include "chainable_controllers_demo/rigid_pose_broadcaster.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace chainable_controllers_demo
{
RigidPoseBroadcaster::RigidPoseBroadcaster() = default;

controller_interface::CallbackReturn RigidPoseBroadcaster::on_init()
{
  auto_declare<std::vector<std::string>>("interfaces", {});
  return controller_interface::CallbackReturn::SUCCESS;
}

// est to connect to Kalman's pose interfaces
controller_interface::InterfaceConfiguration 
RigidPoseBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Loaded from yaml
  config.names = get_node()->get_parameter("interfaces").as_string_array();
  return config;
}

std::vector<hardware_interface::CommandInterface>
RigidPoseBroadcaster::on_export_reference_interfaces()
{
  reference_interfaces_.resize(1);
  std::vector<hardware_interface::CommandInterface> refs;
  refs.emplace_back(get_node()->get_name(), "dummy", &reference_interfaces_[0]);
  return refs;
}

controller_interface::InterfaceConfiguration RigidPoseBroadcaster::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}
controller_interface::CallbackReturn RigidPoseBroadcaster::on_configure(const rclcpp_lifecycle::State &) { return controller_interface::CallbackReturn::SUCCESS; }
controller_interface::CallbackReturn RigidPoseBroadcaster::on_activate(const rclcpp_lifecycle::State &) { 
  t_ = 0.0; return controller_interface::CallbackReturn::SUCCESS; 
}
controller_interface::CallbackReturn RigidPoseBroadcaster::on_deactivate(const rclcpp_lifecycle::State &) { return controller_interface::CallbackReturn::SUCCESS; }
bool RigidPoseBroadcaster::on_set_chained_mode(bool) { return true; }
controller_interface::return_type RigidPoseBroadcaster::update_reference_from_subscribers() { return controller_interface::return_type::OK; }

controller_interface::return_type RigidPoseBroadcaster::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  t_ += period.seconds();
  double x_cmd = t_ * 0.1;

  // PUSH the data to Kalman
  if (command_interfaces_.size() >= 3) {
      command_interfaces_[0].set_value(x_cmd); // x
      command_interfaces_[1].set_value(0.0);   // y
      command_interfaces_[2].set_value(0.0);   // theta
  }
  return controller_interface::return_type::OK;
}
} // namespace

PLUGINLIB_EXPORT_CLASS(
  chainable_controllers_demo::RigidPoseBroadcaster,
  controller_interface::ChainableControllerInterface)
