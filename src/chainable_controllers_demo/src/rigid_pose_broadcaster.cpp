#include "chainable_controllers_demo/rigid_pose_broadcaster.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;
using controller_interface::return_type;

namespace chainable_controllers_demo
{

RigidPoseBroadcaster::RigidPoseBroadcaster() = default;

CallbackReturn RigidPoseBroadcaster::on_init()
{
  RCLCPP_INFO(get_node()->get_logger(), "RigidPoseBroadcaster on_init");
  return CallbackReturn::SUCCESS;
}

//nothing from the state for now we will implement in the needle version though 
InterfaceConfiguration RigidPoseBroadcaster::state_interface_configuration() const
{
  InterfaceConfiguration cfg;
  cfg.type = interface_configuration_type::NONE;
  return cfg;
}

// nothing here as well we jsut expose the reference and will never write to any hardware on anything 
InterfaceConfiguration RigidPoseBroadcaster::command_interface_configuration() const
{
  InterfaceConfiguration cfg;
  cfg.type = interface_configuration_type::NONE;
  return cfg;
}

// Export pose as reference (command) interfaces 


std::vector<hardware_interface::CommandInterface>
RigidPoseBroadcaster::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> refs;
  refs.reserve(3);

  refs.emplace_back(get_node()->get_name(), "pose.x", &pose_x_);
  refs.emplace_back(get_node()->get_name(), "pose.y", &pose_y_);
  refs.emplace_back(get_node()->get_name(), "pose.theta", &pose_theta_);

  return refs;
}


controller_interface::return_type RigidPoseBroadcaster::update_reference_from_subscribers()
{
  // chained only nothing to do with subscriber in real will take data from state itnerface and nothing from subscriber so will alwasy stay the same
  return return_type::OK;
}

bool RigidPoseBroadcaster::on_set_chained_mode(bool /*chained_mode*/)
{
  // chained only for now but when the kalamn gets out of it it will as well so will have to work here
  return true;
}

CallbackReturn RigidPoseBroadcaster::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "RigidPoseBroadcaster configured");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RigidPoseBroadcaster::on_activate(const rclcpp_lifecycle::State &)
{
  t_ = 0.0;
  pose_x_ = 0.0;
  pose_y_ = 0.0;
  pose_theta_ = 0.0;

  RCLCPP_INFO(get_node()->get_logger(), "RigidPoseBroadcaster activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RigidPoseBroadcaster::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "RigidPoseBroadcaster deactivated");
  return CallbackReturn::SUCCESS;
}

return_type RigidPoseBroadcaster::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // simple dummy pose
  t_ += period.seconds();
  pose_x_ += 0.01;   // 100 Hz
  pose_y_ += 0.0;
  pose_theta_ = 0.0;

  // Just updating pose_x_/pose_y_/pose_theta_ is enough for now
  return return_type::OK;
}

} // namespace 

PLUGINLIB_EXPORT_CLASS(
  chainable_controllers_demo::RigidPoseBroadcaster,
  controller_interface::ChainableControllerInterface)

