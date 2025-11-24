#include "chainable_controllers_demo/broyden_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace chainable_controllers_demo
{
BroydenController::BroydenController() = default;

controller_interface::CallbackReturn BroydenController::on_init()
{

  auto_declare<std::vector<std::string>>("interfaces", {});
  return controller_interface::CallbackReturn::SUCCESS;
}

//  interfaces we want to write to
controller_interface::InterfaceConfiguration 
BroydenController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // This will be loaded from yaml
  config.names = get_node()->get_parameter("interfaces").as_string_array();
  return config;
}


std::vector<hardware_interface::CommandInterface>
BroydenController::on_export_reference_interfaces()
{
  reference_interfaces_.resize(1);
  std::vector<hardware_interface::CommandInterface> refs;
  refs.emplace_back(get_node()->get_name(), "dummy", &reference_interfaces_[0]);
  return refs;
}

controller_interface::InterfaceConfiguration 
BroydenController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn BroydenController::on_configure(const rclcpp_lifecycle::State &)
{
  // Subscriber to /kalman_estimate 
  est_sub_ = get_node()->create_subscription<geometry_msgs::msg::Pose>(
    "/kalman_estimate", rclcpp::SystemDefaultsQoS(),
    std::bind(&BroydenController::estCallback, this, std::placeholders::_1));
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BroydenController::on_activate(const rclcpp_lifecycle::State &)
{
  t_ = 0.0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BroydenController::on_deactivate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

bool BroydenController::on_set_chained_mode(bool) { return true; }
controller_interface::return_type BroydenController::update_reference_from_subscribers() { return controller_interface::return_type::OK; }
void BroydenController::estCallback(const geometry_msgs::msg::Pose::SharedPtr) {} // kept simple for demo

controller_interface::return_type BroydenController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  t_ += period.seconds();
  double v_cmd = 0.2;
  double w_cmd = 0.3 * std::sin(0.5 * t_);

  //  PUSH the data to the Kalman

  if (command_interfaces_.size() >= 2) {
      command_interfaces_[0].set_value(v_cmd);
      command_interfaces_[1].set_value(w_cmd);
  }
  return controller_interface::return_type::OK;
}
} // namespace

PLUGINLIB_EXPORT_CLASS(
  chainable_controllers_demo::BroydenController,
  controller_interface::ChainableControllerInterface)




