#include "chainable_controllers_demo/broyden_controller.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <functional>

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;
using controller_interface::return_type;

namespace chainable_controllers_demo
{

BroydenController::BroydenController() = default;

CallbackReturn BroydenController::on_init()
{
  RCLCPP_INFO(get_node()->get_logger(), "BroydenController on_init");
  return CallbackReturn::SUCCESS;
}

// no  state interfaces from other controllers or hardware.
InterfaceConfiguration BroydenController::state_interface_configuration() const
{
  InterfaceConfiguration cfg;
  cfg.type = interface_configuration_type::NONE;
  return cfg;
}

// no hardware command interfaces as well.

InterfaceConfiguration BroydenController::command_interface_configuration() const
{
  InterfaceConfiguration cfg;
  cfg.type = interface_configuration_type::NONE;
  return cfg;
}

// Exporting our reference interfaces.
// Things will appear as broyden_controller/vel.v and broyden_controller/vel.w.
std::vector<hardware_interface::CommandInterface>
BroydenController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> refs;
  refs.reserve(2);

  refs.emplace_back(get_node()->get_name(), "vel.v", &v_ref_);
  refs.emplace_back(get_node()->get_name(), "vel.w", &w_ref_);

  return refs;
}


controller_interface::return_type BroydenController::update_reference_from_subscribers()
{
  // not subscribing to anything for this adn won't ever need to as that part will be fro kalman only
  return return_type::OK;
}

bool BroydenController::on_set_chained_mode(bool /*chained_mode*/)
{
  // For now, alwasy chained.

  return true;
}

CallbackReturn BroydenController::on_configure(const rclcpp_lifecycle::State &)
{
  // Subscribe to Kalman estimated pose on /kalman_estimate
  est_sub_ = get_node()->create_subscription<geometry_msgs::msg::Pose>(
    "/kalman_estimate",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&BroydenController::estCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "BroydenController configured");
  return CallbackReturn::SUCCESS;
}

CallbackReturn BroydenController::on_activate(const rclcpp_lifecycle::State &)
{
  // Initialize references and internal time
  v_ref_ = 0.2;
  w_ref_ = 0.0;
  t_ = 0.0;

  est_x_ = 0.0;
  est_y_ = 0.0;
  est_theta_ = 0.0;

  RCLCPP_INFO(get_node()->get_logger(), "BroydenController activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn BroydenController::on_deactivate(const rclcpp_lifecycle::State &)
{
  est_sub_.reset();
  RCLCPP_INFO(get_node()->get_logger(), "BroydenController deactivated");
  return CallbackReturn::SUCCESS;
}

// Subscriber callback: store latest estimated pose from Kalman
void BroydenController::estCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  est_x_ = msg->position.x;
  est_y_ = msg->position.y;

  // Extract yaw (theta) from quaternion 
  const double z = msg->orientation.z;
  const double w = msg->orientation.w;
  est_theta_ = 2.0 * std::atan2(z, w);
}

return_type BroydenController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  t_ += period.seconds();

  // Dummy reference generator
  v_ref_ = 0.2;
  w_ref_ = 0.3 * std::sin(0.5 * t_);


  return return_type::OK;
}

}  // namespace

PLUGINLIB_EXPORT_CLASS(
  chainable_controllers_demo::BroydenController,
  controller_interface::ChainableControllerInterface)

