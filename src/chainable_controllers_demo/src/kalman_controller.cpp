#include "chainable_controllers_demo/kalman_controller.hpp"


#include <cmath>
#include <limits>
#include <stdexcept>

#include "controller_interface/helpers.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace chainable_controllers_demo
{
KalmanController::KalmanController() = default;

controller_interface::CallbackReturn KalmanController::on_init()
{
  try
  {
    // read the chained interface names
    command_interface_names_ =
      auto_declare<std::vector<std::string>>("interfaces", {});
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init with message: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
KalmanController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = command_interface_names_;  // pose.x, pose.y, pose.theta, vel.v, vel.w
  return config;
}

controller_interface::InterfaceConfiguration
KalmanController::state_interface_configuration() const
{
  // doing nothing with the state interfaces but still needs to be here
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn KalmanController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Refresh interfaces list in case parameters changed
  if (!get_node()->get_parameter("interfaces", command_interface_names_))
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Parameter 'interfaces' not set, KalmanController will have no chained inputs.");
  }

  if (command_interface_names_.size() != 5)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "KalmanController expects 5 interfaces "
      "(pose.x, pose.y, pose.theta, vel.v, vel.w), but got %zu",
      command_interface_names_.size());
  }

  // Publisher for the pose
  pose_pub_ = get_node()->create_publisher<geometry_msgs::msg::Pose>(
    "/kalman_estimate", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_node()->get_logger(), "KalmanController configured.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn KalmanController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Ensure that we got all the requested interfaces in the correct order
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;

  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interface_names_, std::string(""), ordered_interfaces) ||
    command_interface_names_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu command interfaces, got %zu",
      command_interface_names_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "KalmanController activated.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn KalmanController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Nothing special to clean up
  RCLCPP_INFO(get_node()->get_logger(), "KalmanController deactivated.");
  return controller_interface::CallbackReturn::SUCCESS;
}

bool KalmanController::on_set_chained_mode(bool /*chained_mode*/)
{
  // behave the same whether chained or not
  return true;
}

controller_interface::return_type KalmanController::update_and_write_commands(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  //  this controller only reads
  // from chained command interfaces and publishes a fused /kalman_estimate Pose.

  if (command_interfaces_.size() < 5)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *(get_node()->get_clock()),
      1000,
      "KalmanController needs at least 5 command interfaces, but has %zu",
      command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  // Order is defined by the interfaces parameter and checked in on_activate()
  const double pose_x   = command_interfaces_[0].get_value();  // rigid_pose_broadcaster/pose.x
  const double pose_y   = command_interfaces_[1].get_value();  // rigid_pose_broadcaster/pose.y
  const double pose_th  = command_interfaces_[2].get_value();  // rigid_pose_broadcaster/pose.theta
  const double vel_v    = command_interfaces_[3].get_value();  // broyden_controller/vel.v
  const double vel_w    = command_interfaces_[4].get_value();  // broyden_controller/vel.w

  // just a dummy operation tov erify the outputs
  const double fused_x     = pose_x + vel_v;
  const double fused_y     = pose_y + vel_w;
  const double fused_theta = pose_th;

  geometry_msgs::msg::Pose msg;
  msg.position.x = fused_x;
  msg.position.y = fused_y;
  msg.position.z = 0.0;

  // this part we won't be needing in our needle work I kept just so that the thing looks complete in that we are publishing both kinda data 
  const double half_yaw = fused_theta * 0.5;
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = std::sin(half_yaw);
  msg.orientation.w = std::cos(half_yaw);

  pose_pub_->publish(msg);

  // NO calls to command_interfaces(no hardware)
  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface>
KalmanController::on_export_reference_interfaces()
{
  // klamna doesn't exposes adny controller as we discussed in the meeting we will be jsut publishing on a topic as 
  // exposing the kalman calculationg as a reference will cause a cyclic dependency and won't work.
  return {};
}

controller_interface::return_type KalmanController::update_reference_from_subscribers()
{
  // As of now i haven't given this feature to the kalman maybe in future if needed which ain't likely but there was something in which the broyden was also getting the data from a topic as i remeber 
  // in the hospital the data transfer will be happeneing through a topic and for the broyden we had that feature that we can switch from controller to topic and ofr the real needle one i will 
  return controller_interface::return_type::OK;
}


}  // namespace 

PLUGINLIB_EXPORT_CLASS(
  chainable_controllers_demo::KalmanController,
  controller_interface::ChainableControllerInterface)

