#include "chainable_controllers_demo/kalman_controller.hpp"
#include <cmath>
#include "controller_interface/helpers.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace chainable_controllers_demo
{
KalmanController::KalmanController() = default;

controller_interface::CallbackReturn KalmanController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
KalmanController::on_export_reference_interfaces()
{
  // 1. Resize storage for 7 interfaces (5 Inputs, 2 Outputs)
  reference_interfaces_.resize(7);
  std::vector<hardware_interface::CommandInterface> refs;
  
  // INPUTS (Others write to these)
  refs.emplace_back(get_node()->get_name(), "pose.x", &reference_interfaces_[0]);
  refs.emplace_back(get_node()->get_name(), "pose.y", &reference_interfaces_[1]);
  refs.emplace_back(get_node()->get_name(), "pose.theta", &reference_interfaces_[2]);
  refs.emplace_back(get_node()->get_name(), "vel.v", &reference_interfaces_[3]);
  refs.emplace_back(get_node()->get_name(), "vel.w", &reference_interfaces_[4]);

  // OUTPUTS (We write to these, others read them)
  refs.emplace_back(get_node()->get_name(), "est.x", &reference_interfaces_[5]);
  refs.emplace_back(get_node()->get_name(), "est.y", &reference_interfaces_[6]);

  return refs;
}

controller_interface::InterfaceConfiguration
KalmanController::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
KalmanController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn KalmanController::on_configure(
  const rclcpp_lifecycle::State &)
{
  pose_pub_ = get_node()->create_publisher<geometry_msgs::msg::Pose>(
    "/kalman_estimate", rclcpp::SystemDefaultsQoS());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn KalmanController::on_activate(
  const rclcpp_lifecycle::State &)
{
  for(auto & val : reference_interfaces_) val = 0.0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn KalmanController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

bool KalmanController::on_set_chained_mode(bool) { return true; }

controller_interface::return_type KalmanController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // STEP 1: READ INPUTS (From indices 0-4)
  const double pose_x   = reference_interfaces_[0];
  const double pose_y   = reference_interfaces_[1];
  const double pose_th  = reference_interfaces_[2];
  const double vel_v    = reference_interfaces_[3];
  const double vel_w    = reference_interfaces_[4];

  // STEP 2: COMPUTE FUSION
  const double fused_x     = pose_x + vel_v;
  const double fused_y     = pose_y + vel_w;
  const double fused_theta = pose_th;

  // STEP 3: WRITE OUTPUTS (To indices 5-6)
  reference_interfaces_[5] = fused_x;
  reference_interfaces_[6] = fused_y;
  
  


  // Optional: Publish to topic for visualization only
  geometry_msgs::msg::Pose msg;
  msg.position.x = fused_x;
  msg.position.y = fused_y;
  msg.position.z = 0.0;
  const double half_yaw = fused_theta * 0.5;
  msg.orientation.z = std::sin(half_yaw);
  msg.orientation.w = std::cos(half_yaw);
  pose_pub_->publish(msg);

  return controller_interface::return_type::OK;
}

controller_interface::return_type KalmanController::update_reference_from_subscribers()
{
  return controller_interface::return_type::OK;
}

} // namespace chainable_controllers_demo

PLUGINLIB_EXPORT_CLASS(
  chainable_controllers_demo::KalmanController,
  controller_interface::ChainableControllerInterface)
