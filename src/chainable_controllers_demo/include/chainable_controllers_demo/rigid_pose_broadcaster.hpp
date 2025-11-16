#pragma once

#include <controller_interface/chainable_controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace chainable_controllers_demo
{

class RigidPoseBroadcaster : public controller_interface::ChainableControllerInterface
{
public:
  RigidPoseBroadcaster();

  controller_interface::CallbackReturn on_init() override;

  // No hardware state interfaces
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // doesn't command hardware 
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  bool on_set_chained_mode(bool chained_mode) override;

  // Main update: update pose_x_, pose_y_, pose_theta_
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:
  // Export pose.x, pose.y, pose.theta as reference interfaces
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;


  controller_interface::return_type update_reference_from_subscribers() override;

  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_theta_{0.0};

  double t_{0.0};
};

}  // namespace chainable_controllers_demo

