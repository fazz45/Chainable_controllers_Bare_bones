#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace chainable_controllers_demo
{

class RigidPoseBroadcaster : public controller_interface::ChainableControllerInterface
{
public:
  RigidPoseBroadcaster();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // claim interfaces from Kalman 
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  // export a dummy interface to satisfy the ChainableController requirement
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  bool on_set_chained_mode(bool chained_mode) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  controller_interface::return_type update_reference_from_subscribers() override;

protected:
  double t_{0.0};
};

}  // namespace chainable_controllers_demo
