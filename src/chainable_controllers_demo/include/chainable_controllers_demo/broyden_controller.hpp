#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace chainable_controllers_demo
{

class BroydenController : public controller_interface::ChainableControllerInterface
{
public:
  BroydenController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Claims 4 interfaces (2 to write, 2 to read)
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  // Dummy export
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  bool on_set_chained_mode(bool chained_mode) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  controller_interface::return_type update_reference_from_subscribers() override;

protected:
  double t_{0.0};
  
  // Storage for the feedback read from Kalman
  double last_est_x_{0.0};
  double last_est_y_{0.0};
};

}  // namespace chainable_controllers_demo
