#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/pose.hpp"
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

  // Claim interfaces from Kalman
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  // Export dummy interface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  bool on_set_chained_mode(bool chained_mode) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  controller_interface::return_type update_reference_from_subscribers() override;

protected:
  // Callback for the Kalman estimate subscription
  void estCallback(const geometry_msgs::msg::Pose::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr est_sub_;

  // Internal time for generating the sine wave
  double t_{0.0};

  // --- THESE ARE THE VARIABLES YOU WERE MISSING ---
  double est_x_{0.0};
  double est_y_{0.0};
  double est_theta_{0.0};
};

}  // namespace chainable_controllers_demo
