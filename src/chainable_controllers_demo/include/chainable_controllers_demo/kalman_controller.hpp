#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace chainable_controllers_demo
{

class KalmanController : public controller_interface::ChainableControllerInterface
{
public:
  KalmanController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Exports 7 interfaces: 5 Inputs + 2 Outputs
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  bool on_set_chained_mode(bool chained_mode) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  controller_interface::return_type update_reference_from_subscribers() override;

protected:
  // We keep the publisher JUST for rqt_plot verification, 
  // but Broyden will not use it anymore.
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
};

}  // namespace chainable_controllers_demo
