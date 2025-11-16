#pragma once

#include <controller_interface/chainable_controller_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

namespace chainable_controllers_demo
{ 	

class BroydenController : public controller_interface::ChainableControllerInterface
{
public:
  BroydenController();

  controller_interface::CallbackReturn on_init() override;

  // doesn’t consume any state interfaces (neither the  hardware state and nor the upstream state)
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // for this structure we are not writing anything on the hardware here its task is just to write for kalman and cosnume the kalman_estimate in the real needle implementation this part will change
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  bool on_set_chained_mode(bool chained_mode) override;

  // Main control step: update v_ref_ / w_ref_ 
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:
  // Export our reference interfaces vel.v and vel.w to downstream(kalman) controllers
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  // We don’t use subscribers for it(in the original verison there was this option but now that we put kalman in between this funcitonality of shifting from the reference to state will in kalman
  controller_interface::return_type update_reference_from_subscribers() override;

  // Subscriber callback: latest /kalman_estimate pose
  void estCallback(const geometry_msgs::msg::Pose::SharedPtr msg);

  // Reference signals exposed as chainable interfaces
  double v_ref_{0.0};
  double w_ref_{0.0};

  // Internal time for dummy trajectory / future Broyden logic
  double t_{0.0};

  // Latest Kalman estimate
  double est_x_{0.0};
  double est_y_{0.0};
  double est_theta_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr est_sub_;
};

}  // namespace chainable_controllers_demo

