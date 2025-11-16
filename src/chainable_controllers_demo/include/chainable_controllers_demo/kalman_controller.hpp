#ifndef KALMAN_CONTROLLER__KALMAN_CONTROLLER_HPP_
#define KALMAN_CONTROLLER__KALMAN_CONTROLLER_HPP_

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
  // Called once to set up parameters
  controller_interface::CallbackReturn on_init() override;


  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  // No hardware state interfaces 
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  
  bool on_set_chained_mode(bool chained_mode) override;

  // only read from chained interfaces for now and publish a Pose
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:
  // doesn’t export any reference interfaces 
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  // for now we don't use any subscribers
  controller_interface::return_type update_reference_from_subscribers() override;

  // Names of chained command interfaces 
  std::vector<std::string> command_interface_names_;

  // Publisher for the fused pose
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
};

}  // namespace kalman_controller

#endif  // KALMAN_CONTROLLER__KALMAN_CONTROLLER_HPP_

