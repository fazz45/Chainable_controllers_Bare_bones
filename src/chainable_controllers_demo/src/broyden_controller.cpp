#include "chainable_controllers_demo/broyden_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace chainable_controllers_demo
{
BroydenController::BroydenController() = default;

controller_interface::CallbackReturn BroydenController::on_init()
{
  auto_declare<std::vector<std::string>>("interfaces", {});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
BroydenController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Loads: ["kalman/vel.v", "kalman/vel.w", "kalman/est.x", "kalman/est.y"]
  config.names = get_node()->get_parameter("interfaces").as_string_array();
  return config;
}

std::vector<hardware_interface::CommandInterface>
BroydenController::on_export_reference_interfaces()
{
  reference_interfaces_.resize(1);
  std::vector<hardware_interface::CommandInterface> refs;
  refs.emplace_back(get_node()->get_name(), "dummy", &reference_interfaces_[0]);
  return refs;
}

controller_interface::InterfaceConfiguration 
BroydenController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn BroydenController::on_configure(const rclcpp_lifecycle::State &)
{
  // NO SUBSCRIBER HERE ANYMORE!
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BroydenController::on_activate(const rclcpp_lifecycle::State &)
{
  t_ = 0.0;
  last_est_x_ = 0.0;
  last_est_y_ = 0.0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BroydenController::on_deactivate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

bool BroydenController::on_set_chained_mode(bool) { return true; }
controller_interface::return_type BroydenController::update_reference_from_subscribers() { return controller_interface::return_type::OK; }

controller_interface::return_type BroydenController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  t_ += period.seconds();

  // STEP 1: CALCULATE COMMAND (Using 'last_est' from previous cycle if needed)
  double v_cmd = 0.2;
  double w_cmd = 0.3 * std::sin(0.5 * t_);

  if (command_interfaces_.size() >= 4) {
      // STEP 2: WRITE COMMANDS (Indices 0 and 1 -> vel.v, vel.w)
      command_interfaces_[0].set_value(v_cmd);
      command_interfaces_[1].set_value(w_cmd);

      // STEP 3: READ FEEDBACK (Indices 2 and 3 -> est.x, est.y)
      last_est_x_ = command_interfaces_[2].get_value();
      last_est_y_ = command_interfaces_[3].get_value();
      
      // --- VERIFICATION LOG ---
      // Print once per second (every 100 cycles)

          RCLCPP_INFO(get_node()->get_logger(), 
              "Broyden READ from Interface -> X: %.5f | Y: %.5f", 
              last_est_x_, last_est_y_);
      
  }
  
  
  return controller_interface::return_type::OK;
}
} // namespace

PLUGINLIB_EXPORT_CLASS(
  chainable_controllers_demo::BroydenController,
  controller_interface::ChainableControllerInterface)
