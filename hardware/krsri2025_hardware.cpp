#include "krsri2025_hardware.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace krsri2025
{
// In your krsri2025_hardware.hpp file, make sure to add the new vector for velocity commands:
// std::vector<double> hw_velocity_commands_;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Resize vectors for joint states and commands
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // Add resizing for the new velocity command vector
  hw_velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // === UPDATE: Expect 2 command interfaces (position and velocity) ===
    if (joint.command_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobotSystem"), "Joint '%s' has %zu command interfaces. 2 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // === UPDATE: Verify that both 'position' and 'velocity' command interfaces exist ===
    bool has_position_command_interface = false;
    bool has_velocity_command_interface = false;
    for (const auto & interface : joint.command_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION) {
        has_position_command_interface = true;
      }
      if (interface.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity_command_interface = true;
      }
    }

    if (!has_position_command_interface) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobotSystem"), "Joint '%s' is missing '%s' command interface.",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION);
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    if (!has_velocity_command_interface) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobotSystem"), "Joint '%s' is missing '%s' command interface.",
        joint.name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // Verify we have position and velocity state interfaces (this part remains the same)
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobotSystem"), "Joint '%s' has %zu state interface. 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset values when configuring hardware
  for (std::size_t i = 0; i < hw_positions_.size(); i++) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
    // === UPDATE: Initialize velocity commands as well ===
    hw_velocity_commands_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Successfully configured!");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    // === UPDATE: Export both position and velocity command interfaces ===
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]));
  }

  return command_interfaces;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set initial command values to current positions
  for (std::size_t i = 0; i < hw_commands_.size(); i++) {
    if (std::isnan(hw_commands_[i])) {
      hw_commands_[i] = hw_positions_[i];
    }
    // === UPDATE: Initialize velocity command as well ===
    if (std::isnan(hw_velocity_commands_[i])) {
      hw_velocity_commands_[i] = 0.0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Successfully activated!");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Successfully deactivated!");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // For this simulation, we will simply set the current state
  // to be the same as the commanded state.
  // In a real robot, you would read sensor values from your hardware here.
  for (std::size_t i = 0; i < hw_positions_.size(); i++) {
    hw_positions_[i] = hw_commands_[i];
    hw_velocities_[i] = hw_velocity_commands_[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // In a real hardware interface, you would send commands to your hardware here.
  // The ros2_control framework handles writing the commands into the
  // hw_commands_ and hw_velocity_commands_ vectors.
  // For this simulation, the logic is handled in the read() function.

  return hardware_interface::return_type::OK;
}

}  // namespace krsri2025

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(krsri2025::RobotSystem, hardware_interface::SystemInterface)
