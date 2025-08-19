#include "krsri_controller.hpp"

#include <stddef.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace krsri2025
{
RobotController::RobotController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn RobotController::on_init()
{
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ =
    auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ =
    auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

  point_interp_.positions.assign(joint_names_.size(), 0);
  point_interp_.velocities.assign(joint_names_.size(), 0);
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : command_interface_types_) {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : state_interface_types_) {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::CallbackReturn RobotController::on_configure(const rclcpp_lifecycle::State &)
{
  auto callback =
    [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg) -> void {
    RCLCPP_INFO(get_node()->get_logger(), "Received new Trajectory.");
    traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
    new_msg_ = true;
  };

  joint_command_subscriber_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_activate(const rclcpp_lifecycle::State &)
{
  joint_position_command_interface_.clear();
  joint_velocity_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();

  for (auto & interface : command_interfaces_) {
    if (interface.get_interface_name().find("position") != std::string::npos) {
      joint_position_command_interface_.emplace_back(interface);
    } else if (interface.get_interface_name().find("velocity") != std::string::npos) {
      joint_velocity_command_interface_.emplace_back(interface);
    }
  }

  for (auto & interface : state_interfaces_) {
    if (interface.get_interface_name().find("position") != std::string::npos) {
      joint_position_state_interface_.emplace_back(interface);
    } else if (interface.get_interface_name().find("velocity") != std::string::npos) {
      joint_velocity_state_interface_.emplace_back(interface);
    }
  }
  return CallbackReturn::SUCCESS;
}

void interpolate_point(
  const trajectory_msgs::msg::JointTrajectoryPoint & point_1,
  const trajectory_msgs::msg::JointTrajectoryPoint & point_2,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp, double delta)
{
  point_interp.positions.resize(point_1.positions.size());
  point_interp.velocities.resize(point_1.velocities.size());
  
  for (size_t i = 0; i < point_1.positions.size(); ++i) {
    point_interp.positions[i] = point_1.positions[i] + delta * (point_2.positions[i] - point_1.positions[i]);
    if (i < point_1.velocities.size() && i < point_2.velocities.size()) {
      point_interp.velocities[i] = point_1.velocities[i] + delta * (point_2.velocities[i] - point_1.velocities[i]);
    }
  }
}

void interpolate_trajectory_point(
  const trajectory_msgs::msg::JointTrajectory & traj_msg, const rclcpp::Duration & cur_time,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp, bool & reached_end)
{
  reached_end = false;
  
  if (traj_msg.points.empty()) {
    reached_end = true;
    return;
  }

  size_t index = 0;
  for (size_t i = 0; i < traj_msg.points.size(); ++i) {
    if (cur_time <= rclcpp::Duration(traj_msg.points[i].time_from_start)) {
      index = i;
      break;
    }
    if (i == traj_msg.points.size() - 1) {
      reached_end = true;
      point_interp = traj_msg.points[i];
      return;
    }
  }

  if (index == 0) {
    point_interp = traj_msg.points[0];
  } else {
    auto dt1 = rclcpp::Duration(traj_msg.points[index - 1].time_from_start);
    auto dt2 = rclcpp::Duration(traj_msg.points[index].time_from_start);
    double delta = (cur_time - dt1).seconds() / (dt2 - dt1).seconds();
    interpolate_point(traj_msg.points[index - 1], traj_msg.points[index], point_interp, delta);
  }
}

controller_interface::return_type RobotController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (new_msg_) {
    trajectory_msg_ = *traj_msg_external_point_ptr_.readFromRT();
    start_time_ = time;
    new_msg_ = false;
  }

  if (trajectory_msg_ != nullptr) {
    bool reached_end;
    interpolate_trajectory_point(*trajectory_msg_, time - start_time_, point_interp_, reached_end);

    if (reached_end) {
      RCLCPP_INFO(get_node()->get_logger(), "Trajectory execution complete.");
      trajectory_msg_.reset();
    }

    for (size_t i = 0; i < joint_position_command_interface_.size(); i++) {
      if (!joint_position_command_interface_[i].get().set_value(point_interp_.positions[i])) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to set position value for index %ld", i);
      }
    }
    for (size_t i = 0; i < joint_velocity_command_interface_.size(); i++) {
      if (!joint_velocity_command_interface_[i].get().set_value(point_interp_.velocities[i])) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to set velocity value for index %ld", i);
      }
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RobotController::on_deactivate(const rclcpp_lifecycle::State &)
{
  release_interfaces();
  return CallbackReturn::SUCCESS;
}

}  // namespace krsri2025

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(krsri2025::RobotController, controller_interface::ControllerInterface)
