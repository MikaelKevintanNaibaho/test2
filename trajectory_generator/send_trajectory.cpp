#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "foot_trajectory.hpp"

// A struct to hold gait parameters.
struct GaitParams
{
  double total_gait_time;
  double step_length;
  double swing_height;
  int trajectory_points;
  double dt;
};

// A class to represent the MoveIt2-based trajectory generator ROS2 node.
class MoveIt2TrajectoryGeneratorNode : public rclcpp::Node
{
public:
  MoveIt2TrajectoryGeneratorNode()
  : Node(
      "moveit2_trajectory_generator",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    RCLCPP_INFO(this->get_logger(), "Starting MoveIt2 Trajectory Generator Node");

    // Initialize publishers
    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "krsri_controller/joint_trajectory", 10);
    marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("foot_swing_markers", 10);
    display_trajectory_pub_ =
      this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("display_planned_path", 10);

    // Load parameters
    loadGaitParams();

    // Initialize MoveIt2 components
    initializeMoveIt();

    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Initialization failed, aborting trajectory generation.");
      return;
    }

    // Generate and execute trajectory
    generateAndPublishTrajectory();
  }

private:
  void loadGaitParams()
  {
    gait_params_.total_gait_time = this->get_parameter("gait.total_gait_time").as_double();
    gait_params_.step_length = this->get_parameter("gait.step_length").as_double();
    gait_params_.swing_height = this->get_parameter("gait.swing_height").as_double();
    gait_params_.trajectory_points = this->get_parameter("gait.trajectory_points").as_int();
    gait_params_.dt =
      gait_params_.total_gait_time / static_cast<double>(gait_params_.trajectory_points - 1);
  }

  void initializeMoveIt()
  {
    try {
      // Initialize robot model loader
      robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
        shared_from_this(), "robot_description");

      if (!robot_model_loader_->getModel()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load robot model");
        rclcpp::shutdown();
        return;
      }

      robot_model_ = robot_model_loader_->getModel();
      robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
      robot_state_->setToDefaultValues();

      // Initialize move group interfaces for each leg
      for (const auto & prefix : leg_prefixes_) {
        std::string group_name =
          prefix + "_leg";  // Assuming planning groups are named like "FL_leg", "FR_leg", etc.

        try {
          auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), group_name);

          move_group->setPlanningTime(5.0);
          move_group->setNumPlanningAttempts(3);
          move_group->setMaxVelocityScalingFactor(0.5);
          move_group->setMaxAccelerationScalingFactor(0.3);

          move_groups_[prefix] = move_group;

          RCLCPP_INFO(this->get_logger(), "Initialized MoveGroup for %s", prefix.c_str());
        } catch (const std::exception & e) {
          RCLCPP_ERROR(
            this->get_logger(), "Failed to initialize MoveGroup for %s: %s", prefix.c_str(),
            e.what());
        }
      }

      // Initialize planning scene interface
      planning_scene_interface_ =
        std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

      RCLCPP_INFO(this->get_logger(), "MoveIt2 initialization complete");

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "MoveIt2 initialization failed: %s", e.what());
      rclcpp::shutdown();
    }
  }

  void generateAndPublishTrajectory()
  {
    RCLCPP_INFO(this->get_logger(), "Generating trot gait with MoveIt2");

    // Get current robot state
    robot_state_->setToDefaultValues();
    loadDefaultJointPositions();

    // Compute initial foot positions
    auto default_foot_positions = computeCurrentFootPositions();

    // Generate trajectory waypoints
    std::vector<moveit_msgs::msg::RobotTrajectory> leg_trajectories;
    std::map<std::string, std::vector<geometry_msgs::msg::Point>> leg_paths;

    // Generate trajectories for each phase
    generateTrotGaitTrajectories(default_foot_positions, leg_trajectories, leg_paths);

    // Combine and publish trajectories
    combineAndPublishTrajectories(leg_trajectories);

    // Publish visualization markers
    publishTrajectoryMarkers(leg_paths);

    RCLCPP_INFO(this->get_logger(), "Trajectory generation complete");
  }

  void loadDefaultJointPositions()
  {
    RCLCPP_INFO(this->get_logger(), "Loading default joint positions...");

    for (const auto & prefix : leg_prefixes_) {
      std::string ns = "default_stance." + prefix;

      double coxa, femur, tibia;
      if (
        this->get_parameter(ns + ".coxa", coxa) && this->get_parameter(ns + ".femur", femur) &&
        this->get_parameter(ns + ".tibia", tibia)) {
        // Set joint values in robot state
        robot_state_->setJointPositions(prefix + "_coxa_link_joint", &coxa);
        robot_state_->setJointPositions(prefix + "_femur_link_joint", &femur);
        robot_state_->setJointPositions(prefix + "_tibia_link_joint", &tibia);

        RCLCPP_INFO(
          this->get_logger(), "Set %s joints: coxa=%.3f, femur=%.3f, tibia=%.3f", prefix.c_str(),
          coxa, femur, tibia);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load default joints for %s", prefix.c_str());
        rclcpp::shutdown();
        return;
      }
    }
  }

  std::map<std::string, Eigen::Vector3d> computeCurrentFootPositions()
  {
    std::map<std::string, Eigen::Vector3d> foot_positions;

    for (const auto & prefix : leg_prefixes_) {
      std::string end_effector = prefix + "_end_effector_link";

      const Eigen::Isometry3d & end_effector_state =
        robot_state_->getGlobalLinkTransform(end_effector);
      Eigen::Vector3d position = end_effector_state.translation();

      foot_positions[prefix] = position;

      RCLCPP_INFO(
        this->get_logger(), "%s foot position: [%.3f, %.3f, %.3f]", prefix.c_str(), position.x(),
        position.y(), position.z());
    }

    return foot_positions;
  }

  void generateTrotGaitTrajectories(
    const std::map<std::string, Eigen::Vector3d> & default_foot_positions,
    std::vector<moveit_msgs::msg::RobotTrajectory> & leg_trajectories,
    std::map<std::string, std::vector<geometry_msgs::msg::Point>> & leg_paths)
  {
    // Generate trajectory for each time step
    for (int i = 0; i < gait_params_.trajectory_points; i++) {
      double current_time = i * gait_params_.dt;
      bool is_phase_one = (current_time < gait_params_.total_gait_time / 2.0);

      for (const auto & prefix : leg_prefixes_) {
        bool is_swing_leg = (is_phase_one && (prefix == "FL" || prefix == "BR")) ||
                            (!is_phase_one && (prefix == "FR" || prefix == "BL"));

        if (is_swing_leg) {
          generateSwingTrajectoryWaypoint(
            prefix, current_time, is_phase_one, default_foot_positions, leg_paths);
        }
      }
    }
  }

  void generateSwingTrajectoryWaypoint(
    const std::string & prefix, double current_time, bool is_phase_one,
    const std::map<std::string, Eigen::Vector3d> & default_foot_positions,
    std::map<std::string, std::vector<geometry_msgs::msg::Point>> & leg_paths)
  {
    double phase_time =
      is_phase_one ? current_time : (current_time - gait_params_.total_gait_time / 2.0);
    double swing_phase = phase_time / (gait_params_.total_gait_time / 2.0);

    Eigen::Vector3d p_start = default_foot_positions.at(prefix);

    // Adjust step length based on leg prefix
    double adjusted_step_length = gait_params_.step_length;
    if (prefix == "FR" || prefix == "FL") {
      adjusted_step_length = -gait_params_.step_length;
    }

    Eigen::Vector3d p_end = p_start + Eigen::Vector3d(adjusted_step_length, 0.0, 0.0);

    // Generate foot trajectory using your existing FootSwingTrajectory class
    FootSwingTrajectory<double> trajectory;
    Vec3<double> start_vec(p_start.x(), p_start.y(), p_start.z());
    Vec3<double> end_vec(p_end.x(), p_end.y(), p_end.z());

    trajectory.setInitialPosition(start_vec);
    trajectory.setFinalPosition(end_vec);
    trajectory.setHeight(gait_params_.swing_height);
    trajectory.computeSwingTrajectoryBezier(swing_phase, gait_params_.total_gait_time / 2.0);

    Vec3<double> cart_pos_vec = trajectory.getPosition();

    // Store trajectory point for visualization
    geometry_msgs::msg::Point p;
    p.x = cart_pos_vec[0];
    p.y = cart_pos_vec[1];
    p.z = cart_pos_vec[2];
    leg_paths[prefix].push_back(p);

    // Plan to this cartesian position using MoveIt2
    if (move_groups_.find(prefix) != move_groups_.end()) {
      planToCartesianPosition(prefix, cart_pos_vec);
    }
  }

  void planToCartesianPosition(const std::string & prefix, const Vec3<double> & target_pos)
  {
    auto move_group = move_groups_[prefix];

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = target_pos[0];
    target_pose.position.y = target_pos[1];
    target_pose.position.z = target_pos[2];
    target_pose.orientation.w = 1.0;

    move_group->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
      // Store the planned trajectory segment
      // In a full implementation, you'd collect these and combine them
      RCLCPP_DEBUG(this->get_logger(), "Successfully planned trajectory for %s", prefix.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to plan trajectory for %s", prefix.c_str());
    }
  }

  void combineAndPublishTrajectories(
    const std::vector<moveit_msgs::msg::RobotTrajectory> & leg_trajectories)
  {
    // This is a simplified version - in practice you'd need to carefully synchronize
    // and combine the individual leg trajectories into a single coordinated movement

    trajectory_msgs::msg::JointTrajectory combined_trajectory;
    combined_trajectory.header.stamp = this->now();

    // Populate joint names for all legs
    for (const auto & prefix : leg_prefixes_) {
      combined_trajectory.joint_names.push_back(prefix + "_coxa_link_joint");
      combined_trajectory.joint_names.push_back(prefix + "_femur_link_joint");
      combined_trajectory.joint_names.push_back(prefix + "_tibia_link_joint");
    }

    // For demonstration, create a simple trajectory point with current positions
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.0);

    // Get current joint positions
    std::vector<double> current_positions;
    for (const auto & prefix : leg_prefixes_) {
      const double * coxa = robot_state_->getJointPositions(prefix + "_coxa_link_joint");
      const double * femur = robot_state_->getJointPositions(prefix + "_femur_link_joint");
      const double * tibia = robot_state_->getJointPositions(prefix + "_tibia_link_joint");

      current_positions.push_back(*coxa);
      current_positions.push_back(*femur);
      current_positions.push_back(*tibia);
    }

    point.positions = current_positions;
    combined_trajectory.points.push_back(point);

    RCLCPP_INFO(this->get_logger(), "Publishing combined trajectory");
    trajectory_pub_->publish(combined_trajectory);
  }

  void publishTrajectoryMarkers(
    const std::map<std::string, std::vector<geometry_msgs::msg::Point>> & leg_paths)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    // Define colors for each leg
    std::map<std::string, std::array<float, 3>> colors;
    colors["FL"] = {1.0, 0.0, 0.0};  // Red
    colors["FR"] = {0.0, 1.0, 0.0};  // Green
    colors["BL"] = {0.0, 0.0, 1.0};  // Blue
    colors["BR"] = {1.0, 1.0, 0.0};  // Yellow

    for (const auto & pair : leg_paths) {
      const auto & prefix = pair.first;
      const auto & path_points = pair.second;

      if (path_points.empty()) continue;

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = this->now();
      marker.ns = "moveit2_foot_paths";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.01;  // Line width

      marker.color.r = colors[prefix][0];
      marker.color.g = colors[prefix][1];
      marker.color.b = colors[prefix][2];
      marker.color.a = 1.0;

      marker.points = path_points;
      marker_array.markers.push_back(marker);
    }

    RCLCPP_INFO(
      this->get_logger(), "Publishing %zu MoveIt2 trajectory markers", marker_array.markers.size());
    marker_pub_->publish(marker_array);
  }

  // Member Variables
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_trajectory_pub_;

  GaitParams gait_params_;
  std::vector<std::string> leg_prefixes_ = {"FL", "FR", "BL", "BR"};

  // MoveIt2 components
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  std::shared_ptr<moveit::core::RobotState> robot_state_;
  std::map<std::string, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>
    move_groups_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // MoveIt2 requires multi-threaded execution
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<MoveIt2TrajectoryGeneratorNode>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
