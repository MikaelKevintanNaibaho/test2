#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>  // Fixed include path
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "foot_trajectory.hpp"

// A struct to hold the joint angles for a single leg.
struct LegJoints
{
  double coxa;
  double femur;
  double tibia;
};

// A struct to hold gait parameters.
struct GaitParams
{
  double total_gait_time;
  double step_length;
  double swing_height;
  int trajectory_points;
  double dt;
};

// A class to represent the trajectory generator ROS2 node.
class TrajectoryGeneratorNode : public rclcpp::Node
{
public:
  TrajectoryGeneratorNode()
  : Node(
      "send_trajectory",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // Now, safely get the parameter
    if (!this->get_parameter("robot_description", robot_description_)) {
      RCLCPP_ERROR(this->get_logger(), "robot_description parameter not set. Shutting down.");
      // Properly shut down the node
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Successfully received robot_description.");
    // Initializes the publisher for the joint trajectory messages.
    pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "krsri_controller/joint_trajectory", 10);

    marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("foot_swing_markers", 10);

    // Load parameters, initialize kinematics, and generate the trajectory.
    loadGaitParams();
    initializeKinematics();

    // If initialization failed (e.g. robot_description missing), initializeKinematics()
    // may have called rclcpp::shutdown(); bail out instead of continuing.
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Initialization failed, aborting trajectory generation.");
      return;
    }

    generateAndPublishTrajectory();
  }

private:
  // Loads all necessary gait parameters from the ROS parameter server.
  void loadGaitParams()
  {
    gait_params_.total_gait_time = this->get_parameter("gait.total_gait_time").as_double();
    gait_params_.step_length = this->get_parameter("gait.step_length").as_double();
    gait_params_.swing_height = this->get_parameter("gait.swing_height").as_double();
    gait_params_.trajectory_points = this->get_parameter("gait.trajectory_points").as_int();
    gait_params_.dt =
      gait_params_.total_gait_time / static_cast<double>(gait_params_.trajectory_points - 1);
  }

  // Initializes the KDL kinematics (chains, solvers) for each leg.
  void initializeKinematics()
  {
    std::string robot_description;
    if (!this->get_parameter("robot_description", robot_description)) {
      RCLCPP_ERROR(this->get_logger(), "robot_description parameter not set. Shutting down.");
      rclcpp::shutdown();
      return;
    }

    KDL::Tree robot_tree;
    if (!kdl_parser::treeFromString(robot_description, robot_tree)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to build KDL tree");
      rclcpp::shutdown();
      return;
    }

    for (const auto & prefix : leg_prefixes_) {
      std::string tip_link = prefix + "_end_effector_link";
      KDL::Chain chain;
      if (!robot_tree.getChain(base_link_, tip_link, chain)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain for leg %s", prefix.c_str());
        rclcpp::shutdown();
        return;
      }
      chains_[prefix] = chain;
      ik_solvers_[prefix] =
        std::make_shared<KDL::ChainIkSolverVel_pinv>(chains_[prefix], 0.0000001);
      fk_solvers_[prefix] = std::make_shared<KDL::ChainFkSolverPos_recursive>(chains_[prefix]);
    }
  }

  // Generates the trot gait trajectory and publishes it.
  void generateAndPublishTrajectory()
  {
    // Load default joint positions and compute foot positions
    auto default_joints = loadDefaultJoints();
    auto joint_positions = initializeJointPositions(default_joints);
    auto default_foot_positions = computeDefaultFootPositions(joint_positions);

    // Create trajectory message
    trajectory_msgs::msg::JointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = this->now();
    populateJointNames(trajectory_msg);

    RCLCPP_INFO(
      this->get_logger(), "Generating trot gait with %d points", gait_params_.trajectory_points);
    std::map<std::string, std::vector<geometry_msgs::msg::Point>> leg_paths;

    // Main trajectory generation loop
    for (int i = 0; i < gait_params_.trajectory_points; i++) {
      double current_time = i * gait_params_.dt;
      bool is_phase_one = (current_time < gait_params_.total_gait_time / 2.0);

      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.time_from_start = rclcpp::Duration::from_seconds(current_time);
      std::vector<double> all_positions, all_velocities;

      // Process each leg's movement
      for (const auto & prefix : leg_prefixes_) {
        bool is_swing_leg = (is_phase_one && (prefix == "FL" || prefix == "BR")) ||
                            (!is_phase_one && (prefix == "FR" || prefix == "BL"));

        KDL::JntArray & current_joint_pos = joint_positions[prefix];
        KDL::JntArray joint_velocities(current_joint_pos.rows());

        if (is_swing_leg) {
          processSwingLeg(
            prefix, current_time, is_phase_one, current_joint_pos, joint_velocities,
            default_foot_positions, leg_paths);
        } else {
          processStanceLeg(joint_velocities);
        }

        // Update joint positions and velocities for the message
        for (unsigned int j = 0; j < current_joint_pos.rows(); j++) {
          current_joint_pos(j) += joint_velocities(j) * gait_params_.dt;
          all_positions.push_back(current_joint_pos(j));
          all_velocities.push_back(joint_velocities(j));
        }
      }
      point.positions = all_positions;
      point.velocities = all_velocities;
      trajectory_msg.points.push_back(point);
    }
    publishTrajectoryMarkers(leg_paths);

    // Publish the complete trajectory
    RCLCPP_INFO(this->get_logger(), "Publishing trajectory for all legs");
    pub_->publish(trajectory_msg);

    rclcpp::sleep_for(std::chrono::seconds(1));
    rclcpp::shutdown();
  }

  // Loads the default joint positions from the parameter server.
  std::map<std::string, LegJoints> loadDefaultJoints()
  {
    std::map<std::string, LegJoints> default_joints;
    RCLCPP_INFO(this->get_logger(), "Loading default joint positions for all legs...");
    for (const auto & prefix : leg_prefixes_) {
      std::string ns = "default_stance." + prefix;
      LegJoints joints;

      // Use a flag to track if all params for a leg are found
      bool success = true;
      success &= this->get_parameter(ns + ".coxa", joints.coxa);
      success &= this->get_parameter(ns + ".femur", joints.femur);
      success &= this->get_parameter(ns + ".tibia", joints.tibia);

      if (success) {
        RCLCPP_INFO(
          this->get_logger(), "Successfully loaded for %s: coxa=%.6f, femur=%.6f, tibia=%.6f",
          prefix.c_str(), joints.coxa, joints.femur, joints.tibia);
        default_joints[prefix] = joints;
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to load one or more default stance parameters for leg prefix: %s",
          prefix.c_str());
        // Shut down to prevent running with incorrect initial positions
        rclcpp::shutdown();
      }
    }
    return default_joints;
  }

  // Initializes the KDL joint arrays from the loaded default joint values.
  std::map<std::string, KDL::JntArray> initializeJointPositions(
    const std::map<std::string, LegJoints> & default_joints)
  {
    std::map<std::string, KDL::JntArray> joint_positions;
    RCLCPP_INFO(this->get_logger(), "Initializing KDL joint arrays from loaded defaults...");
    for (const auto & prefix : leg_prefixes_) {
      // Find the joints for the current leg prefix
      auto it = default_joints.find(prefix);
      if (it != default_joints.end()) {
        const auto & joints = it->second;
        joint_positions[prefix] = KDL::JntArray(3);
        joint_positions[prefix](0) = joints.coxa;
        joint_positions[prefix](1) = joints.femur;
        joint_positions[prefix](2) = joints.tibia;

        // Add detailed logging to confirm the values being set
        RCLCPP_INFO(
          this->get_logger(), "KDL JntArray for '%s' initialized to: [%f, %f, %f]", prefix.c_str(),
          joint_positions[prefix](0), joint_positions[prefix](1), joint_positions[prefix](2));
      } else {
        RCLCPP_ERROR(
          this->get_logger(), "Could not find default joints for leg prefix '%s' in the map.",
          prefix.c_str());
        rclcpp::shutdown();
      }
    }
    return joint_positions;
  }

  // Computes the initial foot positions using forward kinematics.
  std::map<std::string, Vec3<double>> computeDefaultFootPositions(
    std::map<std::string, KDL::JntArray> & joint_positions)
  {
    std::map<std::string, Vec3<double>> default_foot_positions;
    for (const auto & prefix : leg_prefixes_) {
      KDL::Frame foot_pose;
      if (fk_solvers_[prefix]->JntToCart(joint_positions[prefix], foot_pose) < 0) {
        RCLCPP_ERROR(this->get_logger(), "FK failed for %s leg", prefix.c_str());
        rclcpp::shutdown();
        return {};
      }
      default_foot_positions[prefix] =
        Vec3<double>(foot_pose.p.x(), foot_pose.p.y(), foot_pose.p.z());

      RCLCPP_INFO(
        this->get_logger(), "%s foot position: x=%.4f y=%.4f z=%.4f", prefix.c_str(),
        foot_pose.p.x(), foot_pose.p.y(), foot_pose.p.z());
    }
    return default_foot_positions;
  }

  // Populates the joint names in the trajectory message.
  void populateJointNames(trajectory_msgs::msg::JointTrajectory & trajectory_msg)
  {
    for (const auto & prefix : leg_prefixes_) {
      trajectory_msg.joint_names.push_back(prefix + "_coxa_link_joint");
      trajectory_msg.joint_names.push_back(prefix + "_femur_link_joint");
      trajectory_msg.joint_names.push_back(prefix + "_tibia_link_joint");
    }
  }

  // Calculates the trajectory for a swing leg.
  void processSwingLeg(
    const std::string & prefix, double current_time, bool is_phase_one,
    KDL::JntArray & current_joint_pos, KDL::JntArray & joint_velocities,
    const std::map<std::string, Vec3<double>> & default_foot_positions,
    std::map<std::string, std::vector<geometry_msgs::msg::Point>> & leg_paths)
  {
    double phase_time =
      is_phase_one ? current_time : (current_time - gait_params_.total_gait_time / 2.0);
    double swing_phase = phase_time / (gait_params_.total_gait_time / 2.0);

    Vec3<double> p_start = default_foot_positions.at(prefix);
    Vec3<double> p_end = p_start + Vec3<double>(gait_params_.step_length, 0.0, 0.0);

    FootSwingTrajectory<double> trajectory;
    trajectory.setInitialPosition(p_start);
    trajectory.setFinalPosition(p_end);
    trajectory.setHeight(gait_params_.swing_height);
    trajectory.computeSwingTrajectoryBezier(swing_phase, gait_params_.total_gait_time / 2.0);

    Vec3<double> cart_pos_vec = trajectory.getPosition();
    Vec3<double> cart_vel_vec = trajectory.getVelocity();
    KDL::Twist cart_twist(
      KDL::Vector(cart_vel_vec[0], cart_vel_vec[1], cart_vel_vec[2]), KDL::Vector::Zero());

    // NEW: Store the calculated cartesian point
    geometry_msgs::msg::Point p;
    p.x = cart_pos_vec[0];
    p.y = cart_pos_vec[1];
    p.z = cart_pos_vec[2];
    leg_paths[prefix].push_back(p);

    ik_solvers_[prefix]->CartToJnt(current_joint_pos, cart_twist, joint_velocities);
  }

  // Sets the velocity for a stance leg to zero.
  void processStanceLeg(KDL::JntArray & joint_velocities)
  {
    for (unsigned int j = 0; j < joint_velocities.rows(); j++) {
      joint_velocities(j) = 0.0;
    }
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
      marker.header.frame_id = base_link_;
      marker.header.stamp = this->now();
      marker.ns = "foot_paths";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.01;  // Line width

      marker.color.r = colors[prefix][0];
      marker.color.g = colors[prefix][1];
      marker.color.b = colors[prefix][2];
      marker.color.a = 1.0;  // Opacity

      marker.points = path_points;
      marker_array.markers.push_back(marker);
    }

    RCLCPP_INFO(
      this->get_logger(), "Publishing %zu trajectory markers.", marker_array.markers.size());
    marker_pub_->publish(marker_array);
  }

  // Member Variables
  std::string robot_description_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  GaitParams gait_params_;
  std::vector<std::string> leg_prefixes_ = {"FL", "FR", "BL", "BR"};
  std::string base_link_ = "base_link";

  std::map<std::string, KDL::Chain> chains_;
  std::map<std::string, std::shared_ptr<KDL::ChainIkSolverVel_pinv>> ik_solvers_;
  std::map<std::string, std::shared_ptr<KDL::ChainFkSolverPos_recursive>> fk_solvers_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryGeneratorNode>();

  return 0;
}
