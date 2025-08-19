#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>  // Fixed include path
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "foot_trajectory.hpp"

struct LegJoints
{
  double coxa;
  double femur;
  double tibia;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("send_trajectory");
  auto pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "krsri_controller/joint_trajectory", 10);

  // Get robot description
  std::string robot_description;
  if (!node->get_parameter("robot_description", robot_description)) {
    RCLCPP_ERROR(node->get_logger(), "robot_description parameter not set. Shutting down.");
    return -1;
  }

  // Get default joint positions from parameters file
  std::map<std::string, LegJoints> default_joints;
  for (const auto & prefix : {"FL", "FR", "BL", "BR"}) {
    std::string ns = "default_stance." + std::string(prefix);

    LegJoints joints;
    node->get_parameter(ns + ".coxa", joints.coxa);
    node->get_parameter(ns + ".femur", joints.femur);
    node->get_parameter(ns + ".tibia", joints.tibia);

    default_joints[prefix] = joints;
  }

  // Get gait parameters
  double total_gait_time = node->get_parameter("gait.total_gait_time").as_double();
  double step_length = node->get_parameter("gait.step_length").as_double();
  double swing_height = node->get_parameter("gait.swing_height").as_double();
  int trajectory_points = node->get_parameter("gait.trajectory_points").as_int();
  double dt = total_gait_time / static_cast<double>(trajectory_points - 1);

  // Define leg prefixes and kinematic chain details
  std::vector<std::string> leg_prefixes = {"FL", "FR", "BL", "BR"};
  std::string base_link = "base_link";
  std::map<std::string, KDL::Chain> chains;
  std::map<std::string, std::shared_ptr<KDL::ChainIkSolverVel_pinv>> ik_solvers;
  std::map<std::string, std::shared_ptr<KDL::ChainFkSolverPos_recursive>> fk_solvers;

  // KDL tree
  KDL::Tree robot_tree;
  if (!kdl_parser::treeFromString(robot_description, robot_tree)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to build KDL tree");
    return -1;
  }

  for (const auto & prefix : leg_prefixes) {
    std::string tip_link = prefix + "_tibia_link";
    KDL::Chain chain;
    if (!robot_tree.getChain(base_link, tip_link, chain)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to get KDL chain for leg %s", prefix.c_str());
      return -1;
    }
    chains[prefix] = chain;
    ik_solvers[prefix] = std::make_shared<KDL::ChainIkSolverVel_pinv>(chains[prefix]);
    fk_solvers[prefix] = std::make_shared<KDL::ChainFkSolverPos_recursive>(chains[prefix]);
  }

  // Initialize joints from parameters
  std::map<std::string, KDL::JntArray> joint_positions;
  for (const auto & prefix : leg_prefixes) {
    const auto & joints = default_joints[prefix];
    joint_positions[prefix] = KDL::JntArray(3);
    joint_positions[prefix](0) = joints.coxa;
    joint_positions[prefix](1) = joints.femur;
    joint_positions[prefix](2) = joints.tibia;
  }

  // Compute default foot positions via FK
  std::map<std::string, Vec3<double>> default_foot_positions;
  for (const auto & prefix : leg_prefixes) {
    KDL::Frame foot_pose;
    if (fk_solvers[prefix]->JntToCart(joint_positions[prefix], foot_pose) < 0) {
      RCLCPP_ERROR(node->get_logger(), "FK failed for %s leg", prefix.c_str());
      return -1;
    }
    default_foot_positions[prefix] =
      Vec3<double>(foot_pose.p.x(), foot_pose.p.y(), foot_pose.p.z());

    RCLCPP_INFO(
      node->get_logger(), "%s foot position: x=%.4f y=%.4f z=%.4f", prefix.c_str(), foot_pose.p.x(),
      foot_pose.p.y(), foot_pose.p.z());
  }

  // Create trajectory generators for each leg
  std::map<std::string, FootSwingTrajectory<double>> trajectories;
  for (const auto & prefix : leg_prefixes) {
    trajectories[prefix] = FootSwingTrajectory<double>();
  }

  // Create main trajectory message
  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = node->now();

  // Populate joint names in correct order
  for (const auto & prefix : leg_prefixes) {
    trajectory_msg.joint_names.push_back(prefix + "_coxa_joint");
    trajectory_msg.joint_names.push_back(prefix + "_femur_joint");
    trajectory_msg.joint_names.push_back(prefix + "_tibia_joint");
  }

  RCLCPP_INFO(node->get_logger(), "Generating trot gait with %d points", trajectory_points);

  // Main generation loop
  for (int i = 0; i < trajectory_points; ++i) {
    double current_time = i * dt;
    bool is_phase_one = (current_time < total_gait_time / 2.0);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(current_time);
    std::vector<double> all_positions, all_velocities;

    // Process each leg
    for (const auto & prefix : leg_prefixes) {
      bool is_swing_leg = (is_phase_one && (prefix == "FL" || prefix == "BR")) ||
                          (!is_phase_one && (prefix == "FR" || prefix == "BL"));

      KDL::JntArray & current_joint_pos = joint_positions[prefix];
      KDL::JntArray joint_velocities(current_joint_pos.rows());

      if (is_swing_leg) {
        // Swing leg trajectory
        double phase_time = is_phase_one ? current_time : (current_time - total_gait_time / 2.0);
        double swing_phase = phase_time / (total_gait_time / 2.0);

        Vec3<double> p_start = default_foot_positions[prefix];
        Vec3<double> p_end = p_start + Vec3<double>(step_length, 0.0, 0.0);

        trajectories[prefix].setInitialPosition(p_start);
        trajectories[prefix].setFinalPosition(p_end);
        trajectories[prefix].setHeight(swing_height);
        trajectories[prefix].computeSwingTrajectoryBezier(swing_phase, total_gait_time / 2.0);

        Vec3<double> cart_vel_vec = trajectories[prefix].getVelocity();
        KDL::Twist cart_twist(
          KDL::Vector(cart_vel_vec[0], cart_vel_vec[1], cart_vel_vec[2]), KDL::Vector::Zero());

        ik_solvers[prefix]->CartToJnt(current_joint_pos, cart_twist, joint_velocities);
      } else {
        // Stance leg - zero velocity
        for (unsigned int j = 0; j < joint_velocities.rows(); ++j) {
          joint_velocities(j) = 0.0;
        }
      }

      // Update joint positions
      for (unsigned int j = 0; j < current_joint_pos.rows(); ++j) {
        current_joint_pos(j) += joint_velocities(j) * dt;
        all_positions.push_back(current_joint_pos(j));
        all_velocities.push_back(joint_velocities(j));
      }
    }

    point.positions = all_positions;
    point.velocities = all_velocities;
    trajectory_msg.points.push_back(point);
  }

  // Publish trajectory
  RCLCPP_INFO(node->get_logger(), "Publishing trajectory for all legs");
  pub->publish(trajectory_msg);

  rclcpp::sleep_for(std::chrono::seconds(1));
  rclcpp::shutdown();
  return 0;
}
