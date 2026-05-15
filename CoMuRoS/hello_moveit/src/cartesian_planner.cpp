#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "cartesian_planner", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("cartesian_planner");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Get the current pose of the end-effector
  geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose;

  // Define waypoints for the Cartesian path
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose);

  // Add waypoints based on the documentation example
  geometry_msgs::msg::Pose target_pose = start_pose;

  // Move up
  target_pose.position.z += 0.1;
  waypoints.push_back(target_pose);

  // Move right
  target_pose.position.y -= 0.1;
  waypoints.push_back(target_pose);

  // Move up and left
  target_pose.position.z += 0.1;
  target_pose.position.y += 0.1;
  target_pose.position.x -= 0.1;
  waypoints.push_back(target_pose);

  // Plan the Cartesian path
  const double eef_step = 0.01;         // End-effector step size (1 cm)
  const double jump_threshold = 0.0;   // Disable jump threshold
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_interface.computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory);

  // Log the results
  if (fraction > 0.99)
  {
    RCLCPP_INFO(logger, "Cartesian path computed successfully (%.2f%% of the waypoints)", fraction * 100.0);
    move_group_interface.execute(trajectory);
    RCLCPP_INFO(logger, "Cartesian path executed successfully.");
  }
  else
  {
    RCLCPP_WARN(logger, "Cartesian path planning failed (%.2f%% of the waypoints)", fraction * 100.0);
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
