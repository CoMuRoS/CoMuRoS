#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create MoveIt MoveGroup Interface for the arm
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_arm = MoveGroupInterface(node, "arm");
  move_group_arm.setPlanningTime(5.0);  // <-- Added here

  // Create MoveIt MoveGroup Interface for the gripper
  auto move_group_gripper = MoveGroupInterface(node, "gripper");
  move_group_gripper.setPlanningTime(5.0);  // <-- Added here

  // Log the planning groups available
  auto robot_model_loader = move_group_arm.getRobotModel();
  auto joint_model_group_names = robot_model_loader->getJointModelGroupNames();
  RCLCPP_INFO(logger, "Available planning groups:");
  for (const auto& group_name : joint_model_group_names)
  {
    RCLCPP_INFO(logger, " - %s", group_name.c_str());
  }

  // Function to set the gripper's width
  auto set_gripper_position = [&move_group_gripper, &logger](double width) {
    std::vector<double> gripper_positions = {width};  // Assuming a single joint for the gripper
    move_group_gripper.setJointValueTarget(gripper_positions);
    
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool success = static_cast<bool>(move_group_gripper.plan(gripper_plan));

    if (success) {
      move_group_gripper.execute(gripper_plan);
      RCLCPP_INFO(logger, "Gripper moved to position: %f", width);
    } else {
      RCLCPP_ERROR(logger, "Gripper movement failed!");
    }
  };

  // Open the gripper before moving the arm
  set_gripper_position(0.04);  // Assuming 4 cm open position

  // Set a target Pose for the arm
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;   // No rotation for simplicity (identity quaternion)
    msg.position.x = 0.15;     // Reachable x-coordinate
    msg.position.y = 0.0;      // Aligned with the robot's center
    msg.position.z = 0.3;      // Above the base plane
    return msg;
  }();
  move_group_arm.setPoseTarget(target_pose);

  // Create a plan to move the arm
  auto const [success, plan] = [&move_group_arm] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_arm.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the arm movement
  if (success)
  {
    move_group_arm.execute(plan);
    RCLCPP_INFO(logger, "Arm movement executed successfully.");

    // Close the gripper after reaching the target
    set_gripper_position(0.0);  // Assuming 0 cm is closed position
  }
  else
  {
    RCLCPP_ERROR(logger, "Arm planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
