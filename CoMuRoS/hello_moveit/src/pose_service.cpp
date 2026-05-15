#include "rclcpp/rclcpp.hpp"
#include "hello_moveit_interfaces/srv/set_pose.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>

class PoseServiceNode : public rclcpp::Node
{
public:
  PoseServiceNode() : Node("pose_service_node"), move_group_initialized_(false)
  {
    // Create the service
    service_ = this->create_service<hello_moveit_interfaces::srv::SetPose>(
        "set_end_effector_pose",
        std::bind(&PoseServiceNode::handleSetPoseRequest, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "PoseServiceNode initialized. Waiting for service calls...");
  }

  void initializeMoveGroupInterface()
  {
    if (!move_group_initialized_)
    {
      RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface...");
      move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), "arm");
      move_group_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized successfully.");
    }
  }

private:
  void handleSetPoseRequest(
      const std::shared_ptr<hello_moveit_interfaces::srv::SetPose::Request> request,
      std::shared_ptr<hello_moveit_interfaces::srv::SetPose::Response> response)
  {
    initializeMoveGroupInterface(); // Ensure MoveGroupInterface is initialized

    RCLCPP_INFO(this->get_logger(), "Received a request to set pose:");
    RCLCPP_INFO(this->get_logger(), " - x: %f, y: %f, z: %f",
                request->pose.position.x, request->pose.position.y, request->pose.position.z);
    RCLCPP_INFO(this->get_logger(), " - orientation (w): %f", request->pose.orientation.w);

    // Set the pose target
    move_group_interface_->setPoseTarget(request->pose);

    // Plan and execute the motion
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Planning successful. Executing...");
      move_group_interface_->execute(plan);
      response->success = true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed.");
      response->success = false;
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::Service<hello_moveit_interfaces::srv::SetPose>::SharedPtr service_;
  bool move_group_initialized_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Use a shared_ptr to manage the node instance
  auto node = std::make_shared<PoseServiceNode>();

  // Ensure MoveGroupInterface is initialized after construction
  node->initializeMoveGroupInterface();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
