#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/srv/set_bool.hpp>

class GripperControlService : public rclcpp::Node
{
public:
    GripperControlService() : Node("gripper_control_service"), logger_(this->get_logger())
    {
        // Initialize MoveIt Interface for the Gripper
        using moveit::planning_interface::MoveGroupInterface;
        move_group_gripper_ = std::make_shared<MoveGroupInterface>(
            std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), "gripper");
        move_group_gripper_->setPlanningTime(5.0);

        // Create Service
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "control_gripper",
            std::bind(&GripperControlService::handle_service, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(logger_, "Gripper control service is ready.");
    }

private:
    void handle_service(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        double width = request->data ? 0.04 : 0.0;  // True = Open, False = Close
        std::vector<double> gripper_positions = {width};
        move_group_gripper_->setJointValueTarget(gripper_positions);

        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        bool success = static_cast<bool>(move_group_gripper_->plan(gripper_plan));

        if (success)
        {
            move_group_gripper_->execute(gripper_plan);
            response->success = true;
            response->message = request->data ? "Gripper opened successfully." : "Gripper closed successfully.";
            RCLCPP_INFO(logger_, response->message.c_str());
        }
        else
        {
            response->success = false;
            response->message = "Gripper movement failed!";
            RCLCPP_ERROR(logger_, response->message.c_str());
        }
    }

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_;
    rclcpp::Logger logger_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperControlService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
