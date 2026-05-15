import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math


class GoalPoseClient(Node):
    def __init__(self):
        super().__init__('goal_pose_client')

        # Action client for navigating to a pose
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def send_goal(self, x, y, theta_degrees):
        """Send a navigation goal pose to the NavigateToPose action server."""

        # Convert theta from degrees to radians
        theta_radians = math.radians(theta_degrees)

        # Wait for the action server to be available
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose action server not available!')
            rclpy.shutdown()  # Terminate if the server is unavailable
            return

        # Create the goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(theta_radians / 2.0)  # Convert radians to quaternion
        goal_msg.pose.pose.orientation.w = math.cos(theta_radians / 2.0)

        # Send the goal and wait for a response
        self.get_logger().info(f'Sending goal to x: {x}, y: {y}, theta: {theta_degrees} degrees')
        self.future = self.action_client.send_goal_async(goal_msg, self.feedback_callback)

        # Handle response when the goal is accepted or rejected
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by NavigateToPose action server!')
            rclpy.shutdown()  # Terminate if the goal is rejected
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    def result_callback(self, future):
        """Handle the result of the navigation action."""
        result = future.result()
        if result.status == 4:  # Status code for SUCCESS
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().error(f'Goal failed with status: {result.status}')

        # Shut down after receiving the result
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseClient()

    # Example goal coordinates (replace with desired values)
    x = 2.0  # Goal X coordinate
    y = 0.0  # Goal Y coordinate
    theta_degrees = 180  # Goal orientation in degrees (e.g., 90 degrees)

    # Send goal
    node.send_goal(x, y, theta_degrees)

    # Spin the node until the action completes or shuts down
    rclpy.spin(node)


if __name__ == '__main__':
    main()
