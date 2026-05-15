import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math


class GoalPoseClient(Node):
    def __init__(self):
        super().__init__('goal_pose_client')
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal_done = False  # Flag to track goal completion

    def send_goal(self, x, y, theta_degrees):
        """
        Send a navigation goal pose to the NavigateToPose action server.
        Args:
            x (float): X-coordinate of the target position.
            y (float): Y-coordinate of the target position.
            theta_degrees (float): Orientation angle in degrees.
        """
        # Convert inputs to floats
        x = float(x)
        y = float(y)
        theta_degrees = float(theta_degrees)

        self.goal_done = False  # Reset the flag before sending a new goal
        theta_radians = math.radians(theta_degrees)

        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose action server not available!')
            return False  # Server unavailable, return failure

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(theta_radians / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta_radians / 2.0)

        self.get_logger().info(f'Sending goal to x: {x:.4f}, y: {y:.4f}, theta: {theta_degrees:.2f} degrees')
        self.future = self.action_client.send_goal_async(goal_msg, self.feedback_callback)
        self.future.add_done_callback(self.goal_response_callback)

        return True  # Successfully sent goal

    def goal_response_callback(self, future):
        """Handle the response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by NavigateToPose action server!')
            self.goal_done = True  # Mark as done to prevent infinite loop
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

        self.goal_done = True  # Mark as done when the result is received


def send_navigation_goal(x, y, theta_degrees):
    """
    Sends a navigation goal to the TurtleBot3 using the /navigate_to_pose action.
    Args:
        x (float): X-coordinate of the target position.
        y (float): Y-coordinate of the target position.
        theta_degrees (float): Orientation angle in degrees.
    """
    rclpy.init()
    node = GoalPoseClient()

    try:
        success = node.send_goal(x, y, theta_degrees)
        if success:
            node.get_logger().info("Goal successfully sent!")
            while rclpy.ok():
                rclpy.spin_once(node)
                if node.goal_done:
                    break
        else:
            node.get_logger().error("Failed to send goal!")
    except KeyboardInterrupt:
        node.get_logger().info("Navigation interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
