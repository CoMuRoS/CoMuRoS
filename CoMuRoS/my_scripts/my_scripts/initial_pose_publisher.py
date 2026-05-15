import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # Publisher for initial pose
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Timer to initialize the robot's pose
        self.timer = self.create_timer(2.0, self.publish_initial_pose)

    def publish_initial_pose(self):
        """Publish the robot's initial pose at the origin."""

        # Create and publish the initial pose message
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.frame_id = 'map'
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.pose.pose.position.x = 0.0  # Robot's initial X position
        initial_pose_msg.pose.pose.position.y = 0.0  # Robot's initial Y position
        initial_pose_msg.pose.pose.orientation.z = 0.0
        initial_pose_msg.pose.pose.orientation.w = 1.0
        initial_pose_msg.pose.covariance = [0.0] * 36  # Valid covariance matrix

        self.initial_pose_publisher.publish(initial_pose_msg)
        self.get_logger().info(f'Published initial pose: {initial_pose_msg}')

        # Cancel the timer after publishing the initial pose
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    initial_pose_publisher = InitialPosePublisher()

    # Spin the node until it is manually stopped
    rclpy.spin(initial_pose_publisher)

    # Clean up when the node is stopped
    initial_pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
