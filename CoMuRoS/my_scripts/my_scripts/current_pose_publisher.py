import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PoseStamped
import math


class TurtleBot3PosePublisher(Node):
    def __init__(self):
        super().__init__('turtlebot3_pose_publisher')

        # Create a TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a publisher for the /current_pose topic
        self.pose_publisher = self.create_publisher(PoseStamped, '/current_pose', 10)

        # Introduce a 4-second delay before starting the transform lookup
        self.timer_once = self.create_timer(4.0, self.start_publishing)

    def start_publishing(self):
        """Start the periodic publishing of the pose after the delay."""
        self.timer_once.cancel()  # Cancel the one-time timer
        self.timer = self.create_timer(1.0, self.publish_pose)  # Start periodic timer

    def publish_pose(self):
        try:
            # Lookup transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )

            # Extract translation (position)
            position = transform.transform.translation
            x, y, z = position.x, position.y, position.z

            # Extract rotation (orientation)
            rotation = transform.transform.rotation
            quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]

            # Create and populate the PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            pose_msg.pose.orientation.x = rotation.x
            pose_msg.pose.orientation.y = rotation.y
            pose_msg.pose.orientation.z = rotation.z
            pose_msg.pose.orientation.w = rotation.w

            # Publish the pose
            self.pose_publisher.publish(pose_msg)

        except (LookupException, ConnectivityException, ExtrapolationException):
            pass  # Silently ignore TF lookup errors

    @staticmethod
    def quaternion_to_yaw(quaternion):
        """Convert a quaternion into a yaw angle (in degrees)."""
        x, y, z, w = quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))  # Convert to degrees


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
