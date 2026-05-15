import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point, Quaternion

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        # Publisher to your robot's pose topic (customize this topic if needed)
        self.publisher_ = self.create_publisher(Pose, 'body_pose', 10)

        # Timer to call publish_pose() once after 1 second
        # self.timer = self.create_timer(1.0, self.publish_pose)
        self.publish_pose()

    def publish_pose(self):
        pose_msg = Pose()

        # Set position (x, y, z)
        pose_msg.position = Point(x=0.0, y=0.0, z=-0.1)

        # Set orientation (quaternion: x, y, z, w)
        pose_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.publisher_.publish(pose_msg)
        self.get_logger().info(f"Published Pose:\n{pose_msg}")

        # Optional: stop publishing after one message
        # self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
