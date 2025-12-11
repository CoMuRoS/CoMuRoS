#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class HolonomicPositionController(Node):
    def __init__(self):
        super().__init__('holonomic_position_controller')

        # SUBSCRIBE to odom
        self.create_subscription(Odometry, '/r1/odom', self.odom_callback, 10)

        # PUBLISH to cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/r1/cmd_vel', 10)

        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Goal
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0

        # Gains
        self.kp_xy = 1.0
        self.kp_yaw = 1.0

        self.get_logger().info("Holonomic Position Controller Started.")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

    def control_loop(self):
        # Compute errors
        ex = self.goal_x - self.x
        ey = self.goal_y - self.y
        eyaw = self.wrap(self.goal_yaw - self.yaw)

        # Velocity commands (holonomic)
        cmd = Twist()
        cmd.linear.x = self.kp_xy * ex
        cmd.linear.y = self.kp_xy * ey
        cmd.angular.z = self.kp_yaw * eyaw

        # Limit speeds (optional)
        cmd.linear.x = max(min(cmd.linear.x, 1.0), -1.0)
        cmd.linear.y = max(min(cmd.linear.y, 1.0), -1.0)
        cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)

        self.cmd_pub.publish(cmd)

    def goto(self, x, y, yaw_deg):
        """Call this to move robot to a goal pose."""
        self.goal_x = x
        self.goal_y = y
        self.goal_yaw = math.radians(yaw_deg)
        self.get_logger().info(f"New goal set: ({x}, {y}, {yaw_deg}°)")

    @staticmethod
    def wrap(a):
        return math.atan2(math.sin(a), math.cos(a))


def main(args=None):
    rclpy.init(args=args)
    node = HolonomicPositionController()

    # Example goal:
    node.goto(-1.0, -1.0, 0.0)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
