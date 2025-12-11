#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class DronePositionController(Node):
    def __init__(self):
        super().__init__('drone_position_controller')

        # SUBSCRIBE to odom
        self.create_subscription(Odometry, '/r3/odom', self.odom_callback, 10)

        # PUBLISH to cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/r3/cmd_vel', 10)

        # Timer for control loop (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        # Goals
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_z = 0.0
        self.goal_yaw = 0.0

        # Gains
        self.kp_xy = 1.0
        self.kp_yaw = 1.0

        # PI controller variables for altitude
        self.kp_z = 0.35
        self.ki_z = 0.05
        self.integral_z = 0.0
        self.integral_limit = 0.5   # anti-windup clamp

        # Time
        self.last_time = time.time()

        self.get_logger().info("Drone Position Controller (with PI altitude control) Started.")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

    def control_loop(self):
        # Compute dt
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt == 0:
            return

        # Position errors
        ex = self.goal_x - self.x
        ey = self.goal_y - self.y
        ez = self.goal_z - self.z
        eyaw = self.wrap(self.goal_yaw - self.yaw)

        # ----------------------------------------
        #  PI CONTROL FOR ALTITUDE (Z-AXIS)
        # ----------------------------------------
        self.integral_z += ez * dt
        # Anti-windup
        self.integral_z = max(min(self.integral_z, self.integral_limit), -self.integral_limit)

        vz = self.kp_z * ez + self.ki_z * self.integral_z
        vz = max(min(vz, 1.0), -1.0)  # clamp output

        # ----------------------------------------
        #  XY + YAW proportional control
        # ----------------------------------------
        vx = self.kp_xy * ex
        vy = self.kp_xy * ey
        wz = self.kp_yaw * eyaw

        # Clamp XY
        vx = max(min(vx, 1.0), -1.0)
        vy = max(min(vy, 1.0), -1.0)
        wz = max(min(wz, 1.0), -1.0)

        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

    def goto(self, x, y, z, yaw_deg):
        """Set a new goal pose for the drone."""
        self.goal_x = x
        self.goal_y = y
        self.goal_z = z
        self.goal_yaw = math.radians(yaw_deg)
        self.integral_z = 0.0   # reset integral on new command
        self.get_logger().info(f"New goal: ({x}, {y}, {z}, yaw={yaw_deg}°)")

    @staticmethod
    def wrap(a):
        return math.atan2(math.sin(a), math.cos(a))


def main(args=None):
    rclpy.init(args=args)
    node = DronePositionController()

    # Example: hover at (0,0,2m)
    node.goto(0.0, 0.0, 2.4, 0.0)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
