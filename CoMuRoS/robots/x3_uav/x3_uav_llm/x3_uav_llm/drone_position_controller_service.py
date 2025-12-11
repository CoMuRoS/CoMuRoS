#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import time

from robot_interface.srv import GotoPoseDrone

NAMESPACE = 'r3'

class DronePositionController(Node):
    def __init__(self):
        super().__init__("drone_position_controller")
        self.declare_parameter('namespace', NAMESPACE)
        self.namespace: str = self.get_parameter('namespace').value

        # Callback groups
        self.service_cb_group = ReentrantCallbackGroup()
        self.subscription_cb_group = ReentrantCallbackGroup()

        # SUBSCRIBE to odom
        self.create_subscription(
            Odometry, f"/{self.namespace}/odom", self.odom_callback, 10,
            callback_group=self.subscription_cb_group
        )

        # CANCEL topic
        self.create_subscription(
            Bool, f"/{self.namespace}/cancel_goto_pose_goal", self.cancel_callback, 10,
            callback_group=self.subscription_cb_group
        )

        # PUBLISH to cmd_vel
        self.cmd_pub = self.create_publisher(Twist, f"/{self.namespace}/cmd_vel", 10)

        # SERVICE SERVER (blocking)
        self.srv = self.create_service(
            GotoPoseDrone,
            f"/{self.namespace}/goto_pose",
            self.goto_service_callback,
            callback_group=self.service_cb_group
        )

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.odom_received = False

        # Cancel flag
        self.cancel_requested = False

        # Gains
        self.kp_xy = 1.0
        self.kp_yaw = 1.0
        self.kp_z = 0.35
        self.ki_z = 0.09
        self.integral_z = 0.0
        self.integral_limit = 0.5

        self.get_logger().info("Blocking Drone Controller Initialized.")

    # -----------------------------------------------------
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        siny = 2 * (q.w*q.z + q.x*q.y)
        cosy = 1 - 2 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

        self.odom_received = True

    # -----------------------------------------------------
    def cancel_callback(self, msg):
        if msg.data:
            self.cancel_requested = True
            self.get_logger().warn("[Drone] CANCEL request received!")

    # -----------------------------------------------------
    # BLOCKING SERVICE CALLBACK
    # -----------------------------------------------------
    def goto_service_callback(self, request, response):
        goal_x = request.x
        goal_y = request.y
        goal_z = request.z
        goal_yaw = math.radians(request.yaw_deg)

        self.cancel_requested = False
        self.integral_z = 0.0

        # Wait for odom before moving
        t0 = time.time()
        while not self.odom_received and time.time() - t0 < 5.0:
            time.sleep(0.01)

        if not self.odom_received:
            response.accepted = False
            response.success = False
            response.message = "No odometry data!"
            return response

        # SERVICE ACCEPTED
        response.accepted = True
        response.success = False
        response.message = "Drone goal execution started."
        self.get_logger().info(response.message)

        timeout = time.time() + 60.0  # 60 sec timeout
        last_time = time.time()

        while rclpy.ok():

            # TIMEOUT
            if time.time() > timeout:
                self.stop()
                response.success = False
                response.message = "Drone goal timed out!"
                return response

            # CANCEL
            if self.cancel_requested:
                self.stop()
                response.success = False
                response.message = "Drone goal canceled!"
                return response

            # Time delta
            now = time.time()
            dt = now - last_time
            last_time = now
            if dt <= 0:
                dt = 0.01

            # Errors
            ex = goal_x - self.x
            ey = goal_y - self.y
            ez = goal_z - self.z
            eyaw = self.wrap(goal_yaw - self.yaw)

            # ----------------------
            # ALTITUDE PI CONTROL
            # ----------------------
            self.integral_z += ez * dt
            self.integral_z = max(min(self.integral_z, self.integral_limit), -self.integral_limit)

            vz = self.kp_z * ez + self.ki_z * self.integral_z
            vz = max(min(vz, 1.0), -1.0)

            # XY + yaw control
            vx = max(min(self.kp_xy * ex, 1.0), -1.0)
            vy = max(min(self.kp_xy * ey, 1.0), -1.0)
            wz = max(min(self.kp_yaw * eyaw, 1.0), -1.0)

            # Publish command
            cmd = Twist()
            cmd.linear.x = vx
            cmd.linear.y = vy
            cmd.linear.z = vz
            cmd.angular.z = wz
            self.cmd_pub.publish(cmd)

            # Reach condition
            if abs(ex) < 0.05 and abs(ey) < 0.05 and abs(ez) < 0.05 and abs(eyaw) < 0.05:
                self.stop()
                response.success = True
                response.message = "Drone reached the goal!"
                self.get_logger().info("[Drone] Goal reached!")
                return response

            time.sleep(0.05)

    # -----------------------------------------------------
    def stop(self):
        """Stop the drone safely."""
        self.cmd_pub.publish(Twist())

    @staticmethod
    def wrap(a):
        return math.atan2(math.sin(a), math.cos(a))


def main(args=None):
    rclpy.init(args=args)
    node = DronePositionController()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
