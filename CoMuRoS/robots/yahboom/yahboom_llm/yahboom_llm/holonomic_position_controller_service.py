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

from robot_interface.srv import GotoPoseHolonomic

NAMESPACE = 'r2'

class HolonomicPositionController(Node):
    def __init__(self):
        super().__init__("holonomic_position_controller")
        self.declare_parameter('namespace', NAMESPACE)
        self.namespace: str = self.get_parameter('namespace').value


        # Callback groups
        self.service_cb_group = ReentrantCallbackGroup()
        self.subscription_cb_group = ReentrantCallbackGroup()

        # Subscribers
        self.create_subscription(
            Odometry,
            f"/{self.namespace}/odom",
            self.odom_callback,
            10,
            callback_group=self.subscription_cb_group
        )

        self.create_subscription(
            Bool,
            f"/{self.namespace}/cancel_goto_pose_goal",
            self.cancel_callback,
            10,
            callback_group=self.subscription_cb_group
        )

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, f"/{self.namespace}/cmd_vel", 10)

        # Service
        self.srv = self.create_service(
            GotoPoseHolonomic,
            f"/{self.namespace}/goto_pose",
            self.goto_service_callback,
            callback_group=self.service_cb_group
        )

        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        self.cancel_requested = False

        # Gains
        self.kp_xy = 1.2
        self.kp_yaw = 2.0

        self.get_logger().info("Holonomic Blocking Controller Initialized.")

    # -----------------------------------------------------
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

        self.odom_received = True

    # -----------------------------------------------------
    def cancel_callback(self, msg):
        if msg.data:
            self.cancel_requested = True
            self.get_logger().warn("[Holonomic] CANCEL request received!")

    # -----------------------------------------------------
    # BLOCKING SERVICE CALLBACK
    # -----------------------------------------------------
    def goto_service_callback(self, request, response):
        self.get_logger().info(
            f"[Holonomic] New Goal: ({request.x}, {request.y}, {request.yaw_deg}°)"
        )

        goal_x = request.x
        goal_y = request.y
        goal_yaw = math.radians(request.yaw_deg)

        self.cancel_requested = False

        # Ensure odom received before movement
        start_wait = time.time()
        while not self.odom_received and time.time() - start_wait < 3.0:
            time.sleep(0.01)

        if not self.odom_received:
            response.accepted = False
            response.success = False
            response.message = "No odometry data received."
            return response

        # Send acceptance (final success comes later)
        response.accepted = True
        response.success = False
        response.message = "Holonomic goal execution started."
        self.get_logger().info("Starting holonomic movement...")
        
        # Movement loop
        timeout = time.time() + 100000.0  # 60 sec timeout

        while rclpy.ok():

            # --- Timeout ---
            if time.time() > timeout:
                self.stop()
                response.success = False
                response.message = "Holonomic goal timed out."
                self.get_logger().warn(response.message)
                return response

            # --- Cancel ---
            if self.cancel_requested:
                self.stop()
                response.success = False
                response.message = "Holonomic goal canceled."
                self.get_logger().warn(response.message)
                return response

            # Compute errors
            ex = goal_x - self.x
            ey = goal_y - self.y
            eyaw = self.wrap(goal_yaw - self.yaw)

            dist = math.sqrt(ex*ex + ey*ey)

            cmd = Twist()
            cmd.linear.x = max(min(self.kp_xy * ex, 1.0), -1.0)
            cmd.linear.y = max(min(self.kp_xy * ey, 1.0), -1.0)
            cmd.angular.z = max(min(self.kp_yaw * eyaw, 1.0), -1.0)

            self.cmd_pub.publish(cmd)

            self.get_logger().info(
                f"[Holonomic] ex={ex:.3f}, ey={ey:.3f}, eyaw={eyaw:.3f}, dist={dist:.3f}"
            )

            # --- Goal reached ---
            if dist < 0.05 and abs(eyaw) < 0.05:
                self.stop()
                response.success = True
                response.message = "Holonomic goal reached!"
                self.get_logger().info("[Holonomic] Goal reached!")
                return response

            time.sleep(0.05)

    # -----------------------------------------------------
    def stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def wrap(a):
        return math.atan2(math.sin(a), math.cos(a))


def main(args=None):
    rclpy.init(args=args)
    node = HolonomicPositionController()

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
