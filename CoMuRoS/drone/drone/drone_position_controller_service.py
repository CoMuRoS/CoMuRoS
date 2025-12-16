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

class PIDController:
    """Generic PID controller."""
    def __init__(self, kp, ki, kd, output_limit=1.0, integral_limit=0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        
        self.integral = 0.0
        self.prev_error = 0.0
        
    def reset(self):
        """Reset PID state."""
        self.integral = 0.0
        self.prev_error = 0.0
        
    def compute(self, error, dt):
        """Compute PID output."""
        # Proportional
        p_term = self.kp * error
        
        # Integral with anti-windup
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        i_term = self.ki * self.integral
        
        # Derivative
        if dt > 0:
            d_term = self.kd * (error - self.prev_error) / dt
        else:
            d_term = 0.0
        self.prev_error = error
        
        # Total output with saturation
        output = p_term + i_term + d_term
        output = max(min(output, self.output_limit), -self.output_limit)
        
        return output


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

        # PID Controllers for each axis
        self.pid_x = PIDController(kp=2.5, ki=0.1, kd=0.8, output_limit=1.0, integral_limit=1.0)
        self.pid_y = PIDController(kp=2.5, ki=0.1, kd=0.8, output_limit=1.0, integral_limit=1.0)
        self.pid_z = PIDController(kp=2.5, ki=0.2, kd=0.6, output_limit=1.0, integral_limit=1.0)
        self.pid_yaw = PIDController(kp=1.0, ki=0.1, kd=0.5, output_limit=1.0)

        # Thresholds
        self.position_threshold = 0.05  # meters
        self.yaw_threshold = 0.05  # radians (~3 degrees)

        self.start_drone()

        self.get_logger().info("Three-Step PID Drone Controller Initialized.")

    def start_drone(self):
        msg = Twist()
        msg.angular.z = 0.1
        self.cmd_pub.publish(msg)
        time.sleep(1)
        msg = Twist()
        self.cmd_pub.publish(msg)

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
    # BLOCKING SERVICE CALLBACK - THREE STEP APPROACH
    # -----------------------------------------------------
    def goto_service_callback(self, request, response):
        goal_x = request.x
        goal_y = request.y
        goal_z = request.z
        goal_yaw = math.radians(request.yaw_deg)

        self.cancel_requested = False
        
        # Reset all PID controllers
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()

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

        # ========================================
        # STEP 1: Reach Position (X, Y, Z)
        # ========================================
        self.get_logger().info("[Drone] Step 1: Moving to position...")
        result = self.move_to_position(goal_x, goal_y, goal_z, timeout=400000005.0)
        
        if not result["success"]:
            self.stop()
            response.success = False
            response.message = result["message"]
            return response

        # ========================================
        # STEP 2: Correct Yaw
        # ========================================
        self.get_logger().info("[Drone] Step 2: Correcting yaw...")
        result = self.correct_yaw(goal_yaw, timeout=10000100005.0)
        
        if not result["success"]:
            self.stop()
            response.success = False
            response.message = result["message"]
            return response

        # ========================================
        # STEP 3: Fine-tune Both Position and Yaw
        # ========================================
        self.get_logger().info("[Drone] Step 3: Fine-tuning position and yaw...")
        result = self.fine_tune_both(goal_x, goal_y, goal_z, goal_yaw, timeout=20000000000.0)
        
        if not result["success"]:
            self.stop()
            response.success = False
            response.message = result["message"]
            return response

        # SUCCESS
        self.stop()
        response.success = True
        response.message = "Drone reached goal position and yaw!"
        self.get_logger().info("[Drone] Goal fully reached!")
        return response

# -----------------------------------------------------
    def move_to_position(self, goal_x, goal_y, goal_z, timeout=45.0):
        """Step 1: Move to target position using PID control."""
        timeout_time = time.time() + timeout
        last_time = time.time()

        while rclpy.ok():
            if time.time() > timeout_time:
                return {"success": False, "message": "Position goal timed out!"}
            if self.cancel_requested:
                return {"success": False, "message": "Position goal canceled!"}

            now = time.time()
            dt = now - last_time
            last_time = now
            if dt <= 0:
                dt = 0.01

            # Compute errors in WORLD frame
            ex = goal_x - self.x
            ey = goal_y - self.y
            ez = goal_z - self.z

            # PID control in WORLD frame
            vx_world = self.pid_x.compute(ex, dt)
            vy_world = self.pid_y.compute(ey, dt)
            vz = self.pid_z.compute(ez, dt)

            # Transform to BODY frame
            vx_body = vx_world * math.cos(self.yaw) + vy_world * math.sin(self.yaw)
            vy_body = -vx_world * math.sin(self.yaw) + vy_world * math.cos(self.yaw)

            # Publish command
            cmd = Twist()
            cmd.linear.x = vx_body
            cmd.linear.y = vy_body
            cmd.linear.z = vz
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

            # Check if position reached
            if (abs(ex) < self.position_threshold and 
                abs(ey) < self.position_threshold and 
                abs(ez) < self.position_threshold):
                self.get_logger().info("[Drone] Position reached!")
                return {"success": True, "message": "Position reached"}

            time.sleep(0.05)

    # -----------------------------------------------------
    def correct_yaw(self, goal_yaw, timeout=15.0):
        """Step 2: Correct yaw while maintaining position."""
        timeout_time = time.time() + timeout
        last_time = time.time()

        target_x = self.x
        target_y = self.y
        target_z = self.z
        
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()

        while rclpy.ok():
            if time.time() > timeout_time:
                return {"success": False, "message": "Yaw correction timed out!"}
            if self.cancel_requested:
                return {"success": False, "message": "Yaw correction canceled!"}

            now = time.time()
            dt = now - last_time
            last_time = now
            if dt <= 0:
                dt = 0.01

            # Position errors in WORLD frame
            ex = target_x - self.x
            ey = target_y - self.y
            ez = target_z - self.z
            eyaw = self.wrap(goal_yaw - self.yaw)

            # PID control in WORLD frame
            vx_world = self.pid_x.compute(ex, dt)
            vy_world = self.pid_y.compute(ey, dt)
            vz = self.pid_z.compute(ez, dt)
            wz = self.pid_yaw.compute(eyaw, dt) * 0.5

            # Transform to BODY frame
            vx_body = vx_world * math.cos(self.yaw) + vy_world * math.sin(self.yaw)
            vy_body = -vx_world * math.sin(self.yaw) + vy_world * math.cos(self.yaw)

            # Publish command
            cmd = Twist()
            cmd.linear.x = vx_body
            cmd.linear.y = vy_body
            cmd.linear.z = vz
            cmd.angular.z = wz
            self.cmd_pub.publish(cmd)

            if abs(eyaw) < self.yaw_threshold:
                self.get_logger().info("[Drone] Yaw corrected!")
                return {"success": True, "message": "Yaw corrected"}

            time.sleep(0.05)

    # -----------------------------------------------------
    def fine_tune_both(self, goal_x, goal_y, goal_z, goal_yaw, timeout=20.0):
        """Step 3: Fine-tune both position and yaw together."""
        timeout_time = time.time() + timeout
        last_time = time.time()
        
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()
        
        self.get_logger().info(f"[Step 3] Start pos: ({self.x:.3f}, {self.y:.3f}, {self.z:.3f}), yaw: {math.degrees(self.yaw):.1f}°")
        self.get_logger().info(f"[Step 3] Goal pos: ({goal_x:.3f}, {goal_y:.3f}, {goal_z:.3f}), yaw: {math.degrees(goal_yaw):.1f}°")
        
        log_counter = 0

        while rclpy.ok():
            if time.time() > timeout_time:
                return {"success": False, "message": "Fine-tuning timed out!"}
            if self.cancel_requested:
                return {"success": False, "message": "Fine-tuning canceled!"}

            now = time.time()
            dt = now - last_time
            last_time = now
            if dt <= 0:
                dt = 0.01

            # Compute all errors in WORLD frame
            ex = goal_x - self.x
            ey = goal_y - self.y
            ez = goal_z - self.z
            eyaw = self.wrap(goal_yaw - self.yaw)

            # PID control in WORLD frame
            vx_world = self.pid_x.compute(ex, dt)
            vy_world = self.pid_y.compute(ey, dt)
            vz = self.pid_z.compute(ez, dt)
            wz = self.pid_yaw.compute(eyaw, dt)

            # Transform to BODY frame
            vx_body = vx_world * math.cos(self.yaw) + vy_world * math.sin(self.yaw)
            vy_body = -vx_world * math.sin(self.yaw) + vy_world * math.cos(self.yaw)

            # Log every 20 iterations
            log_counter += 1
            if log_counter % 20 == 0:
                self.get_logger().info(
                    f"[Step 3] Errors: x={ex:.3f}, y={ey:.3f}, z={ez:.3f}, yaw={math.degrees(eyaw):.1f}° | "
                    f"Cmds: vx={vx_body:.2f}, vy={vy_body:.2f}, vz={vz:.2f}, wz={wz:.2f}"
                )

            # Publish command
            cmd = Twist()
            cmd.linear.x = vx_body
            cmd.linear.y = vy_body
            cmd.linear.z = vz
            cmd.angular.z = wz
            self.cmd_pub.publish(cmd)

            # Check convergence
            if (abs(ex) < self.position_threshold and 
                abs(ey) < self.position_threshold and 
                abs(ez) < self.position_threshold and
                abs(eyaw) < self.yaw_threshold):
                self.get_logger().info("[Drone] Fine-tuning complete!")
                return {"success": True, "message": "Fine-tuning complete"}

            time.sleep(0.05)

    # -----------------------------------------------------
    def stop(self):
        """Stop the drone safely."""
        self.cmd_pub.publish(Twist())

    @staticmethod
    def wrap(a):
        """Wrap angle to [-pi, pi]."""
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