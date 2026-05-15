
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion , Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from enum import Enum
import math
import time

class MotionMode(Enum):
    SIT = 0
    STAND = 1
    GOTO = 2
    STOP = 3
    MOVE_FORWARD = 4
    MOVE_BACKWARD = 5

class Go2Interface(Node):
    def __init__(self):
        super().__init__('go2_interface')

        self.publisher_ = self.create_publisher(Pose, 'body_pose', 10)
        self.twist_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.imu_subscriber = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom/filtered', self.odom_callback, 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_ang_z = 0.0

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.angle = 0.0

        self.step = 0.005
        self.timer_period = 0.05
        self.timer = None
        self.mode = MotionMode.STOP
        self._task_done = True

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_ang_z = math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def publish_pose(self):
        pose_msg = Pose()
        pose_msg.position = Point(x=self.current_x, y=self.current_y, z=self.current_z)
        if self.mode in [MotionMode.SIT, MotionMode.STAND]:
            pose_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        else:
            if abs(self.angle - self.current_ang_z) > self.step:
                pose_msg.orientation = self.yaw_to_quaternion(self.current_ang_z)
            else:
                pose_msg.orientation = self.yaw_to_quaternion(self.angle)
        self.publisher_.publish(pose_msg)

    def publish_vel(self, x=0.0, y=0.0, z=0.0):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = z
        self.twist_publisher_.publish(twist)

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def sit(self):
        self._task_done = False
        self.target_z = -0.1
        self.mode = MotionMode.SIT
        while not self._task_done:
            self.smooth_move()
            rclpy.spin_once(self, timeout_sec=self.timer_period)
        time.sleep(1)

    def stand(self):
        self._task_done = False
        self.target_z = 0.0
        self.mode = MotionMode.STAND
        while not self._task_done:
            self.smooth_move()
            rclpy.spin_once(self, timeout_sec=self.timer_period)
        time.sleep(1)

    def smooth_move(self):
        if abs(self.target_z - self.current_z) < self.step:
            self.current_z = self.target_z
            self.publish_pose()
            self._task_done = True
            return
        direction = 1.0 if self.target_z > self.current_z else -1.0
        self.current_z += direction * self.step * 0.6
        self.publish_pose()

    def move_forward(self):
        self._task_done = False
        self.publish_vel(x=0.3)
        start = time.time()
        while time.time() - start < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.publish_vel(0.0, 0.0, 0.0)
        self._task_done = True

    def move_backward(self):
        self._task_done = False
        self.publish_vel(x=-0.3)
        start = time.time()
        print("started")
        while time.time() - start < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.publish_vel(0.0, 0.0, 0.0)
        self._task_done = True
        print("finsih")


    def go_to_position(self, x: float, y: float, z: float):
        self._task_done = False
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.angle = math.atan2(y - self.current_y, x - self.current_x)
        self.mode = MotionMode.GOTO

        while abs(self.angle - self.current_ang_z) > self.step:
            direction = 1.0 if self.angle > self.current_ang_z else -1.0
            self.publish_vel(0.0, 0.0, direction)
            rclpy.spin_once(self, timeout_sec=self.timer_period)
        self.publish_vel(0.0, 0.0, 0.0)

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.hypot(dx, dy)
        duration = distance / 0.5
        self.publish_vel(x=0.5)
        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.publish_vel(0.0, 0.0, 0.0)
        self._task_done = True

    def stop(self):
        self._task_done = False
        self.publish_vel(0.0, 0.0, 0.0)
        self._task_done = True


# ================== MAIN =============================

def main(args=None):
    rclpy.init(args=args)
    node = Go2Interface()

    # Example: Sit, then stand after 3 seconds
    # node.sit()

    # def stand_after_delay():
    #     node.stand()

    # node.create_timer(3.0, stand_after_delay)

    # node.move_forward()
    # node.move_backward()


    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
