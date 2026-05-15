import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion , Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
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

        self.io_group = ReentrantCallbackGroup()        # For pub/subs
        self.goto_timer_group = MutuallyExclusiveCallbackGroup()  # For timer isolation

        self.publisher_ = self.create_publisher(Pose, 'body_pose', 10, callback_group=self.io_group)
        self.twist_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10, callback_group=self.io_group)
        self.status_pub = self.create_publisher(String, 'task_status', 10, callback_group=self.io_group)
        self.go2_goto_status_pub = self.create_publisher(Bool , "/go2_goto_status", 10, callback_group=self.io_group )
        
        self.imu_subscriber = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10, callback_group=self.io_group)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom/ground_truth', self.odom_callback, 10, callback_group=self.io_group)

        self.object_poses = {
            "kitchen table" : [-3.610566 , 1.89094, 0.2],
            "dining table" : [1.438722 , -0.617209, 0.2],
        }

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
        self.goto_active = False

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_ang_z = math.atan2(siny_cosp, cosy_cosp)
        # print(f"[Imu] Angle {self.current_ang_z}")

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # print(f"[Odometry] Position: x={self.current_x:.2f}, y={self.current_y:.2f}, z={self.current_z:.2f}")

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
        self.publish_success_status(task="Sit")
        msg = Bool()
        msg.data = True
        # self.go2_goto_status_pub.publish(msg)

    def stand(self):
        self._task_done = False
        self.target_z = 0.0
        self.mode = MotionMode.STAND
        while not self._task_done:
            self.smooth_move()
            rclpy.spin_once(self, timeout_sec=self.timer_period)
        self.publish_success_status(task="Stand")
        msg = Bool()
        msg.data = True
        # self.go2_goto_status_pub.publish(msg)

    def smooth_move(self):
        if abs(self.target_z - self.current_z) < self.step:
            self.current_z = self.target_z
            self.publish_pose()
            self._task_done = True
            return
        direction = 1.0 if self.target_z > self.current_z else -1.0
        self.current_z += direction * self.step * 0.9
        self.publish_pose()
        
    def move_forward(self):
        self._task_done = False
        self.publish_vel(x=0.3)
        start = time.time()
        while time.time() - start < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.publish_vel(0.0, 0.0, 0.0)
        self._task_done = True
        self.publish_success_status("Move Forward")
        msg = Bool()
        msg.data = True
        # self.go2_goto_status_pub.publish(msg)

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
        self.publish_success_status(task="Move Backward")
        msg = Bool()
        msg.data = True
        # self.go2_goto_status_pub.publish(msg)

    def stop(self):
        self.publish_vel(0.0, 0.0, 0.0)
        self.publish_success_status(task="STOP")
        msg = Bool()
        msg.data = True
        # self.go2_goto_status_pub.publish(msg)

    def go_to_position(self, x: float, y: float, z=0.0):
        self._task_done = False
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.mode = MotionMode.GOTO
        self.goto_active = True
        self.goto_timer = self.create_timer(
            0.1,
            lambda: self.goto_helper(f"{self.target_x , self.target_y}"),
            callback_group=self.goto_timer_group
        )
        self.goto_future = rclpy.task.Future()
        rclpy.spin_until_future_complete(self, self.goto_future)
        self.get_logger().info("GOTO COMPLETE")
        msg = Bool()
        msg.data = True
        # self.go2_goto_status_pub.publish(msg)

    def go_to_object(self,object=""):
        self._task_done = False
        object = object.lower()
        self.pose = self.object_poses.get(object,[None,None,None])
        self.target_x = self.pose[0]
        self.target_y = self.pose[1]
        self.target_z = self.pose[2]
        self.mode = MotionMode.GOTO
        if self.target_x == None and self.target_y == None :
            self.get_logger().error("goto object No valid Target")
            return
        print(f"Moving to position {self.target_x} , {self.target_y}")
        self.goto_active = True
        self.goto_timer = self.create_timer(
            0.1,
            lambda: self.goto_helper(f"{self.target_x , self.target_y}"),
            callback_group=self.goto_timer_group
        )
        self.goto_future = rclpy.task.Future()
        rclpy.spin_until_future_complete(self, self.goto_future)
        self.get_logger().info(" GOTO OBJECT COMPLETE ")
        msg = Bool()
        msg.data = True
        # self.go2_goto_status_pub.publish(msg)


    def goto_helper(self,target: str):
        self.angle = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        print(f"Diff {self.angle - self.current_ang_z}")
        if abs(self.angle - self.current_ang_z) > 0.1: 
            direction = 1.0 if self.angle > self.current_ang_z else -1.0
            self.publish_vel(0.0,0.0,direction)
        else:
            print(f"Length Diff {abs(self.current_x - self.target_x)}")
            if abs(self.current_x - self.target_x) < 0.28:  
                print("Goal Completely Reached")
                self.publish_vel(0.0,0.0,0.0)
                self.publish_success_status(task=f"Go To {target}")
                if self.goto_active:
                    print("DESTROY !!!!!!!!!!!!!!")
                    self.goto_active = False
                    self.goto_timer.cancel()
                    self.goto_timer = None
                    if self.goto_future is not None and not self.goto_future.done():
                        self.goto_future.set_result(True)
            else:
                self.publish_vel(1.0,0.0,0.0)


    def publish_success_status(self, task=""):
        msg = String()
        msg.data = f"Quadruped (status) : {task} : COMPLETE"
        self.status_pub.publish(msg)

    # def success(self):
    #     with open("/home/vipul/ws/himura_ws/src/quadruped_sim/quadruped_sim/data.txt", "a") as f:
    #         f.write("Quadruped (status) : TASKS COMPLETE\n")  # Added newline for clarity
    #         f.flush()

        
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
    # node.go_to_position(7.788722,-8.51,0.0)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
