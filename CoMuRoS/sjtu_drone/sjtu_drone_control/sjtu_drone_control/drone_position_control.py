import rclpy
import time
from sjtu_drone_control.drone_utils.drone_object import DroneObject
from collections import deque
from geometry_msgs.msg import Pose
import time
from std_msgs.msg import String

class DronePositionControl(DroneObject):
    def __init__(self):
        super().__init__('drone_position_control')
        self.waypoints = [
            [5.0,  5.0,  8.0],
            [5.0, -5.0,  8.0],
            [3.0, -5.0,  8.0],
            [3.0,  5.0,  8.0],
            [1.0,  5.0,  8.0],
            [1.0, -5.0,  8.0],
            [-1.0, -5.0, 8.0],
            [-1.0,  5.0, 8.0],
            [-3.0,  5.0, 8.0],
            [-3.0, -5.0, 8.0],
            [-5.0, -5.0, 8.0],
            [-5.0,  5.0, 8.0],
            [-7.0,  5.0, 8.0],
            [-7.0, -5.0, 8.0],
            [-9.0, -5.0, 8.0],
            [-9.0,  5.0, 8.0],
            [-11.0, 5.0, 8.0], #17
            [-11.0, -5.0, 8.0],
            [-13.0, -5.0, 8.0],
            [-13.0, 5.0, 8.0], #20
            [-15.0, 5.0, 8.0], #21
            [-15.0, -5.0, 8.0], #22
            [-15.486129, 30.768650 , 8.0],
            # [-30.0 , 5.0 , 8.0]

            # [-32.110558, -1.929892 , 5.0], #23

        ]
        self.target_wp_idx = 0
        self.curr_pose = [0.0 , 0.0 , 0.0]
        self.sub_gt_pose = self.create_subscription(Pose, '/simple_drone/gt_pose', self.pose_callback, 10)
        self.pub = self.create_publisher(String , "/chat/input", 10)
        self.announced_wps = set()
        self.goto_timer = None
        self.takeOff()
        self.get_logger().info('Drone takeoff')

        # Set the m_posCtrl flag to True
        self.posCtrl(True)
        self.get_logger().info('Position control mode set to True')

        # self.move_drone_to_pose(self.waypoints[0][0], self.waypoints[0][1], self.waypoints[1][2])
        # self.move_drone_to_pose(0.0,0.0,5.0)
        # time.sleep(10.0)
        # time.sleep(10.0)
        self.move_drone_to_pose(5.0,5.0,8.0)
        time.sleep(10.0)
    
    def move_drone_to_pose(self, x, y, z):
        # Override the move_drone_to_pose method if specific behavior is needed
        super().moveTo(x, y, z)
        self.get_logger().info(f'Moving drone to pose: x={x}, y={y}, z={z}')

    def pose_callback(self, msg):
        # print(f"Drone Pose {msg.position}")
        self.curr_pose[0] = round(msg.position.x) 
        self.curr_pose[1] = round(msg.position.y) 
        self.curr_pose[2] = round(msg.position.z) 
    
    def start_surveillance(self):
        self.goto_timer = self.create_timer(2.0,self.goto_closest_target)

    def stop_surveiallance(self):
        if self.goto_timer :
            self.goto_timer.destroy()
    
    def stop(self):
        if self.goto_timer :
            self.goto_timer.destroy()

    def goto_closest_target(self):
        # Check for all test cases 

        # Exact Waypoint matching 
        for idx, wp in enumerate(self.waypoints):
            print(wp)
            # print(f"Drone Pose {self.curr_pose[0]} , {self.curr_pose[1]}")
            if round(wp[0]) == self.curr_pose[0] and round(wp[1]) == self.curr_pose[1]:
                if idx == 22 or idx == 23: 
                    self.target_wp_idx = 23
                else:
                    self.target_wp_idx = idx + 1 
                
                print(f"Current pose matches waypoint index: {self.target_wp_idx} → {wp}")
                msg = String()
                if self.target_wp_idx == 3 and 3 not in self.announced_wps:
                    msg.data = "Drone (msg) : Found a survivor at co-ordinates [3.829624 , -5.803646]"
                    self.pub.publish(msg)
                    self.announced_wps.add(3)

                elif self.target_wp_idx == 5 and 5 not in self.announced_wps:
                    msg.data = "Drone (msg) : Found a survivor at co-ordinates [0.307561 , 5.616884]"
                    self.pub.publish(msg)
                    self.announced_wps.add(5)

                # elif self.target_wp_idx == 14 and 14 not in self.announced_wps:
                #     msg.data = "Drone (msg) : Found a survivor at co-ordinates [-6.292633 , -7.923528]"
                #     self.pub.publish(msg)
                #     self.announced_wps.add(14)

                self.move_drone_to_pose(self.waypoints[self.target_wp_idx][0], self.waypoints[self.target_wp_idx][1] , self.waypoints[self.target_wp_idx][2])
                return

        # X coordinate matching 
        same_x = []
        for idx, wp in enumerate(self.waypoints):
            if round(wp[0]) == self.curr_pose[0] :
                same_x.append(wp)
                self.target_wp_idx = idx
        
        if len(same_x) == 2:
            print(f"Sending waypoint index X: {self.target_wp_idx}")
            self.move_drone_to_pose(self.waypoints[self.target_wp_idx][0], self.waypoints[self.target_wp_idx][1] , self.waypoints[self.target_wp_idx][2])
            return
        else:
            # Y coordinate matching 
            same_y = []

            # Step 1: Filter waypoints with same Y
            for wp in self.waypoints:
                if round(wp[1]) == round(self.curr_pose[1]):
                    same_y.append(wp)

            # Step 2: Sort by x value
            same_y_sorted = sorted(same_y, key=lambda wp: wp[0])

            # Step 3: Find next immediate smaller x
            curr_x = round(self.curr_pose[0])
            next_wp = None

            for wp in reversed(same_y_sorted):  # reverse to go from largest to smallest
                if wp[0] < curr_x:
                    next_wp = wp
                    break

            # Step 4: Set the target index
            if next_wp is not None:
                self.target_wp_idx = self.waypoints.index(next_wp)
                print(f"Next target waypoint: {next_wp}, index = {self.target_wp_idx}")
            else:
                print("No smaller x found on same Y row.")
                
            print(f"Sending waypoint index Y: {self.target_wp_idx}")
            self.move_drone_to_pose(self.waypoints[self.target_wp_idx][0], self.waypoints[self.target_wp_idx][1] , self.waypoints[self.target_wp_idx][2])
        

def main(args=None):
    rclpy.init(args=args)
    drone = DronePositionControl()
    # drone.start_surveillance()
    rclpy.spin(drone)
    drone.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()