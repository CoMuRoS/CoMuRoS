#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from threading import Thread

from pymoveit2 import MoveIt2
# from pymoveit2.robots import ur5 as robot
from pymoveit2.robots import panda as robot
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import Plane

class UR5Commander(Node):

    def __init__(self):
        super().__init__("ur5_commander")

        # Create callback group that allows execution of callbacks in parallel without restrictions
        self.callback_group = ReentrantCallbackGroup()

        # Get parameters    
        self.pick_position = [ [0.0 , 0.38 , -0.08], #arm1 Pick
                               [0.4, 0.5, 0.2] ]     #arm2 Pick
        
        self.pick_quat_xyzw = [ [1.0, 0.0, 0.0, 0.0], #arm1 Pick Quat
                                [1.0, 0.0, 0.0, 0.0]] #arm2 Pick Quat
        
        self.place_position = [ [0.6, 0.3, -0.35], #arm1 Place pose
                                [0.5, -0.4, 0.2]]  #arm2 Place pose
        
        self.place_quat_xyzw = [ [1.0, 0.0, 0.0, 0.0], #arm1 Place Quat
                                [1.0, 0.0, 0.0, 0.0]] #arm2 Place Quat
        self.cartesian = True


    def pick_routine(self,robot_name):

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )
        if robot_name == "arm1" : idx = 0
        elif robot_name == "arm2" : idx = 1

        try:
            # Move to pose
            self.get_logger().info(
                f"Moving to {{position: {list(self.pick_position[idx])}, quat_xyzw: {list(self.pick_quat_xyzw[idx])}}}"
            )
            self.add_ground_plane()
            self.moveit2.move_to_pose(position=self.pick_position[idx], quat_xyzw=self.pick_quat_xyzw[idx], cartesian=self.cartesian)
            self.moveit2.wait_until_executed()
        except Exception as err:
            self.get_logger().info(f'Exception occured. {err}')

        self.get_logger().info(f'Movement completed')

    def place_routine(self,robot_name):

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(prefix=robot_name),
            end_effector_name=robot.end_effector_name(prefix=robot_name),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )
        if robot_name == "arm1" : idx = 0
        elif robot_name == "arm2" : idx = 1

        try:
            # Move to pose
            self.get_logger().info(
                f"Moving to {{position: {list(self.place_position[idx])}, quat_xyzw: {list(self.place_quat_xyzw[idx])}}}"
            )
            self.add_ground_plane()
            self.moveit2.move_to_pose(position=self.place_position[idx], quat_xyzw=self.place_quat_xyzw[idx], cartesian=self.cartesian)
            self.moveit2.wait_until_executed()
        except Exception as err:
            self.get_logger().info(f'Exception occured. {err}')

        self.get_logger().info(f'Movement completed')

    # def move_to_pos --> Implemented already in MOVEIT2


    def get_planning_scene(self):

        # Create a client for the GetPlanningScene service
        get_scene_service = self.create_client(GetPlanningScene, "get_planning_scene")

        # Wait for the service to be available
        while not get_scene_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for the get_planning_scene service...")

        # Create a request object
        request = GetPlanningScene.Request()

        # Set the desired scene components to be returned
        request.components.components = request.components.WORLD_OBJECT_NAMES
        # Call the GetPlanningScene service
        future = get_scene_service.call(request)

        if future is not None:
            if future.scene is not None:
                scene = future.scene
                for obj in scene.world.collision_objects:
                    print("Collision Object ID:", obj.id)
                    print("Collision Object Type:", obj.type)
                    print("Collision Object Plane:", obj.planes)
                    print("Collision Object Plane Pose:", obj.plane_poses)
                    print("-----------------------------")
            return future.scene

        return None

    def add_ground_plane(self):

        # Create a CollisionObject message
        collision_object = CollisionObject()
        collision_object.id = "ground_plane"
        collision_object.header.frame_id = "world"

        # Define the ground plane as a box shape
        ground_plane = Plane()
        ground_plane.coef = [0.0, 0.0, 1.0, 0.0]

        # Set the ground plane's pose
        ground_plane_pose = Pose()
        ground_plane_pose.position.z = -0.005  # Adjust the height of the ground plane

        collision_object.planes.append(ground_plane)
        collision_object.plane_poses.append(ground_plane_pose)

        # Create a PlanningScene message
        scene = PlanningScene()
        scene.world.collision_objects.append(collision_object)
        scene.is_diff = True
        
        publisher_ = self.create_publisher(PlanningScene, 'planning_scene', 10)
        publisher_.publish(scene)
        
def main():
    rclpy.init()
    # Create node for this example
    node = UR5Commander()
    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    node.pick_routine("arm1")

    try:
        executor_thread.join()
    except KeyboardInterrupt:
        print("Shutting down")
        rclpy.shutdown()

if __name__ == "__main__":
    main()