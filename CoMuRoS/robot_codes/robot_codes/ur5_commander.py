#!/usr/bin/env python3
import sys
import argparse
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from threading import Thread
import time

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
from linkattacher_msgs.srv import AttachLink, DetachLink

class UR5Commander(Node):

    def __init__(self, ns=""):
        super().__init__("ur5_commander",namespace=ns)

        # Create callback group that allows execution of callbacks in parallel without restrictions
        self.callback_group = ReentrantCallbackGroup()
        self.attacher = self.create_client(AttachLink,"/ATTACH_LINK")
        self.detacher = self.create_client(DetachLink,"/DETACH_LINK")

        # Get parameters    
        self.pick_position = [ [0.0 , 0.38 , -0.08], #arm1 Pick
                               [0.4, 0.5, -0.52] ]     #arm2 Pick
        
        self.pick_quat_xyzw = [ [1.0, 0.0, 0.0, 0.0], #arm1 Pick Quat
                                [1.0, 0.0, 0.0, 0.0]] #arm2 Pick Quat
        
        self.place_position = [ [0.6, 0.3, -0.35], #arm1 Place pose
                                [0.5, -0.4, -0.05]]  #arm2 Place pose
        
        self.place_quat_xyzw = [ [1.0, 0.0, 0.0, 0.0], #arm1 Place Quat
                                [1.0, 0.0, 0.0, 0.0]] #arm2 Place Quat
        
        self.prepick_position = [ [0.6,0.3,0.1],
                                [0.4, 0.5, 0.2]]
        
        self.postpick_position = [ [0.6,0.3,0.1],
                                 [0.4, 0.5, 0.2]]
        
        self.start_position = [ [0.3,0.15,0.4],
                                [0.3,0.15,0.4]]        

        self.cartesian = True
        self.ns = ns.lstrip("/")  # Remove leading '/' if present
        print(self.ns)


    def pick_routine(self, object="Salad_on_a_Plate"):

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )
        if self.ns == "arm1" : idx = 0
        elif self.ns == "arm2" : idx = 1

        try:
            # if self.ns == "arm2":
            #     self.detach_link(model1="go2",link1="base_link",model2="Salad_on_a_Plate",link2="link_0")

            # Move to pose
            self.get_logger().info(
                f"Moving to {{position: {list(self.pick_position[idx])}, quat_xyzw: {list(self.pick_quat_xyzw[idx])}}}"
            )
            self.add_ground_plane()
            self.moveit2.move_to_pose(position=self.prepick_position[idx], quat_xyzw=self.pick_quat_xyzw[idx], cartesian=self.cartesian)
            self.moveit2.wait_until_executed()
            time.sleep(2.0)

            self.add_ground_plane()
            self.moveit2.move_to_pose(position=self.pick_position[idx], quat_xyzw=self.pick_quat_xyzw[idx], cartesian=self.cartesian)
            self.moveit2.wait_until_executed()
            self.attach_link(model1=self.ns,link1="wrist_3_link",model2=object,link2="link_0")
            time.sleep(2.0)

            self.add_ground_plane()
            self.moveit2.move_to_pose(position=self.postpick_position[idx], quat_xyzw=self.pick_quat_xyzw[idx], cartesian=self.cartesian)
            self.moveit2.wait_until_executed()
            time.sleep(2.0)

            # self.add_ground_plane()
            # self.moveit2.move_to_pose(position=self.place_position[idx], quat_xyzw=self.place_quat_xyzw[idx], cartesian=self.cartesian)
            # self.moveit2.wait_until_executed()
        except Exception as err:
            self.get_logger().info(f'Exception occured. {err}')

        self.get_logger().info(f'Movement completed')

    def place_routine(self, object="Salad_on_a_Plate"):
        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )
        if self.ns == "arm1" : idx = 0
        elif self.ns == "arm2" : idx = 1

        try:
            # Move to pose
            self.get_logger().info(
                f"Moving to {{position: {list(self.place_position[idx])}, quat_xyzw: {list(self.place_quat_xyzw[idx])}}}"
            )

            cart = True
            self.add_ground_plane()
            self.moveit2.move_to_pose(position=self.place_position[idx], quat_xyzw=self.place_quat_xyzw[idx], cartesian=cart)
            self.moveit2.wait_until_executed()
            self.detach_link(model1=self.ns,link1="wrist_3_link",model2=object,link2="link_0")
            time.sleep(2.0)

            # self.add_ground_plane()
            # self.moveit2.move_to_pose(position=self.postpick_position[idx], quat_xyzw=self.pick_quat_xyzw[idx], cartesian=self.cartesian)
            # self.moveit2.wait_until_executed()
            # time.sleep(2.0)

            self.add_ground_plane()
            self.moveit2.move_to_pose(position=self.start_position[idx], quat_xyzw=self.pick_quat_xyzw[idx], cartesian=cart)
            self.moveit2.wait_until_executed()
            # time.sleep(.0)

            # if self.ns == "arm1":
            #     self.attach_link(model1="go2",link1="base_link",model2="Salad_on_a_Plate",link2="link_0")


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

    def attach_link(self,model1="arm1",link1="wrist_3_link",model2="Salad_on_a_Plate",link2="link_0"):
        print(f'Sent Attach Request for {model2}')
        req = AttachLink.Request()
        req.model1_name = model1
        req.link1_name = link1
        req.model2_name = model2
        req.link2_name = link2

        future = self.attacher.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def detach_link(self,model1="arm1",link1="wrist_3_link",model2="Salad_on_a_Plate",link2="link_0"):
        print(f'Sent Detach Request for {model2}')
        req = DetachLink.Request()
        req.model1_name = model1
        req.link1_name = link1
        req.model2_name = model2
        req.link2_name = link2

        future = self.detacher.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

        
def main():
    rclpy.init()
    # Create node for this example
    parser = argparse.ArgumentParser()
    parser.add_argument('--ns', type=str, default="")
    args = parser.parse_args()

    node = UR5Commander(ns=args.ns)    # Spin the node in background thread(s)

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # node.pick_routine("arm1")
    # node.place_routine("arm1")

    try:
        executor_thread.join()
    except KeyboardInterrupt:
        print("Shutting down")
        rclpy.shutdown()

if __name__ == "__main__":
    main()