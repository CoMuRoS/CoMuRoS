#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

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

def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [0.5, 0.0, 0.25])
    node.declare_parameter("quat_xyzw", [1.0, 0.0, 0.0, 0.0])
    node.declare_parameter("cartesian", False)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM,
        callback_group=callback_group,
        allow_approximate=True,  # Optional: add this flag

    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    # Move to pose
    node.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )

    try:
        add_ground_plane(node)
        moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
        moveit2.wait_until_executed()
    except Exception as err:
        node.get_logger().info(f'Exception occured. {err}')

    node.get_logger().info(f'Movement completed')
    rclpy.shutdown()
    exit(0)

'''
def reset_collision_objects(node):
    # Create a PlanningScene message
    planning_scene = PlanningScene()

    # Set the operation to REMOVE or CLEAR to remove or clear all collision objects respectively
    planning_scene.world.collision_objects.clear()
    planning_scene.is_diff = True

    # Publish the PlanningScene to update the collision objects in the planning scene
    publisher = node.create_publisher(PlanningScene, "planning_scene", 10)
    publisher.publish(planning_scene)

    # Wait for the update to be applied
    apply_scene_service = node.create_client(ApplyPlanningScene, "apply_planning_scene")
    while not apply_scene_service.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for the /apply_planning_scene service...")
    request = ApplyPlanningScene.Request()
    #apply_scene_service.call(request)
'''

def get_planning_scene(node):

    # Create a client for the GetPlanningScene service
    get_scene_service = node.create_client(GetPlanningScene, "get_planning_scene")

    # Wait for the service to be available
    while not get_scene_service.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for the get_planning_scene service...")

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

def add_ground_plane(node):

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
    
    publisher_ = node.create_publisher(PlanningScene, 'planning_scene', 10)
    publisher_.publish(scene)

if __name__ == "__main__":
    main()
