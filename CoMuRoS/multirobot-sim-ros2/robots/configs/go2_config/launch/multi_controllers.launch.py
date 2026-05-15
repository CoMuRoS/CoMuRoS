#!/usr/bin/env python3
"""
Multi-Robot Launch File

This launch file:
  - Launches the Go2 simulation via an ExecuteProcess.
  - When the Go2 process exits, an event handler starts the Waffle group.
  - The Waffle group includes a base launch (from turtlebot3_manipulation_bringup)
    and a Gazebo spawner node.
  
Make sure that your Go2 and Waffle controller configuration files (and URDF/xacro files)
are set up in their respective packages.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PythonExpression
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    RegisterEventHandler,
    LogInfo,
    ExecuteProcess,TimerAction,    IncludeLaunchDescription,

)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare 
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

def generate_launch_description():
    ld = LaunchDescription()

    # Declare common launch arguments
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true"))
    ld.add_action(DeclareLaunchArgument("rviz", default_value="false", description="Launch rviz"))
    ld.add_action(DeclareLaunchArgument("robot_name", default_value="champ", description="Robot name"))
    ld.add_action(DeclareLaunchArgument("start_rviz", default_value="false", description="Whether to execute rviz2"))
    ld.add_action(DeclareLaunchArgument("prefix", default_value="\"\"", description="Prefix of the joint and link names"))
    ld.add_action(DeclareLaunchArgument("use_sim", default_value="true", description="Start robot in Gazebo simulation"))

    # if not is_valid_to_launch():
    #     print('Cannot launch fake robot on Raspberry Pi')
    #     return LaunchDescription([])

    # Get package share directories
    config_pkg_share = get_package_share_directory("go2_config")
    descr_pkg_share = get_package_share_directory("go2_description")
    default_world_path = os.path.join(config_pkg_share, "worlds", "default.world")

    # --- GO2 Launch via ExecuteProcess ---
    # Launch Go2 from go2_config's gazebo.launch.py using ExecuteProcess,
    # so that we can use an exit handler.
    go2_cmd = [
        "ros2", "launch", "go2_config", "gazebo.launch.py"
        # "--ros-args", "-p", "use_sim_time:=", LaunchConfiguration("use_sim_time")
    ]
    go2_ld = ExecuteProcess(
        cmd=go2_cmd,
        output="screen"
    )

    # --- Waffle Launch ---
    # Base launch for waffle (from turtlebot3_manipulation_bringup)
    base_launch_file = PathJoinSubstitution([
        FindPackageShare("turtlebot3_manipulation_bringup"), "launch", "base.launch.py"
    ])
    base_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file),
        launch_arguments={
            'start_rviz': LaunchConfiguration('start_rviz'),
            'prefix': LaunchConfiguration('prefix'),
            'use_sim': LaunchConfiguration('use_sim'),
        }.items(),
        # namespace="waffle",
    )
    # Waffle spawn node
    waffle_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", "waffle",
            "-topic", "/waffle/robot_description",
            "-robot_namespace", "waffle",
            "-x", "3.00",
            "-y", "-2.00",
            "-z", "0.1",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.75",
        ],
        namespace="waffle",

    )
    # waffle_group = GroupAction([
    #     PushRosNamespace("waffle"),
    #     base_launch_include,
    #     waffle_spawner,
    # ],scoped=True)



    # waffle_timer = TimerAction(
    #     period=10.0,
    #     actions=[waffle_group]
    # )

    # =======================================================================

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    burger_multi_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'empty_world.launch.py')
        ),
        launch_arguments={
            # 'namespace': "burger_cam_1",
            'use_sim': LaunchConfiguration('use_sim'),
        }.items()
    )

    # burger_group = GroupAction([ 
    #     PushRosNamespace("burger_cam_1"),
    #     burger_multi_robot
    #     ],
    #     scoped=True)

    # burger_timer = TimerAction(
    #     period=15.0,
    #     actions=[burger_group]
    # )

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    # Use the turtlebot3_gazebo package if that's where lab.world is located.
    turtle_pkg_share = FindPackageShare(package="turtlebot3_simulations").find("turtlebot3_gazebo")
    # Alternatively, if lab.world is really in champ_gazebo, use gz_pkg_share instead.
    gz_pkg_share = FindPackageShare(package="champ_gazebo").find("champ_gazebo")

    # Declare the 'world' argument with lab.world as the default value.
    declare_gazebo_world = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(gz_pkg_share, "worlds", "disaster.world"),
        description="World file to load"
    )
    declare_headless = DeclareLaunchArgument("headless", default_value="False")

    ld.add_action(declare_gazebo_world)
    ld.add_action(declare_headless)

    # Use LaunchConfiguration substitutions so that the arguments are resolved at runtime.
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")

    gazebo_config = os.path.join(
        FindPackageShare(package="champ_gazebo").find("champ_gazebo"),
        "config", "gazebo.yaml"
    )
    pkg_share = FindPackageShare(package="champ_description").find("champ_description")
    launch_dir = os.path.join(pkg_share, "launch")

    # Debug: log the resolved world file path (it will be resolved at runtime)
    log_world = LogInfo(msg=["Using world file: ", world])

    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[
            "gzserver",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
            world,                            
            '--ros-args',
            '--params-file', gazebo_config
        ],
        cwd=[launch_dir],
        output="screen",
    )


    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([" not ", headless])),
        cmd=["gzclient"],
        cwd=[launch_dir],
        output="screen",
    )

    # Declare arguments to control launching Waffle and Burger
    ld.add_action(DeclareLaunchArgument("launch_burger", default_value="true", description="Launch Burger Robot?"))
    ld.add_action(DeclareLaunchArgument("launch_waffle", default_value="true", description="Launch Waffle Robot?"))
    ld.add_action(DeclareLaunchArgument("waffle_namespace", default_value="waffle", description="Namespace for Waffle"))
    ld.add_action(DeclareLaunchArgument("burger_namespace", default_value="burger_cam_1", description="Namespace for Burger"))

    # Get launch configurations
    launch_waffle = LaunchConfiguration("launch_waffle")
    launch_burger = LaunchConfiguration("launch_burger")
    waffle_namespace = LaunchConfiguration("waffle_namespace")
    burger_namespace = LaunchConfiguration("burger_namespace")

    # Waffle group with conditional namespace
    waffle_group = GroupAction([
        PushRosNamespace(waffle_namespace),  # Apply namespace dynamically
        base_launch_include,
        waffle_spawner,
    ], scoped=True)

    waffle_timer = TimerAction(
        period=10.0,
        actions=[waffle_group],
        condition=IfCondition(launch_waffle)  # Launch only if enabled
    )
    
    # Burger group with conditional namespace
    burger_group = GroupAction([
        PushRosNamespace(burger_namespace),  # Apply namespace dynamically
        burger_multi_robot,
    ], scoped=True)

    burger_timer = TimerAction(
        period=15.0,
        actions=[burger_group],
        condition=IfCondition(launch_burger)  # Launch only if enabled
    )

    go2_timer = TimerAction(
        period=1.0,
        actions=[go2_ld]
    )
 

    # Add actions to LaunchDescription
    ld.add_action(go2_timer)
    ld.add_action(waffle_timer)
    ld.add_action(burger_timer)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(gzclient_cmd)
    # ld.add_action(start_gazebo_client_cmd)

    
    return ld

# if __name__ == '__main__':
#     generate_launch_description()
