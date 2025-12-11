#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, NotSubstitution


def generate_launch_description():
    # === Package paths ===
    desc_pkg = FindPackageShare('x3_uav_description').find('x3_uav_description')
    ign_pkg  = FindPackageShare('x3_uav_ignition').find('x3_uav_ignition')

    use_ignition = LaunchConfiguration('use_ignition')
    use_ignition_arg = DeclareLaunchArgument('use_ignition', default_value='True')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True')

    use_multi_rviz = LaunchConfiguration('use_multi_rviz')
    use_multi_rviz_arg = DeclareLaunchArgument('use_multi_rviz', default_value='false')

    use_single_rviz = LaunchConfiguration('use_single_rviz')
    use_single_rviz_arg = DeclareLaunchArgument('use_single_rviz', default_value='false')

    headless = LaunchConfiguration('headless')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')

    world_file = LaunchConfiguration('world_file')
    default_world_file = PathJoinSubstitution([
        ign_pkg,
        'worlds',
        'empty_world.world'
    ])
    declare_world_file = DeclareLaunchArgument(
        name='world_file',
        default_value=default_world_file,
        description='World file name (e.g., empty.world, house.world, pick_and_place_demo.world)'
    )
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')

    start_gz_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [ '-r -s -v2 ', world_file ],
            'on_exit_shutdown': 'true'
        }.items(),
        condition=IfCondition(use_ignition)
    )

    start_gz_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v2'
        }.items(),
        condition=IfCondition(
            AndSubstitution(
                NotSubstitution(headless),
                use_ignition
            )
        )
    )

    ignition_no_namespace_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_tf_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_ignition),
    )



    x3_uav_robot = os.path.join(ign_pkg, 'launch', 'x3_uav_robot.launch.py')

    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(x3_uav_robot),
        launch_arguments={
            'use_jsp': 'false',
            'use_jsp_gui': 'false',
            'robot_name': 'x3_uav',
            'prefix': 'x3_1',
            'use_sim_time': use_sim_time,
            # 'xacro_file': xacro_file,
            'use_ignition': use_ignition,
            'use_plugin': 'True',
            'use_ros2_control': 'false',
            'use_mock_hardware': 'false',
            'spawn_x': '0.0',
            'spawn_y': '1.0',
            'spawn_z': '0.05',
            'spawn_roll': '0.0',
            'spawn_pitch': '0.0',
            'spawn_yaw': '0.0',
            'use_rviz': use_multi_rviz,
        }.items()
    )


    robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(x3_uav_robot),
        launch_arguments={
            'use_jsp': 'false',
            'use_jsp_gui': 'false',
            'robot_name': 'x3_uav',
            'prefix': 'x3_2',
            'use_sim_time': use_sim_time,
            # 'xacro_file': xacro_file,
            'use_ignition': use_ignition,
            'use_plugin': 'True',
            'use_ros2_control': 'false',
            'use_mock_hardware': 'false',
            'spawn_x': '0.0',
            'spawn_y': '-1.0',
            'spawn_z': '0.05',
            'spawn_roll': '0.0',
            'spawn_pitch': '0.0',
            'spawn_yaw': '0.0',
            'use_rviz': use_multi_rviz,
        }.items()
    )

    single_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='single_rviz',
        arguments=['-d', os.path.join(desc_pkg, 'rviz', 'multi_robot.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_single_rviz),
        output='screen'
    )

    ld = LaunchDescription()


    ld.add_action(use_sim_time_arg)

    ld.add_action(use_ignition_arg)

    ld.add_action(use_multi_rviz_arg)
    ld.add_action(use_single_rviz_arg)
    ld.add_action(headless_arg)
    ld.add_action(declare_world_file)

    ld.add_action(start_gz_server_cmd)
    ld.add_action(start_gz_client_cmd)
    ld.add_action(ignition_no_namespace_bridge)

    ld.add_action(robot1)
    ld.add_action(robot2)

    ld.add_action(single_rviz_node)

    return ld
