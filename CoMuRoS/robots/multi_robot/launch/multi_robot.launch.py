#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    TimerAction
)
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, NotSubstitution


def generate_launch_description():

    # Packages
    pkg_shared            = get_package_share_directory('multi_robot')
    # pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_yahboom_gazebo    = get_package_share_directory('yahboom_rosmaster_gazebo')
    pkg_x3_uav_gazebo     = get_package_share_directory('x3_uav_ignition')
    pkg_ros_gz_sim        = get_package_share_directory('ros_gz_sim')

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )
    declare_use_ignition = DeclareLaunchArgument(
        'use_ignition', default_value='true', description='Enable Ignition Gazebo plugins'
    )
    decalre_use_headless_arg = DeclareLaunchArgument(
        'headless', default_value='false', description='use igntion headless'
    )
    declare_single_rviz_cmd = DeclareLaunchArgument(
        'use_single_rviz', default_value='false', description='use one rviz for all the robot'
    )
    decalre_multi_rviz_cmd = DeclareLaunchArgument(
        'use_multi_rviz', default_value='false', description='use one rviz for each robot'
    )

    # Configurations
    use_sim_time     = LaunchConfiguration('use_sim_time')
    use_ignition     = LaunchConfiguration('use_ignition')
    headless         = LaunchConfiguration('headless')
    use_single_rviz  = LaunchConfiguration('use_single_rviz')
    use_multi_rviz   = LaunchConfiguration('use_multi_rviz')

    # Gazebo
    world_file = os.path.join(
        pkg_shared,
        'worlds',
        # 'empty_world.world'
        'food_court.sdf'
    )

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

    igntion_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_tf_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Robot launch file paths
    yahboom_launch = PathJoinSubstitution([
        pkg_yahboom_gazebo, 'launch', 'yahboom_robot.launch.py'
    ])

    # tb3_launch = PathJoinSubstitution([
    #     pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_robot.launch.py'
    # ])

    x3_uav_robot = PathJoinSubstitution([
        pkg_x3_uav_gazebo, 'launch', 'x3_uav_robot.launch.py'
    ])


    robot3_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(x3_uav_robot),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'robot_name': 'x3_uav',
                    'prefix': 'r3',
                    'use_ignition': use_ignition,
                    'use_plugin': 'True',
                    'use_ros2_control': 'false',
                    'use_mock_hardware': 'false',
                    'spawn_x': '-1.0',
                    'spawn_y': '1.0',
                    'spawn_z': '0.05',
                    'spawn_roll': '0.0',
                    'spawn_pitch': '0.0',
                    'spawn_yaw': '0.0',
                    'use_rviz': use_multi_rviz,
                }.items()
            )
        ]
    )

    robot2_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(yahboom_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'robot_name': 'rosmaster_x3',
                    'prefix': 'r2',
                    'use_ignition': use_ignition,
                    'use_plugin': 'True',
                    'use_ros2_control': 'False',
                    'use_mock_hardware': 'False',
                    'spawn_x': '0.0',
                    'spawn_y': '0.0',
                    'spawn_z': '0.05',
                    'spawn_roll': '0.0',
                    'spawn_pitch': '0.0',
                    'spawn_yaw': '0.0',
                    'use_rviz': use_multi_rviz,
                }.items()
            )
        ]
    )

    # robot2_launch = TimerAction(
    #     period=15.0,
    #     actions=[
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(tb3_launch),
    #             launch_arguments={
    #                 'use_sim_time': use_sim_time,
    #                 'robot_name': 'turtlebot3_burger_cam',
    #                 'prefix': 'r2',
    #                 'use_ignition': use_ignition,
    #                 'use_plugin': 'true',
    #                 'use_ros2_control': 'false',
    #                 'spawn_x': '0.0',
    #                 'spawn_y': '0.0',
    #                 'spawn_z': '0.05',
    #                 'spawn_roll': '0.0',
    #                 'spawn_pitch': '0.0',
    #                 'spawn_yaw': '0.0',
    #                 'use_rviz': use_multi_rviz,
    #             }.items()
    #         )
    #     ]
    # )




    robot1_launch = TimerAction(
        period=25.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(yahboom_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'robot_name': 'rosmaster_x3',
                    'prefix': 'r1',
                    'use_ignition': use_ignition,
                    'use_plugin': 'True',
                    'use_ros2_control': 'False',
                    'use_mock_hardware': 'False',
                    'spawn_x': '11.0',
                    'spawn_y': '0.0',
                    'spawn_z': '0.05',
                    'spawn_roll': '0.0',
                    'spawn_pitch': '0.0',
                    'spawn_yaw': '0.0',
                    'use_rviz': use_multi_rviz,
                }.items()
            )
        ]
    )

    single_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='single_rviz',
        arguments=['-d', os.path.join(pkg_shared, 'rviz', 'multi_robot.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_single_rviz),
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_ignition,
        decalre_use_headless_arg,
        decalre_multi_rviz_cmd,
        declare_single_rviz_cmd,

        start_gz_server_cmd,
        start_gz_client_cmd,

        igntion_bridge_cmd,

        robot3_launch,
        robot2_launch,
        robot1_launch,

        single_rviz_node
    ])