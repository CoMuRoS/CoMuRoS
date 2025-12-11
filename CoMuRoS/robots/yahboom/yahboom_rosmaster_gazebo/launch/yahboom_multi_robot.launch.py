#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, TimerAction, SetLaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, NotSubstitution
from launch.actions import OpaqueFunction

def generate_launch_description():

    # Packages
    pkg_shared            = get_package_share_directory('yahboom_rosmaster_gazebo')
    # pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_yahboom_gazebo    = get_package_share_directory('yahboom_rosmaster_gazebo')
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
        'use_single_rviz', default_value='true', description='use one rviz for all the robot'
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

    world_file = LaunchConfiguration('world_file')
    default_world_file = PathJoinSubstitution([
        pkg_yahboom_gazebo,
        'worlds',
        'empty_world.world'
    ])
    declare_world_file = DeclareLaunchArgument(
        name='world_file',
        default_value=default_world_file,
        description='World file name (e.g., empty.world, house.world, pick_and_place_demo.world)'
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
        condition=IfCondition(use_ignition),
    )

    yahboom_launch = PathJoinSubstitution([
        os.path.join(pkg_yahboom_gazebo, 'launch', 'yahboom_robot.launch.py')

    ])

    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(yahboom_launch),
        launch_arguments={
            'robot_name': 'rosmaster_x3',
            'prefix': 'r1',
            'use_sim_time': use_sim_time,
            'use_jsp': 'False',
            'use_jsp_gui': 'False',
            'use_ignition': use_ignition,
            'use_plugin': 'True',
            'use_ros2_control': 'False',
            'use_mock_hardware': 'False',
            'use_rviz': use_multi_rviz,
            'enable_odom_tf': 'false',
            'spawn_x': '0.0',
            'spawn_y': '-1.0',
            'spawn_z': '0.05',
            'spawn_roll': '0.0',
            'spawn_pitch': '0.0',
            'spawn_yaw': '0.0'
        }.items()
    )

    robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(yahboom_launch),
        launch_arguments={
            'robot_name': 'rosmaster_x3',
            'prefix': 'r2',
            'use_sim_time': use_sim_time,
            'jsp_gui': 'False',
            'use_ignition': use_ignition,
            'use_plugin': 'True',
            'use_ros2_control': 'False',
            'use_mock_hardware': 'False',
            'use_rviz': use_multi_rviz,
            'enable_odom_tf': 'false',
            'spawn_x': '0.0',
            'spawn_y': '1.0',
            'spawn_z': '0.05',
            'spawn_roll': '0.0',
            'spawn_pitch': '0.0',
            'spawn_yaw': '0.0'
        }.items()
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
        declare_world_file,

        start_gz_server_cmd,
        start_gz_client_cmd,

        igntion_bridge_cmd,

        robot1,
        robot2,

        single_rviz_node

    ])
