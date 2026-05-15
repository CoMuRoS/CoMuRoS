import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node , PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

def is_valid_to_launch():
    # Path includes model name of Raspberry Pi series
    path = '/sys/firmware/devicetree/base/model'
    if os.path.exists(path):
        return False
    else:
        return True

def generate_launch_description():


    # ================================================== GO2 =============================================================

    # Paths
    config_pkg_share = launch_ros.substitutions.FindPackageShare("go2_config").find("go2_config")
    descr_pkg_share = launch_ros.substitutions.FindPackageShare("go2_description").find("go2_description")
    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    ros_control_config = os.path.join(config_pkg_share, "config/ros_control/ros_control.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    default_model_path = os.path.join(descr_pkg_share, "xacro/robot.xacro")
    default_world_path = os.path.join(config_pkg_share, "worlds/default.world")

    ld = LaunchDescription()

    # # Declare launch arguments
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true"))
    ld.add_action(DeclareLaunchArgument("rviz", default_value="false", description="Launch rviz"))
    ld.add_action(DeclareLaunchArgument("robot_name", default_value="champ", description="Robot name"))
    # ld.add_action(DeclareLaunchArgument("lite", default_value="false", description="Lite"))
    # ld.add_action(DeclareLaunchArgument("gui", default_value="true", description="Use GUI"))
    # ld.add_action(DeclareLaunchArgument("world_init_x", default_value="0.0"))
    # ld.add_action(DeclareLaunchArgument("world_init_y", default_value="0.0"))
    # ld.add_action(DeclareLaunchArgument("world_init_z", default_value="0.275"))
    # ld.add_action(DeclareLaunchArgument("world_init_heading", default_value="0.0"))
    # ld.add_action(DeclareLaunchArgument("ros_control_file", default_value=ros_control_config, description="Ros control config path"))
    # ld.add_action(DeclareLaunchArgument("world", default_value=default_world_path, description="Gazebo world name"))
    ld.add_action(DeclareLaunchArgument('start_rviz',  default_value='false', description='Whether execute rviz2'))
    ld.add_action(DeclareLaunchArgument('prefix', default_value='""', description='Prefix of the joint and link names'))
    ld.add_action(DeclareLaunchArgument('use_sim',default_value='true',description='Start robot in Gazebo simulation.'))
    # 

    # bringup_ld = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory("champ_bringup"), "launch", "bringup.launch.py")
    #     ),
    #     launch_arguments={
    #         "description_path": default_model_path,
    #         "joints_map_path": joints_config,
    #         "links_map_path": links_config,
    #         "gait_config_path": gait_config,
    #         "use_sim_time": LaunchConfiguration("use_sim_time"),
    #         "robot_name": LaunchConfiguration("robot_name"),
    #         "gazebo": "true",
    #         "lite": LaunchConfiguration("lite"),
    #         "rviz": LaunchConfiguration("rviz"),
    #         "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
    #         "hardware_connected": "false",
    #         "publish_foot_contacts": "false",
    #         "close_loop_odom": "true",
    #     }.items(),
    # )

    # Include Gazebo launch
    go2_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("go2_config"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            # "robot_name": LaunchConfiguration("go2"),
            # "world": LaunchConfiguration("world"),
            # "lite": LaunchConfiguration("lite"),
            # "world_init_x": LaunchConfiguration("world_init_x"),
            # "world_init_y": LaunchConfiguration("world_init_y"),
            # "world_init_z": LaunchConfiguration("world_init_z"),
            # "world_init_heading": LaunchConfiguration("world_init_heading"),
            # "gui": LaunchConfiguration("gui"),
            # "close_loop_odom": "true",
        }.items(),
    )
    # ================================================= Waffle ==============================================

    if not is_valid_to_launch():
        print('Can not launch fake robot in Raspberry Pi')
        return LaunchDescription([])

    # Define launch file path
    base_launch_file = PathJoinSubstitution([
        FindPackageShare("turtlebot3_manipulation_bringup"),
        "launch",
        "base.launch.py"
    ]
    # ,
    # namespace="waffle"
    )


    # Include the launch file
    base_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file),
        launch_arguments={
            'start_rviz': LaunchConfiguration('start_rviz'),
            'prefix': LaunchConfiguration('prefix'),
            'use_sim': LaunchConfiguration('use_sim'),
        }.items(),
        # namespace="waffle",
    )

    moveit_config_path = get_package_share_directory("turtlebot3_manipulation_moveit_config")

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_path, "launch", "moveit_core.launch.py")
        )
    )

    # Gazebo Spawner Node
    waffle_gazebo_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", "waffle",
            "-topic", "/waffle/robot_description",
            "-robot_namespace", "waffle",  # <-- Set namespace
            "-x", '-2.00',
            "-y", '-0.50',
            "-z", '0.01',
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0",
        ],
        namespace="waffle",

    )

    waffle_group = GroupAction([
        PushRosNamespace("waffle"),
        base_launch_include,    
        waffle_gazebo_spawner_cmd,
    ])




    # ============================================ Burger =========================================

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    # Define LaunchConfigurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock')
    declare_namespace = DeclareLaunchArgument('namespace', default_value='burger', description='Namespace for the robot')
    declare_x_pose = DeclareLaunchArgument('x_pose', default_value='0.0', description='X position')
    declare_y_pose = DeclareLaunchArgument('y_pose', default_value='0.0', description='Y position')
    declare_z_pose = DeclareLaunchArgument('z_pose', default_value='0.0', description='Z position')
    declare_roll = DeclareLaunchArgument('roll', default_value='0.0', description='Roll angle')
    declare_pitch = DeclareLaunchArgument('pitch', default_value='0.0', description='Pitch angle')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw angle')


    namespace = LaunchConfiguration('namespace', default='burger')  # Already defined

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,  # Use the LaunchConfiguration variable
            'use_sim_time': use_sim_time
        }.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,  # Use the LaunchConfiguration variable
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )
    
    burger_multi_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'multi_robot.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Add declared arguments
    # ld.add_action(declare_use_sim_time)
    # ld.add_action(declare_namespace)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_z_pose)
    ld.add_action(declare_roll)
    ld.add_action(declare_pitch)
    ld.add_action(declare_yaw)

# =============================================================================================================

    # ld = LaunchDescription()

    # ld.add_action(bringup_ld)
    # ld.add_action(gazebo_ld)
    # ld.add_action(go2_ld)
    ld.add_action(waffle_group)
    # ld.add_action(base_launch_include)
    # ld.add_action(waffle_gazebo_spawner_cmd)
    # ld.add_action(moveit_launch)
    # ld.add_action(robot_state_publisher_cmd)
    # ld.add_action(spawn_turtlebot_cmd)
    # ld.add_action(burger_multi_robot)
    
    return ld