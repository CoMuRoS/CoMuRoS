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


def process_controller(context, *args, **kwargs):
    """
    Resolve LaunchConfigurations here (so we can use them in os.path.join and string ops).
    Creates a new controller config file from the template by replacing placeholders.
    """
    desc_pkg = kwargs['desc_pkg']
    robot_name = LaunchConfiguration('robot_name').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    enable_odom_tf = LaunchConfiguration('enable_odom_tf').perform(context)

    # Clean the prefix (remove trailing slash if present)
    name = prefix.rstrip('/') if prefix else ''

    # Build file paths
    template_path = os.path.join(desc_pkg, 'config', robot_name, 'ros2_controllers_template.yaml')

    if name:
        config_filename = f'ros2_controllers_{name}.yaml'
    else:
        config_filename = 'ros2_controllers.yaml'
    config_path = os.path.join(desc_pkg, 'config', robot_name, config_filename)

    # Read template and replace placeholders
    with open(template_path, 'r', encoding='utf-8') as f:
        content = f.read()

    content = content.replace('${prefix}', prefix)
    content = content.replace('enable_odom_tf: true', f'enable_odom_tf: {enable_odom_tf}')

    # Ensure directory exists
    os.makedirs(os.path.dirname(config_path), exist_ok=True)

    # Write new config file
    with open(config_path, 'w', encoding='utf-8') as f:
        f.write(content)

    return [SetLaunchConfiguration('configure_controller', config_path)]


# ... (everything above unchanged)

def generate_launch_description():
    # === Package paths ===
    desc_pkg = FindPackageShare('yahboom_rosmaster_description').find('yahboom_rosmaster_description')
    bringup_pkg = FindPackageShare('yahboom_rosmaster_bringup').find('yahboom_rosmaster_bringup')

    # === Launch Arguments (your preferred style) ===
    jsp_gui = LaunchConfiguration('jsp_gui')
    jsp_gui_arg = DeclareLaunchArgument('jsp_gui', default_value='false')

    prefix = LaunchConfiguration('prefix')
    prefix_arg = DeclareLaunchArgument('prefix', default_value='')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True')

    robot_name = LaunchConfiguration('robot_name')
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='rosmaster_x3')

    xacro_file = LaunchConfiguration('xacro_file')
    xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        default_value=os.path.join(desc_pkg, 'urdf', 'robots', 'rosmaster_x3.urdf.xacro')
    )

    use_gazebo = LaunchConfiguration('use_gazebo')
    use_gazebo_arg = DeclareLaunchArgument('use_gazebo', default_value='True')

    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_ros2_control_arg = DeclareLaunchArgument('use_ros2_control', default_value='False')

    use_plugin = LaunchConfiguration('use_plugin')
    use_plugin_arg = DeclareLaunchArgument('use_plugin', default_value='True')

    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    use_mock_hardware_arg = DeclareLaunchArgument('use_mock_hardware', default_value='False')

    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='False')

    enable_odom_tf = LaunchConfiguration('enable_odom_tf')
    enable_odom_tf_arg = DeclareLaunchArgument(
        'enable_odom_tf',
        default_value='false',
        choices=['true', 'false'],
        description='Whether to enable odometry transform broadcasting via ROS 2 Control'
    )

    headless = LaunchConfiguration('headless')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')

    package_name_gazebo = 'yahboom_rosmaster_gazebo'
    pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    gazebo_worlds_path = 'worlds'
    default_world_file = 'empty.world'

    world_file = LaunchConfiguration('world_file')
    world_path = PathJoinSubstitution([
        pkg_share_gazebo,
        gazebo_worlds_path,
        world_file
    ])

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    # --- ADD: declare missing world/pose args so they show up in launch ---
    declare_world_cmd = DeclareLaunchArgument(
        name='world_file',
        default_value=default_world_file,
        description='World file name (e.g., empty.world, house.world, pick_and_place_demo.world)')
    
    declare_x_cmd = DeclareLaunchArgument('x', default_value='0.0', description='x component of initial position, meters')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='0.0', description='y component of initial position, meters')
    declare_z_cmd = DeclareLaunchArgument('z', default_value='0.05', description='z component of initial position, meters')
    declare_roll_cmd = DeclareLaunchArgument('roll', default_value='0.0', description='roll angle of initial orientation, radians')
    declare_pitch_cmd = DeclareLaunchArgument('pitch', default_value='0.0', description='pitch angle of initial orientation, radians')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='0.0', description='yaw angle of initial orientation, radians')
    # ---------------------------------------------------------------------

    # === Controller Manager + Load Controllers ===
    config_file = OpaqueFunction(
        function=process_controller, kwargs={'desc_pkg': desc_pkg}
    )

    # === robot_state_publisher ===
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'display.launch.py')
        ),
        launch_arguments={
            'jsp_gui': jsp_gui,
            'prefix': prefix,
            'use_sim_time': use_sim_time,
            'robot_name': robot_name,
            'xacro_file': xacro_file,
            'use_gazebo': use_gazebo,
            'use_plugin': use_plugin,
            'use_ros2_control': use_ros2_control,
            'use_mock_hardware': use_mock_hardware,
            'configure_controller': LaunchConfiguration('configure_controller'),
            'use_rviz': use_rviz
        }.items()
    )



    # load_controllers_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(bringup_pkg, 'launch', 'load_ros2_controllers.launch.py')
    #     ),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'prefix': prefix,
    #         'robot_name': robot_name
    #     }.items()
    # )

    gazebo_models_path = 'models'
    gazebo_models_path = os.path.join(pkg_share_gazebo, gazebo_models_path)

    # Set Gazebo model path
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gazebo_models_path)

    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')

    # --- ADD: define missing bridge config path ---
    default_ros_gz_bridge_config_file_path = os.path.join(
        pkg_share_gazebo, 'config', 'ros_gz_bridge.yaml'
    )
    # ---------------------------------------------

    # Start Gazebo (server)
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', [' -r -s -v 4 ', world_path])],
        # --- ADD: only when use_gazebo true ---
        condition=IfCondition(use_gazebo),
    )

    # Start Gazebo client (GUI) if not headless
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-g ']}.items(),
        condition=IfCondition(AndSubstitution(
            NotSubstitution(headless),   # headless == false
            use_gazebo                   # and use_gazebo == true
        )),
    )

    # Bridge ROS topics and Gazebo messages
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': default_ros_gz_bridge_config_file_path,
        }],
        output='screen',
        # --- ADD: gate by use_gazebo ---
        condition=IfCondition(use_gazebo),
    )

    # Image bridge
    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/cam_1/image'],
        remappings=[('/cam_1/image', '/cam_1/color/image_raw')],
        # --- ADD: gate by use_gazebo ---
        condition=IfCondition(use_gazebo),
    )

    # Spawn the robot
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ],
        # --- ADD: gate by use_gazebo ---
        condition=IfCondition(use_gazebo),
    )

    # === LaunchDescription ===
    ld = LaunchDescription()
    # arguments
    ld.add_action(jsp_gui_arg)
    ld.add_action(prefix_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(robot_name_arg)
    ld.add_action(xacro_file_arg)
    ld.add_action(use_gazebo_arg)
    ld.add_action(use_plugin_arg)
    ld.add_action(use_ros2_control_arg)
    ld.add_action(use_mock_hardware_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(enable_odom_tf_arg)
    ld.add_action(headless_arg)                # ADD
    ld.add_action(declare_world_cmd)           # ADD
    ld.add_action(declare_x_cmd)               # ADD
    ld.add_action(declare_y_cmd)               # ADD
    ld.add_action(declare_z_cmd)               # ADD
    ld.add_action(declare_roll_cmd)            # ADD
    ld.add_action(declare_pitch_cmd)           # ADD
    ld.add_action(declare_yaw_cmd)             # ADD

    # core
    ld.add_action(config_file)               # set LaunchConfiguration('configure_controller') first
    ld.add_action(robot_state_publisher_cmd) # now display.launch.py sees it
    # ld.add_action(load_controllers_cmd)


    # gazebo bits (added, conditioned)
    ld.add_action(set_env_vars_resources)      # ADD (always safe; just extends resource path)
    ld.add_action(start_gazebo_server_cmd)     # ADD
    ld.add_action(start_gazebo_client_cmd)     # ADD
    ld.add_action(start_gazebo_ros_bridge_cmd) # ADD
    ld.add_action(start_gazebo_ros_image_bridge_cmd)  # ADD
    ld.add_action(start_gazebo_ros_spawner_cmd)       # ADD

    return ld
