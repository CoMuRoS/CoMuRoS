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

def bridge_topics(context, *args, **kwargs):
    """
    Render ros_gz_bridge_template.yaml by replacing {prefix} placeholders.
    - ROS names: leave relative (no leading '/'); node namespace will add <prefix>/...
      Keep global ones absolute ('/tf', '/clock') in the template so they stay global.
    - GZ names: keep absolute and include '{prefix}' where you really want it.
    """
    pkg_share_gazebo = kwargs['pkg_share_gazebo']
    prefix = LaunchConfiguration('prefix').perform(context) or ""
    # Sane normalized string for replacement (no trailing slash)
    name = prefix.rstrip('/')

    template_path = os.path.join(pkg_share_gazebo, 'config', 'ros_gz_bridge_template.yaml')

    # Decide output file name first
    file_name = f'ros_gz_bridge_{name}.yaml' if name else 'ros_gz_bridge.yaml'
    modified_config_path = os.path.join(pkg_share_gazebo, 'config', file_name)

    # Read template as text and do a simple string replace
    with open(template_path, 'r', encoding='utf-8') as f:
        txt = f.read()

    # Replace {prefix} placeholders.
    # NOTE: We DO NOT add slashes here. Put them in the template where needed.
    txt = txt.replace('{prefix}', name)
    # txt = txt.replace('{n_prefix}', 'x3')

    os.makedirs(os.path.dirname(modified_config_path), exist_ok=True)
    with open(modified_config_path, 'w', encoding='utf-8') as f:
        f.write(txt)

    # Return a LaunchConfiguration setter so downstream Nodes can use it
    return [SetLaunchConfiguration('bridge_config_file', modified_config_path)]


def generate_launch_description():
    # === Package paths ===
    desc_pkg = FindPackageShare('x3_uav_description').find('x3_uav_description')
    ign_pkg  = FindPackageShare('x3_uav_ignition').find('x3_uav_ignition')

    use_jsp_gui = LaunchConfiguration('use_jsp_gui')
    use_jsp_gui_arg = DeclareLaunchArgument('use_jsp_gui', default_value='false')

    use_jsp = LaunchConfiguration('use_jsp')
    use_jsp_arg = DeclareLaunchArgument('use_jsp', default_value='false')

    robot_name = LaunchConfiguration('robot_name')
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='x3_uav')

    prefix = LaunchConfiguration('prefix')
    prefix_arg = DeclareLaunchArgument('prefix', default_value='x3')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    xacro_file = LaunchConfiguration('xacro_file')
    xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        default_value=PathJoinSubstitution([
            desc_pkg,
            'urdf',
            PythonExpression(["'", robot_name, "' + '.urdf.xacro'"])
        ])
    )
    use_ignition = LaunchConfiguration('use_ignition')
    use_ignition_arg = DeclareLaunchArgument('use_ignition', default_value='true')

    use_plugin = LaunchConfiguration('use_plugin')
    use_plugin_arg = DeclareLaunchArgument('use_plugin', default_value='true')
    
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_ros2_control_arg = DeclareLaunchArgument('use_ros2_control', default_value='False')

    configure_controller = LaunchConfiguration('configure_controller')
    default_configure_controller = DeclareLaunchArgument(
        'configure_controller',
        default_value=PathJoinSubstitution(
            [desc_pkg, 'config', robot_name, 'ros2_controllers.yaml']
        )
    )

    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    use_mock_hardware_arg = DeclareLaunchArgument('use_mock_hardware', default_value='False')

    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false')

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

    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_roll = LaunchConfiguration('spawn_roll')
    spawn_pitch = LaunchConfiguration('spawn_pitch')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    
    declare_x_cmd = DeclareLaunchArgument('spawn_x', default_value='0.0', description='x component of initial position, meters')
    declare_y_cmd = DeclareLaunchArgument('spawn_y', default_value='0.0', description='y component of initial position, meters')
    declare_z_cmd = DeclareLaunchArgument('spawn_z', default_value='0.05', description='z component of initial position, meters')
    declare_roll_cmd = DeclareLaunchArgument('spawn_roll', default_value='0.0', description='roll angle of initial orientation, radians')
    declare_pitch_cmd = DeclareLaunchArgument('spawn_pitch', default_value='0.0', description='pitch angle of initial orientation, radians')
    declare_yaw_cmd = DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='yaw angle of initial orientation, radians')
    # ---------------------------------------------------------------------

    # === robot_state_publisher ===
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_jsp': use_jsp,
            'use_jsp_gui': use_jsp_gui,
            'robot_name': robot_name,
            'prefix': prefix,
            'use_sim_time': use_sim_time,
            'xacro_file': xacro_file,
            'use_ignition': use_ignition,
            'use_plugin': use_plugin,
            'use_ros2_control': use_ros2_control,
            'use_mock_hardware': use_mock_hardware,
            'configure_controller': configure_controller,
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

    set_ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.pathsep.join([
            os.path.join(desc_pkg, '..'),
            os.path.join(ign_pkg, 'worlds'),
            # os.path.join(ign_pkg, 'models'),
            '/usr/share/gazebo-11/models',
        ]),
        condition=IfCondition(use_ignition)
    )

    set_ign_file_path = AppendEnvironmentVariable(
        name='IGN_FILE_PATH',
        value=os.path.join(desc_pkg, '..'),
        condition=IfCondition(use_ignition)
    )

    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')

    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[(
            'gz_args', [' -r -s -v 4 ', world_file]
        )],
        condition=IfCondition(use_ignition),
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-g ']
        }.items(),
        condition=IfCondition(AndSubstitution(
            NotSubstitution(headless),
            use_ignition
        )),
    )

    bridge_config_action = OpaqueFunction(
        function=bridge_topics, kwargs={'pkg_share_gazebo': ign_pkg}
    )

    # default_ros_gz_bridge_config_file_path = os.path.join(
    #     ign_pkg, 'config', 'ros_gz_bridge.yaml'
    # )

    ignition_no_namespace_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_tf_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )


    # Remove the tel_prefix variable and combine into one expression
    topic_with_prefix = PythonExpression([
        "'/X3/cmd_vel' if '", prefix, "' == '' else '/' + '", prefix, "' + '/cmd_vel'",
        " + '@geometry_msgs/msg/Twist@gz.msgs.Twist'"
    ])

    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        namespace=prefix,
        output='screen',
        arguments=[topic_with_prefix], 
        parameters=[{'use_sim_time': use_sim_time}],
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=prefix,
        parameters=[{
            'config_file': LaunchConfiguration('bridge_config_file'),
            'use_sim_time': use_sim_time,
        }],
        output='screen',
        condition=IfCondition(use_ignition),
    )

    start_gazebo_ros_bottom_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='bottom_camera_image_bridge',
        namespace=prefix,
        arguments=[
            [prefix, '/bottom_camera/image'],
        ],
        remappings=[
            ([prefix, '/bottom_camera/image'], 'bottom_camera/color/image_raw'),
            ([prefix, '/bottom_camera/image/compressed'], 'bottom_camera/color/image_raw/compressed'),
            ([prefix, '/bottom_camera/image/compressedDepth'], 'bottom_camera/color/image_raw/compressedDepth'),
            ([prefix, '/bottom_camera/image/theora'], 'bottom_camera/color/image_raw/theora'),
        ],
        condition=IfCondition(use_ignition),
        output='screen',
    )

    start_gazebo_ros_bottom_depth_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='bottom_camera_depth_image_bridge',
        namespace=prefix,
        arguments=[
            [prefix, '/bottom_camera/depth_image'],
        ],
        remappings=[
            ([prefix, '/bottom_camera/depth_image'], 'bottom_camera/depth_image/image_raw'),
            ([prefix, '/bottom_camera/depth_image/compressed'], 'bottom_camera/depth_image/image_raw/compressed'),
            ([prefix, '/bottom_camera/depth_image/compressedDepth'], 'bottom_camera/depth_image/image_raw/compressedDepth'),
            ([prefix, '/bottom_camera/depth_image/theora'], 'bottom_camera/depth_image/image_raw/theora'),
        ],
        condition=IfCondition(use_ignition),
        output='screen',
    )

    start_gazebo_ros_front_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='front_camera_image_bridge',
        namespace=prefix,
        arguments=[
            [prefix, '/front_camera/image'],
        ],
        remappings=[
            ([prefix, '/front_camera/image'], 'front_camera/color/image_raw'),
            ([prefix, '/front_camera/image/compressed'], 'front_camera/color/image_raw/compressed'),
            ([prefix, '/front_camera/image/compressedDepth'], 'front_camera/color/image_raw/compressedDepth'),
            ([prefix, '/front_camera/image/theora'], 'front_camera/color/image_raw/theora'),
        ],
        condition=IfCondition(use_ignition),
        output='screen',
    )

    start_gazebo_ros_front_depth_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='front_camera_depth_bridge',
        namespace=prefix,
        arguments=[
            [prefix, '/front_camera/depth_image'],
        ],
        remappings=[
            ([prefix, '/front_camera/depth_image'], 'front_camera/depth_image/image_raw'),
            ([prefix, '/front_camera/depth_image/compressed'], 'front_camera/depth_image/image_raw/compressed'),
            ([prefix, '/front_camera/depth_image/compressedDepth'], 'front_camera/depth_image/image_raw/compressedDepth'),
            ([prefix, '/front_camera/depth_image/theora'], 'front_camera/depth_image/image_raw/theora'),
        ],
        condition=IfCondition(use_ignition),
        output='screen',
    )
    
    model_name = PythonExpression([
        "'", robot_name, "' if '", prefix, "' == '' else '", prefix, "'"
    ])

    robot_description_topic = PythonExpression([
        "'/robot_description' if '", prefix, "' == '' else '/", prefix, "/robot_description'"
    ])

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=prefix,
        output='screen',
        arguments=[
            '-topic', robot_description_topic,
            '-name', model_name,
            '-allow_renaming', 'true',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-R', spawn_roll,
            '-P', spawn_pitch,
            '-Y', spawn_yaw
        ],
        condition=IfCondition(use_ignition),
    )

    static_tf_world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_odom',
        namespace=prefix,
        arguments=[
            '0', '0', '0.0',   # translation (x, y, z)
            '0', '0', '0',   # rotation (roll, pitch, yaw)
            'world',         # parent frame
            [prefix, '/odom']  # child frame (namespace aware)
        ],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(use_jsp_gui_arg)
    ld.add_action(use_jsp_arg)
    ld.add_action(prefix_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(robot_name_arg)
    ld.add_action(xacro_file_arg)
    ld.add_action(use_ignition_arg)
    ld.add_action(use_plugin_arg)
    ld.add_action(use_ros2_control_arg)
    ld.add_action(use_mock_hardware_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(headless_arg)
    ld.add_action(declare_world_file)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)

    ld.add_action(default_configure_controller)
    ld.add_action(robot_state_publisher_cmd)
    # ld.add_action(load_controllers_cmd)

    # ld.add_action(set_ign_resource_path)
    # ld.add_action(set_ign_file_path)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(bridge_config_action)
    ld.add_action(ignition_no_namespace_bridge)
    ld.add_action(cmd_vel_bridge)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_bottom_image_bridge_cmd)
    ld.add_action(start_gazebo_ros_bottom_depth_bridge_cmd)
    ld.add_action(start_gazebo_ros_front_image_bridge_cmd)
    ld.add_action(start_gazebo_ros_front_depth_bridge_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(static_tf_world_to_odom)

    return ld
