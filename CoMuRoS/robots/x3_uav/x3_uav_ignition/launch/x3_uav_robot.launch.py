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

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    prefix = LaunchConfiguration('prefix')
    use_ignition = LaunchConfiguration('use_ignition')
    use_plugin = LaunchConfiguration('use_plugin')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    use_rviz = LaunchConfiguration('use_rviz')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_roll = LaunchConfiguration('spawn_roll')
    spawn_pitch = LaunchConfiguration('spawn_pitch')
    spawn_yaw = LaunchConfiguration('spawn_yaw')





    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='x3_uav')
    prefix_arg = DeclareLaunchArgument('prefix', default_value='x3')
    use_ignition_arg = DeclareLaunchArgument('use_ignition', default_value='true')
    use_plugin_arg = DeclareLaunchArgument('use_plugin', default_value='true')
    use_ros2_control_arg = DeclareLaunchArgument('use_ros2_control', default_value='False')
    use_mock_hardware_arg = DeclareLaunchArgument('use_mock_hardware', default_value='False')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false')
    declare_x_cmd = DeclareLaunchArgument('spawn_x', default_value='0.0', description='x component of initial position, meters')
    declare_y_cmd = DeclareLaunchArgument('spawn_y', default_value='0.0', description='y component of initial position, meters')
    declare_z_cmd = DeclareLaunchArgument('spawn_z', default_value='0.05', description='z component of initial position, meters')
    declare_roll_cmd = DeclareLaunchArgument('spawn_roll', default_value='0.0', description='roll angle of initial orientation, radians')
    declare_pitch_cmd = DeclareLaunchArgument('spawn_pitch', default_value='0.0', description='pitch angle of initial orientation, radians')
    declare_yaw_cmd = DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='yaw angle of initial orientation, radians')


    xacro_file = PathJoinSubstitution([
        desc_pkg,
        'urdf',
        PythonExpression(["'", robot_name, "' + '.urdf.xacro'"])
    ])


    robot_description_content = Command([
        'xacro ', xacro_file,
        ' prefix:=', prefix,
        ' use_ignition:=', use_ignition,
        ' use_plugin:=', use_plugin,
    ])

    robot_description = ParameterValue(robot_description_content, value_type=str)

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=prefix, 
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
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


    bridge_config_action = OpaqueFunction(
        function=bridge_topics, kwargs={'pkg_share_gazebo': ign_pkg}
    )

    topic_with_prefix = PythonExpression([
        "'/X3/cmd_vel' if '", prefix, "' == '' else '/' + '", prefix, "' + '/cmd_vel'",
        " + '@geometry_msgs/msg/Twist@gz.msgs.Twist'"
    ])

    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        namespace=prefix,
        arguments=[topic_with_prefix], 
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_ignition),
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
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_ignition),
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
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_ignition),
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
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_ignition),
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
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_ignition),
    )
    



    child_frame = [prefix, '/odom']

    static_tf_world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_prefix_odom',
        namespace=prefix,
        arguments=[
            # '--x', spawn_x, '--y', spawn_y, '--z', spawn_z,
            # '--roll', spawn_roll, '--pitch', spawn_pitch, '--yaw', spawn_yaw,
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'world',
            '--child-frame-id', child_frame
        ],
        output='screen'
    )   

    # Dynamically choose RViz file based on robot name
    rviz_file = PythonExpression([
        "'display.rviz' if ", "'", prefix, "'", " == '' else ", "'", prefix, "'", " + '.rviz'"
    ])

    # Join path dynamically using substitutions
    rviz_config = PathJoinSubstitution([
        desc_pkg,
        'rviz',
        rviz_file
    ])

    # RViz2 Node
    multi_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=prefix,
        output='log',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([

        use_sim_time_arg,
        robot_name_arg,
        prefix_arg,

        use_ignition_arg,
        use_plugin_arg,

        use_ros2_control_arg,
        use_mock_hardware_arg,

        use_rviz_arg,

        declare_x_cmd,
        declare_y_cmd,
        declare_z_cmd,
        declare_roll_cmd,
        declare_pitch_cmd,
        declare_yaw_cmd,

        robot_state_publisher_cmd,
        start_gazebo_ros_spawner_cmd,

        bridge_config_action,
        cmd_vel_bridge,
        start_gazebo_ros_bridge_cmd,
        start_gazebo_ros_bottom_image_bridge_cmd,
        start_gazebo_ros_bottom_depth_bridge_cmd,
        start_gazebo_ros_front_image_bridge_cmd,
        start_gazebo_ros_front_depth_bridge_cmd,

        static_tf_world_to_odom,

        multi_rviz
    ])