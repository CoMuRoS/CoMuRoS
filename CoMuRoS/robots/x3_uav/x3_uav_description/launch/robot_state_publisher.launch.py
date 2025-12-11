from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    descri_pkg_share = get_package_share_directory('x3_uav_description')

    # Launch Configurations
    use_jsp = LaunchConfiguration('use_jsp')
    use_jsp_arg = DeclareLaunchArgument('use_jsp', default_value='false')

    use_jsp_gui = LaunchConfiguration('use_jsp_gui')
    use_jsp_gui_arg = DeclareLaunchArgument('use_jsp_gui', default_value='true')

    prefix = LaunchConfiguration('prefix')
    prefix_arg = DeclareLaunchArgument('prefix', default_value='')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    robot_name = LaunchConfiguration('robot_name')
    use_robot_name_arg = DeclareLaunchArgument('robot_name', default_value='x3_uav')

    xacro_file = LaunchConfiguration('xacro_file')
    xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        default_value=PathJoinSubstitution([
            descri_pkg_share,
            'urdf',
            PythonExpression(["'", robot_name, "' + '.urdf.xacro'"])
        ])
    )

    use_ignition = LaunchConfiguration('use_ignition')
    use_ignition_arg = DeclareLaunchArgument('use_ignition', default_value='false')

    use_plugin = LaunchConfiguration('use_plugin')
    use_plugin_arg = DeclareLaunchArgument('use_plugin', default_value='false')

    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_ros2_control_arg = DeclareLaunchArgument('use_ros2_control', default_value='false')

    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    use_mock_hardware_arg = DeclareLaunchArgument('use_mock_hardware', default_value='false')

    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')

    configure_controller = LaunchConfiguration('configure_controller')
    default_configure_controller = DeclareLaunchArgument(
        'configure_controller',
        default_value=PathJoinSubstitution(
            [descri_pkg_share, 'config', robot_name, 'ros2_controllers.yaml']
        )
    )

    frame_prefix = PythonExpression([
        '"', prefix, '" + "/" if "', prefix, '" else ""'
    ])

    robot_description_content = Command([
        'xacro ', xacro_file,
        ' prefix:=', prefix,
        ' use_ignition:=', use_ignition,
        ' use_plugin:=', use_plugin,
    ])

    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Robot State Publisher Node
    rsp_node = Node(
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

    # Joint State Publisher Node (without GUI)
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
        condition=IfCondition(use_jsp),
    )

    # Joint State Publisher GUI Node
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
        condition=IfCondition(use_jsp_gui),
    )

    # RViz Configuration
    rviz_file = PythonExpression([
        "'display.rviz' if ", "'", prefix, "'", " == '' else ", "'", prefix, "'", " + '.rviz'"
    ])

    rviz_config = PathJoinSubstitution([
        descri_pkg_share,
        'rviz',
        rviz_file
    ])

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=prefix,
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        use_jsp_arg,
        use_jsp_gui_arg,
        use_sim_time_arg,
        prefix_arg,
        use_robot_name_arg,
        xacro_file_arg,
        use_ignition_arg,
        use_plugin_arg,
        default_configure_controller,
        use_ros2_control_arg,
        use_mock_hardware_arg,
        use_rviz_arg,

        rsp_node,
        jsp_node,
        jsp_gui_node,
        rviz_node
    ])