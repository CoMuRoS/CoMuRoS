from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    descri_pkg_share = get_package_share_directory('x3_uav_description')

    use_jsp_gui = LaunchConfiguration('use_jsp_gui')
    use_jsp_gui_arg = DeclareLaunchArgument('use_jsp_gui', default_value='false')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')

    xacro_file = os.path.join(descri_pkg_share, 'urdf', 'x3_uav.urdf.xacro')

    # FIX: pass command tokens separately (no trailing space)
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen',
    )

    # JSP nodes: no need to pass robot_description as a parameter
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=UnlessCondition(use_jsp_gui),
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_jsp_gui),
    )

    rviz_config = PathJoinSubstitution([descri_pkg_share, 'rviz', 'display.rviz'])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        use_jsp_gui_arg,
        use_sim_time_arg,
        use_rviz_arg,
        rsp_node,
        jsp_node,
        jsp_gui_node,
        rviz_node
    ])
