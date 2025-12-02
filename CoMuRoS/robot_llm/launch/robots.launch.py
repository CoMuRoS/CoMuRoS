from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    mobile_robot = Node(
        package='robot_llm',
        executable='mobile_robot',
        name='mobile_robot',
        parameters=[
            {'robot_name': 'burger'},
        ],
        output='screen'
    )

    manipulator_robot = Node(
        package='robot_llm', 
        executable='manipulator_robot',
        name='manipulator_robot',
        parameters=[
            {'robot_name': 'x-arm'},
        ],
        output='screen'
    )

    return LaunchDescription([
        mobile_robot,
        manipulator_robot
    ])