from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    cleaning_robot_node = Node(
        package='cleaning_bot',
        executable='cleaning_bot_llm',
        name='cleaning_bot_llm_node',
        parameters=[{
            'robot_name': 'cleaning_bot',
        }],
        output='screen',
    )

    delivery_bot_node = Node(
        package='delivery_bot',
        executable='delivery_bot_llm',
        name='delivery_bot_llm_node',
        parameters=[{
            'robot_name': 'delivery_bot',
        }],
        output='screen',
    )

    drone_node = Node(
        package='drone', 
        executable='drone_llm',
        name='drone_llm_node',
        parameters=[{
            'robot_name': 'drone',
        }],
        output='screen',
    )

    return LaunchDescription([
        cleaning_robot_node,
        delivery_bot_node,
        drone_node,

    ])