from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    holonomic_robot1 = Node(
        package='cleaning_bot',
        executable='holonomic_position_controller_service',
        name='cleaning_bot_position_controller',
        parameters=[{
            'namespace': 'r1',
        }],
        output='screen',
    )

    holonomic_robot2 = Node(
        package='cleaning_bot',
        executable='holonomic_position_controller_service',
        name='deliver_bot_position_controller',
        parameters=[{
            'namespace': 'r2',
        }],
        output='screen',        
    )

    drone_1 = Node(
        package='drone', 
        executable='drone_position_controller_service',
        name='drone_position_controller',
        parameters=[{
            'namespace': 'r3',
        }],
        output='screen',
    )

    return LaunchDescription([
        holonomic_robot1,
        holonomic_robot2,
        drone_1,


    ])