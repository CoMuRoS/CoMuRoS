from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        'model', default_value='1', description='Model number'
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value='default_config', description='Config file name'
    )

    # Use LaunchConfiguration substitutions
    model = LaunchConfiguration('model')
    config_file = LaunchConfiguration('config_file')

    chat_gui = Node(
        package='chatty',
        executable='chat_gui',
        name='chat_gui',
        output='screen'
    )

    chat_manager = Node(
        package='chatty',
        executable='chat_manager',
        name='chat_manager',
        output='screen'
    )

    task_manager = Node(
        package='chatty',
        executable='task_manager',
        name='task_manager',
        parameters=[
            {'model': model},
            {'config_file': config_file}
        ],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        config_file_arg,
        chat_gui,
        chat_manager,
        task_manager
    ])
