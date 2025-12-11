from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # -------------------- Substitutions --------------------

    model       = LaunchConfiguration('model')
    config_file = LaunchConfiguration('config_file')
    enable_audio_input  = LaunchConfiguration('enable_audio_input')
    enable_audio_output = LaunchConfiguration('enable_audio_output')
    
    # -------------------- Launch Arguments --------------------

    model_arg = DeclareLaunchArgument(
        'model',default_value='4', description='Model number'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value='robot_config_roscon_2025', description='Config file name'
    )

    enable_audio_input_arg = DeclareLaunchArgument(
        'enable_audio_input',
        default_value='false',
        description='Enable microphone input and listening'
    )

    enable_audio_output_arg = DeclareLaunchArgument(
        'enable_audio_output',
        default_value='false',
        description='Enable text-to-speech output'
    )

    # -------------------- Node Definitions --------------------

    chat_interface_node = Node(
        package='chatty',
        executable='chat_gui',
        name='chat_gui',
        output='screen'
    )

    chat_manager_node = Node(
        package='chatty',
        executable='chat_manager',
        name='chat_manager',
        # parameters=[{
        #     'config_file': config_file
        # }],
        output='screen',
    )

    task_manager_node = Node(
        package='chatty',
        executable='task_manager',
        name='task_manager',
        parameters=[{
            'model': model,
            'config_file': config_file,
        }],
        output='screen',
    )

    microphone_node = Node(
        package='chatty',
        executable='microphone',
        name='microphone',
        output='screen',
        condition=IfCondition(enable_audio_input)
    )

    speaking_node = Node(
        package='chatty',
        executable='speak',
        name='tts_speaker',
        output='screen',
        condition=IfCondition(enable_audio_output)
    )

    listening_node = Node(
        package='chatty',
        executable='listen',
        name='whisper_listener',
        output='screen',
        condition=IfCondition(enable_audio_input)
    )

    time_pub_node = Node(
        package='chatty',
        executable='time',
        name='time_publisher',
        output='screen' 
    )

    # -------------------- Return LaunchDescription --------------------

    return LaunchDescription([
        model_arg,
        config_file_arg,
        enable_audio_input_arg,
        enable_audio_output_arg,

        chat_interface_node,
        chat_manager_node,
        task_manager_node,

        microphone_node,
        speaking_node,
        listening_node,

        time_pub_node
    ])

