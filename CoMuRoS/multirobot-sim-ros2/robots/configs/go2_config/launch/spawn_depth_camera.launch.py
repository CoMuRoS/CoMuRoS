#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the directory of the current package
    pkg_share = get_package_share_directory('go2_config')

    # Paths to the world and robot description files
    world_path = os.path.join(pkg_share, 'worlds', 'empty.world')
    urdf_path = os.path.join(pkg_share, 'urdf', 'standalone_depth_camera.urdf')

    # Launch Gazebo with the empty world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    # Spawn the depth camera entity
    spawn_camera = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'depth_camera',
            '-file', urdf_path,
            '-x', '-5.289314',
            '-y', '1.943117',
            '-z', '2.154822',
            '-R', '0',
            '-P', '0.261',
            '-Y', '0'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_camera
    ])
