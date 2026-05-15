import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess ,TimerAction , IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare 
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare the path to the world file
    world_file_name = 'disaster.world'
    world_path = os.path.join(get_package_share_directory('champ_gazebo'), 'worlds', world_file_name)

    # Declare launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Full path to the world model file to load')

    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Flag to enable gazebo GUI')

    # Get the launch configuration variables
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    gazebo_config = os.path.join(
        FindPackageShare(package="champ_gazebo").find("champ_gazebo"),
        "config", "gazebo.yaml"
    )
    pkg_share = FindPackageShare(package="champ_description").find("champ_description")
    launch_dir = os.path.join(pkg_share, "launch")

    # Start Gazebo server
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[
            "gzserver",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
            world,                            
            '--ros-args',
            '--params-file', gazebo_config
        ],
        cwd=[launch_dir],
        output="screen",
    )

    # Start Gazebo client (GUI) if gui is set to true
    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(gui),
        cmd=['gzclient'],
        output='screen')
    



    # if not is_valid_to_launch():
    #     print('Cannot launch fake robot on Raspberry Pi')
    #     return LaunchDescription([])

    # Get package share directories
    config_pkg_share = get_package_share_directory("go2_config")
    descr_pkg_share = get_package_share_directory("go2_description")
    default_world_path = os.path.join(config_pkg_share, "worlds", "default.world")

    # --- GO2 Launch via ExecuteProcess ---
    # Launch Go2 from go2_config's gazebo.launch.py using ExecuteProcess,
    # so that we can use an exit handler.
    go2_cmd = [
        "ros2", "launch", "go2_config", "gazebo.launch.py"
        # "--ros-args", "-p", "use_sim_time:=", LaunchConfiguration("use_sim_time")
    ]

    
    go2_ld = ExecuteProcess(
        cmd=go2_cmd,
        output="screen"
    )

    go2_timer = TimerAction(
        period=7.0,
        actions=[go2_ld]
    )
 
    # ====================================== SJTU Drone ===========================
    sjtu_drone_bringup_path =  ExecuteProcess(
                    cmd=['ros2', 'launch', 'sjtu_drone_bringup', 'sjtu_drone_bringup.launch.py'],
                    output='screen'
                )
    
    sjtu_timer = TimerAction(
        period=3.0,
        actions=[sjtu_drone_bringup_path]
    )
 
        
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_gui_cmd)

    # Add any actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # # Declare common launch arguments
    # ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true"))
    # ld.add_action(DeclareLaunchArgument("rviz", default_value="false", description="Launch rviz"))
    # ld.add_action(DeclareLaunchArgument("robot_name", default_value="champ", description="Robot name"))
    # ld.add_action(DeclareLaunchArgument("start_rviz", default_value="false", description="Whether to execute rviz2"))
    # ld.add_action(DeclareLaunchArgument("prefix", default_value="\"\"", description="Prefix of the joint and link names"))

    ld.add_action(go2_timer)
    ld.add_action(sjtu_timer)

    return ld
