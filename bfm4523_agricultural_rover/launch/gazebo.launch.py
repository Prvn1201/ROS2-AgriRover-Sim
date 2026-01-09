import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from os.path import join

def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_rbot = get_package_share_directory('bfm4523_agricultural_rover')

    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'autonomous_vin.xacro')
    ros_gz_bridge_config = os.path.join(pkg_ros_gz_rbot, 'config', 'ros_gz_bridge_gazebo.yaml')
    
    # 1. World Path
    world_file_path = os.path.join(pkg_ros_gz_rbot, 'urdf', 'farm_world.sdf')

    robot_description_config = xacro.process_file(robot_description_file)
    
    # --- UPDATE: Force use_sim_time on the state publisher ---
    robot_description = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': True 
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # 2. Load the Custom World
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r -v 4 {world_file_path}"}.items()
    )

    spawn_robot = TimerAction(
        period=5.0,  
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                "-topic", "/robot_description",
                "-name", "Autonomous_Vin_V1",
                "-allow_renaming", "false", 
                "-x", "0.0",
                "-y", "0.0",
                "-z", "0.32",
                "-Y", "0.0"
            ],
            output='screen'
        )]
    )

    # --- UPDATE: Ensure Bridge explicitly uses sim time ---
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': ros_gz_bridge_config,
            'use_sim_time': True
        }],
        output='screen'
    )

    # --- UPDATE: Added Laser Filter to remove the Arm from Lidar scan ---
    laser_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[os.path.join(pkg_ros_gz_rbot, 'config', 'laser_filter.yaml')],
        remappings=[
            ('/scan', '/scan'),
            ('/scan_filtered', '/scan_filtered')
        ]
    )

    teleop_terminal = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        ros_gz_bridge,
        robot_state_publisher,
        laser_filter,
        teleop_terminal, 
    ])