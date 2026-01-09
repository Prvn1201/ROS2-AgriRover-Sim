import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from os.path import join

def generate_launch_description():
    # 1. PATH CONFIGURATION
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_my_robot = get_package_share_directory('bfm4523_agricultural_rover')
    
    xacro_file = os.path.join(pkg_my_robot, 'urdf', 'autonomous_vin.xacro')
    bridge_config_file = os.path.join(pkg_my_robot, 'config', 'ros_gz_bridge_gazebo.yaml')
    
    # Use the custom world with sensors if it exists, otherwise empty
    # Change 'lidar_world.sdf' to 'empty.sdf' if you haven't created the world file yet
    world_file = os.path.join(pkg_my_robot, 'urdf', 'lidar_world.sdf')

    # 2. PROCESS XACRO
    doc = xacro.process_file(xacro_file)
    robot_desc = {'robot_description': doc.toxml()}

    # 3. NODES
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_desc],
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r -v 4 {world_file}"
        }.items()
    )

    # Spawn Robot
    spawn_robot = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                "-topic", "/robot_description",
                "-name", "Autonomous_Vin_V1",
                "-allow_renaming", "false",
                "-x", "0.0", "-y", "0.0", "-z", "0.5", "-Y", "0.0"
            ],
            output='screen'
        )]
    )

    # Bridge (With Sim Time enabled)
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_file,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen',
        arguments=['--ros-args', '-p', 'use_sim_time:=true']
    )

    # --- NEW: TELEOP TERMINAL ---
    # This opens a new Gnome Terminal window running the keyboard controller
    teleop_terminal = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        teleop_terminal 
    ])
