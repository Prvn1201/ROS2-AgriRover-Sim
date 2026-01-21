import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'Autonomous_Vin_V1_description'
    share_dir = get_package_share_directory(pkg_name)

    # Path to your world file
    world_file_name = 'my_environment.world'
    world_path = os.path.join(share_dir, 'worlds', world_file_name)

    # Gazebo Server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',  # Keep verbose to see map errors
            'pause': 'false'
        }.items()
    )

    # Gazebo Client (The GUI)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_client
    ])
