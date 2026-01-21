import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_name = 'Autonomous_Vin_V1_description' 
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Path to the world file
    world_file_path = os.path.join(pkg_share, 'worlds', 'vin.world')

    # 2. Path to the gazebo_ros package
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # 3. Important: Tell Gazebo where to find your "my_custom_world" model
    # The model is inside share/package_name/worlds/
    model_path_to_add = os.path.join(pkg_share, 'worlds')

    env_var = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        model_path_to_add
    )

    # 4. Launch Gazebo Server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file_path}.items()
    )

    # 5. Launch Gazebo Client (GUI)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        )
    )

    return LaunchDescription([
        env_var,
        gzserver,
        gzclient,
    ])
