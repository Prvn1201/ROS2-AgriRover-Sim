import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('Autonomous_Vin_V1_description')
    
    # Paths
    map_file = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    
    # Point to YOUR custom config file
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # --- FIX START: Comment out the default params override ---
    # If you leave these lines active, they overwrite your custom file above!
    # nav2_params_file = PathJoinSubstitution([
    #     FindPackageShare('nav2_bringup'), 'params', 'nav2_params.yaml'
    # ])
    # --- FIX END --------------------------------------------

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'rviz',
            'nav2_default_view.rviz'
        ])],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        nav2_launch,
        rviz_node
    ])