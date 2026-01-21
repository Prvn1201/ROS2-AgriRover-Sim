import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Locate the default config file from slam_toolbox
    slam_config_file = PathJoinSubstitution([
        FindPackageShare('slam_toolbox'),
        'config',
        'mapper_params_online_async.yaml'
    ])

    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_file, # Load defaults first
            {
                'use_sim_time': use_sim_time,
                'base_frame': 'base_link',  # <--- CRITICAL FIX: Match your TF tree
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan'
            }
        ]
    )

    # RViz
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
        slam_toolbox_node,
        rviz_node
    ])