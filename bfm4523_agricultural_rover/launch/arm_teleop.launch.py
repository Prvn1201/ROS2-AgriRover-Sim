from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bfm4523_agricultural_rover',
            executable='arm_teleop',
            name='arm_teleop',
            output='screen',
            # This prefix forces the node to open in a new terminal window
            # so you can type commands into it.
            prefix='gnome-terminal --' 
        ),
    ])
