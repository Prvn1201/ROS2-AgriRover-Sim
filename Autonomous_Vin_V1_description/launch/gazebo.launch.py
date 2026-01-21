from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. PATH SETUP
    share_dir = get_package_share_directory('Autonomous_Vin_V1_description')

    # Process URDF/Xacro
    xacro_file = os.path.join(share_dir, 'urdf', 'Autonomous_Vin_V1.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # World File
    world_file_name = 'my_environment.world' 
    world_path = os.path.join(share_dir, 'worlds', world_file_name)

    # Sim Time Configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 2. NODES

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time': use_sim_time}
        ]
    )

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
            'pause': 'false',
            'world': world_path,
            'verbose': 'true',  # Useful for debugging
        }.items()
    )

    # Gazebo Client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # Spawn Entity Node (Spawns the robot)
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'Autonomous_Vin_V1',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # 3. CONTROLLERS
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        output="screen",
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    # 4. EVENT HANDLERS (Wait for spawn before loading controllers)
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_spawn_node,
            on_exit=[diff_drive_spawner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_spawn_node,
            on_exit=[joint_broad_spawner],
        )
    )

    delayed_arm_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_spawn_node,
            on_exit=[arm_spawner],
        )
    )

    # 5. TWIST MUX & JOYSTICK
    twist_mux_params = os.path.join(share_dir, 'config', 'twist_mux.yaml')
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[
            ('cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')
        ]
    )

    joy_params = os.path.join(share_dir, 'config', 'ps4.yaml')
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params, {'use_sim_time': True}],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_params, {'use_sim_time': True}],
        remappings=[
            ('/cmd_vel', '/cmd_vel_joy')
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        delayed_arm_spawner,
        twist_mux_node,
        joy_node,
        teleop_node
    ])