import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('pta_robot_sim')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    urdf_file = os.path.join(pkg, 'urdf', 'pta_robot_planar.urdf')
    default_world = os.path.join(pkg, 'worlds', 'test_room.world')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': 'false',
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'pta_robot',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ]
    )

    mecanum_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'mecanum_drive_controller',
            '--controller-manager', '/controller_manager',
        ]
    )

    # Start joint_state_broadcaster after the robot is spawned
    start_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Start mecanum_drive_controller after joint_state_broadcaster is ready
    start_mecanum_ctrl_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mecanum_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Full path to world file'
        ),
        DeclareLaunchArgument(
            'x',
            default_value='0.65',
            description='Spawn x position'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='1.385',
            description='Spawn y position'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.0',
            description='Spawn z position'
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Spawn yaw'
        ),

        gazebo,
        robot_state_publisher,
        spawn_entity,
        start_jsb_after_spawn,
        start_mecanum_ctrl_after_jsb,
    ])