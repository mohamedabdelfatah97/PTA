import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg            = get_package_share_directory('pta_robot_sim')
    urdf_file      = os.path.join(pkg, 'urdf', 'pta_robot.urdf')
    world_file     = os.path.join(pkg, 'worlds', 'test_room.world')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items()
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'ekf.yaml'),
            {'use_sim_time': True},
        ]
    )

    laser_merger = Node(
        package='dual_laser_merger',
        executable='dual_laser_merger_node',
        name='dual_laser_merger',
        output='screen',
        parameters=[{
            'use_sim_time':      True,
            'laser_1_topic':     '/scan_front',
            'laser_2_topic':     '/scan_rear',
            'merged_scan_topic': '/scan',
            'target_frame':      'base_link',
            'angle_min':         -3.14159,
            'angle_max':          3.14159,
            'range_min':          0.15,
            'range_max':          12.0,
        }]
    )

    spawn = TimerAction(period=8.0, actions=[
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'pta_robot',
                '-x', '0.5',
                '-y', '1.385',
                '-z', '0.0',
                '-Y', '0.0',
            ],
            output='screen'
        )
    ])

    rviz = TimerAction(period=10.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg, 'rviz', 'pta_robot.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'slam_toolbox.yaml'),
            {'use_sim_time': True},
        ],
        remappings=[('/odom', '/odometry/filtered')]
    )

    return LaunchDescription([
        gazebo, rsp, jsp, ekf_node, laser_merger,
        spawn, rviz, slam
    ])
