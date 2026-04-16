import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                             OpaqueFunction, IncludeLaunchDescription,
                             TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg            = get_package_share_directory('pta_robot_sim')
    nav2_bringup   = get_package_share_directory('nav2_bringup')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    urdf_file = os.path.join(pkg, 'urdf', 'pta_robot.urdf')
    world_file = os.path.join(pkg, 'worlds', 'test_room.world')
    map_file   = os.path.join(pkg, 'maps', 'test_room.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    combo = LaunchConfiguration('nav_combo').perform(context)
    combo_to_file = {
        'combo1': os.path.join(pkg, 'config', 'nav2', 'combo1_dwb_navfn_diffdrive.yaml'),
        'combo2': os.path.join(pkg, 'config', 'nav2', 'combo2_dwb_navfn_holonomic.yaml'),
        'combo3': os.path.join(pkg, 'config', 'nav2', 'combo3_mppi_navfn.yaml'),
        'combo4': os.path.join(pkg, 'config', 'nav2', 'combo4_mppi_smac2d.yaml'),
    }
    nav2_params = combo_to_file[combo]
    print(f'[nav2.launch.py] Using Nav2 combo: {combo} → {nav2_params}')

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

    spawn = TimerAction(period=20.0, actions=[
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

    rviz = TimerAction(period=65.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg, 'rviz', 'pta_robot.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])

    localization = TimerAction(period=70.0, actions=[
        GroupAction(
            scoped=True,
            forwarding=False,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_bringup, 'launch', 'localization_launch.py')
                    ),
                    launch_arguments={
                        'map':          map_file,
                        'use_sim_time': 'true',
                        'params_file':  nav2_params,
                        'autostart':    'false',
                    }.items()
                )
            ]
        )
    ])

    navigation = TimerAction(period=90.0, actions=[
        GroupAction(
            scoped=True,
            forwarding=False,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': 'true',
                        'params_file':  nav2_params,
                        'autostart':    'false',
                    }.items()
                )
            ]
        )
    ])

    return [
        gazebo, rsp, jsp, ekf_node, laser_merger,
        spawn, rviz,
        localization, navigation,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'nav_combo',
            default_value='combo1',
            choices=['combo1', 'combo2', 'combo3', 'combo4'],
            description='Which Nav2 parameter set to use'
        ),
        OpaqueFunction(function=launch_setup),
    ])
