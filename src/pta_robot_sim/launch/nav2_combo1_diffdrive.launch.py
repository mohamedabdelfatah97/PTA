import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('pta_robot_sim')

    sim_launch = '/home/pta-sim/pta_sim_ws/src/pta_robot_sim/launch/sim_diff_drive.launch.py'
    ekf_file = os.path.join(pkg, 'config', 'ekf_skid_steer.yaml')
    nav2_params = os.path.join(pkg, 'config', 'nav2', 'combo1_dwb_navfn_diffdrive.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'pta_robot.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    use_rviz = LaunchConfiguration('use_rviz')

    # -------------------------------------------------------------------------
    # Base skid-steer simulation + diff drive + EKF
    # -------------------------------------------------------------------------
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
            'controller_name': 'diff_drive_controller',
            'use_ekf': 'true',
            'ekf_file': ekf_file,
            'use_rviz': 'false',
            'test_phase': 'fusion',
            'record_data': 'false',
            'odom_topic': '/diff_drive_controller/odom',
        }.items()
    )

    # -------------------------------------------------------------------------
    # Nav2 nodes
    # -------------------------------------------------------------------------
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('cmd_vel_smoothed', '/cmd_vel_nav'),
            ('odom', '/odometry/filtered'),
        ],
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[nav2_params],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params],
    )

    rviz_node = TimerAction(period=30.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        )
    ])

    # Bridge Nav2 velocity commands to the diff drive controller input topic.
    # This matches the working manual relay test:
    #   /cmd_vel -> /diff_drive_controller/cmd_vel_unstamped
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_to_diffdrive_relay',
        arguments=['/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg, 'worlds', 'test_room.world')
        ),
        DeclareLaunchArgument('x', default_value='0.65'),
        DeclareLaunchArgument('y', default_value='1.385'),
        DeclareLaunchArgument('z', default_value='0.03'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('use_rviz', default_value='false'),

        sim,

        map_server,
        amcl,
        planner_server,
        controller_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager_localization,
        lifecycle_manager_navigation,
        cmd_vel_relay,
        rviz_node,
    ])