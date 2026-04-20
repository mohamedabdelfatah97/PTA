import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('PTA_ROS2_Gazebo_URDF')
    urdf_file = os.path.join(pkg, 'urdf', 'pta_robot_planar.urdf')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    hospital_world = os.path.expanduser(
        '~/sim_ws/src/PTA_Hospital/worlds/hospital.world'
    )

    # Hospital map — must be built first using hospital_mapping.launch.py
    hospital_map = os.path.expanduser('~/maps/hospital.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # params_file argument — pass a combo file at launch time
    # default: nav2_params.yaml
    # usage: ros2 launch ... params_file:=config/nav2_params_combo1_dwb_navfn_diffdrive.yaml
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg, 'config', 'nav2_params.yaml'),
        description='Full path to nav2 params file'
    )
    nav2_params = LaunchConfiguration('params_file')

    # t=0s: Gazebo with hospital world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': hospital_world,
            'verbose': 'false',
        }.items()
    )

    # t=0s: Robot state publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen'
    )

    # t=0s: Joint state publisher
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # t=0s: EKF node — fuses odom + IMU into /odometry/filtered
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

    # t=0s: Laser merger — combines front and rear LiDAR into /scan
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

    # t=20s: Spawn robot near elevator
    # Position: x=-1.7, y=13.5, facing south (away from elevator)
    spawn = TimerAction(period=20.0, actions=[
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'pta_robot',
                '-x', '-1.7',
                '-y', '13.5',
                '-z', '0.0',
                '-Y', '3.14159',
            ],
            output='screen'
        )
    ])

    # t=65s: RViz with saved config
    rviz = TimerAction(period=65.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/pta-sim/rviz_configs/pta_robot.rviz'],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])

    # t=70s: Stage 1 — localization only (map_server + amcl)
    # AMCL initial pose set via set_initial_pose: true in nav2_params.yaml
    # Activation handled by ~/activate_hospital_nav2.sh
    localization = TimerAction(period=70.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'localization_launch.py')
            ),
            launch_arguments={
                'map': hospital_map,
                'use_sim_time': 'true',
                'params_file': nav2_params,
                'autostart': 'false',
            }.items()
        )
    ])

    # t=90s: Stage 2 — full navigation stack
    # Activation handled by ~/activate_hospital_nav2.sh after localization confirms
    navigation = TimerAction(period=90.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params,
                'autostart': 'false',
            }.items()
        )
    ])

    return LaunchDescription([
        declare_params_file,
        gazebo, rsp, jsp, ekf_node, laser_merger,
        spawn, rviz,
        localization, navigation,
    ])