"""
nav2.launch.py — Nav2 benchmark launch (all combos)

ARCHITECTURE — Benchmark mode:
  /cmd_vel → gazebo_ros_planar_move plugin → body motion (kinematic)
  /odom   ← plugin → EKF → /odometry/filtered → Nav2

  All four combos share this identical backend. The only variable
  between combos is the Nav2 parameter file (planner + controller config).
  This gives a clean apples-to-apples algorithm comparison.

  combo1 — DWB + NavFn, nonholonomic constraint (vy=0, DifferentialMotionModel)
  combo2 — DWB + NavFn, holonomic (vy enabled, OmniMotionModel)
  combo3 — MPPI + NavFn, holonomic, smooth trajectories  ← recommended
  combo4 — MPPI + SmacPlanner2D, holonomic, corridor-aware

WHAT IS NOT IN THIS LAUNCH (by design):
  - mecanum_drive_node.py — removed. It published to /wheel_velocity_controller/commands
    which does not exist in ros2_controllers.yaml. It was dead code in this path.
  - Wheel odometry path — not needed. Plugin owns /odom. EKF fuses plugin odom + IMU.
  - True diff-drive backend for combo1 — combo1 is nonholonomic at the Nav2 layer only.
    If a true diff-drive physical simulation is needed later, use a separate launch.

WHEEL SPINNING (cosmetic only):
  joint_state_broadcaster + mecanum_drive_controller are spawned for visual realism.
  They do not affect body motion or odometry. They can be removed if not needed.
"""

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

    urdf_file  = os.path.join(pkg, 'urdf', 'pta_robot_mecanum_drive.urdf')
    world_file = os.path.join(pkg, 'worlds', 'test_room.world')
    map_file   = os.path.join(pkg, 'maps', 'test_room.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    combo = LaunchConfiguration('nav_combo').perform(context)
    combo_to_file = {
        'combo2': os.path.join(pkg, 'config', 'nav2', 'combo2_dwb_navfn_holonomic.yaml'),
        'combo3': os.path.join(pkg, 'config', 'nav2', 'combo3_mppi_navfn.yaml'),
        'combo4': os.path.join(pkg, 'config', 'nav2', 'combo4_mppi_smac2d.yaml'),
    }
    nav2_params = combo_to_file[combo]
    print(f'[nav2.launch.py] Benchmark mode — combo: {combo}')
    print(f'[nav2.launch.py] Params: {nav2_params}')

    # ── t=0: Gazebo ──────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items()
    )

    # ── t=0: Robot State Publisher ───────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    # ── t=0: EKF — fuses /odom (from planar_move plugin) + IMU ───────────────
    # /odom is published by libgazebo_ros_planar_move.so (object_controller plugin)
    # EKF publishes /odometry/filtered + odom→base_footprint TF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'ekf_holonomic.yaml'),
            {'use_sim_time': True},
        ]
    )

    # ── t=0: Laser merger ────────────────────────────────────────────────────
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

    # ── t=20: Spawn robot ────────────────────────────────────────────────────
    # z=0.03 — small clearance above floor prevents contact bounce at spawn
    spawn = TimerAction(period=20.0, actions=[
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'pta_robot',
                '-x', '0.65',
                '-y', '1.385',
                '-z', '0.03',
                '-Y', '0.0',
            ],
            output='screen'
        )
    ])

    # ── t=25: Spawn joint_state_broadcaster ─────────────────────────────────
    # Publishes /joint_states for wheel visual animation in RViz.
    # Not used for odometry in benchmark mode.
    spawn_jsb = TimerAction(period=25.0, actions=[
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/controller_manager',
            ],
            output='screen'
        )
    ])

    # ── t=27: Spawn mecanum_drive_controller ─────────────────────────────────
    # Cosmetic only — makes wheels spin visually.
    # Does not affect body motion (planar plugin handles that).
    # Does not affect odometry (plugin owns /odom).
    spawn_wheel_ctrl = TimerAction(period=27.0, actions=[
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'mecanum_drive_controller',
                '--controller-manager', '/controller_manager',
            ],
            output='screen'
        )
    ])

    # NOTE: mecanum_drive_node.py is NOT launched here.
    # It published Float64MultiArray to /wheel_velocity_controller/commands,
    # which does not exist in mecanum_drive_controller.yaml and is not used
    # in this benchmark path. Body motion still comes from gazebo_ros_planar_move.
    # mecanum_drive_controller is only for optional wheel/joint visualization.

    # ── t=65: RViz ───────────────────────────────────────────────────────────
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

    # ── t=70: Localization (map_server + amcl) ────────────────────────────────
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

    # ── t=90: Navigation stack ───────────────────────────────────────────────
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
        gazebo, rsp, ekf_node, laser_merger,
        spawn,
        spawn_jsb, spawn_wheel_ctrl,
        rviz,
        localization, navigation,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'nav_combo',
            default_value='combo2',
            choices=['combo2', 'combo3', 'combo4'],
            description='Which holonomic Nav2 parameter set to use'
        ),
        OpaqueFunction(function=launch_setup),
    ])