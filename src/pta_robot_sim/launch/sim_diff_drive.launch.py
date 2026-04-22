import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# =============================================================================
# sim_diff_drive.launch.py
# =============================================================================
# Purpose
#   Clean simulation launch focused on base / odometry validation for a
#   differential-drive setup, with Nav2 intentionally excluded.
#
# What this launch does
#   1) Starts Gazebo with the test world
#   2) Publishes the robot description
#   3) Spawns the robot into Gazebo
#   4) Spawns joint_state_broadcaster
#   5) Spawns the chosen drive controller (default: diff_drive_controller)
#   6) Optionally starts EKF and RViz
#   7) Optionally records structured odom test data to test_<N>.txt
#
# Test philosophy
#   Keep the base stack as small as possible while validating odometry.
#   Start with raw wheel odometry first. Only after raw odometry is clean
#   should EKF / sensor fusion be enabled.
#
# Default active phase
#   PHASE 1 — Wheel odometry only (raw /odom, no EKF)
#
# Later phases
#   PHASE 2 — D455 odometry only
#   PHASE 3 — T265 odometry only
#   PHASE 4 — EKF fusion after raw sources pass validation
#
# Logging strategy
#   This launch can write one structured text file per run:
#       test_1.txt, test_2.txt, test_3.txt, ...
#
#   The file stores JSON lines for:
#       - odometry under test (/odom by default)
#       - Gazebo ground truth (/gazebo/model_states)
#       - commanded motion (/cmd_vel)
#       - optional /joint_states
#
#   This makes it easy to compare:
#       /cmd_vel  vs /odom
#       /odom     vs ground truth
#
# Notes
#   - This file assumes a diff_drive_controller exists in ros2_control config.
#   - If your controller has a different name, pass controller_name:=<name>.
#   - This file does not start Nav2, slam_toolbox, AMCL, planner_server,
#     controller_server, or behavior trees.
# =============================================================================


def generate_launch_description():
    pkg = get_package_share_directory('pta_robot_sim')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    urdf_file = os.path.join(pkg, 'urdf', 'pta_robot_diff_drive.urdf')
    default_world = os.path.join(pkg, 'worlds', 'test_room.world')
    default_ekf = os.path.join(pkg, 'config', 'ekf.yaml')
    default_rviz = os.path.join(pkg, 'rviz', 'pta_robot.rviz')

    # Default to the same directory where this launch file lives.
    # If you want logs elsewhere, pass output_dir:=/your/path.
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    default_output_dir = launch_dir
    logger_script = os.path.join(launch_dir, 'odom_test_logger.py')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    controller_name = LaunchConfiguration('controller_name')
    use_ekf = LaunchConfiguration('use_ekf')
    use_rviz = LaunchConfiguration('use_rviz')
    ekf_file = LaunchConfiguration('ekf_file')
    rviz_config = LaunchConfiguration('rviz_config')
    test_phase = LaunchConfiguration('test_phase')

    record_data = LaunchConfiguration('record_data')
    output_dir = LaunchConfiguration('output_dir')
    test_index = LaunchConfiguration('test_index')
    robot_entity_name = LaunchConfiguration('robot_entity_name')
    odom_topic = LaunchConfiguration('odom_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    ground_truth_topic = LaunchConfiguration('ground_truth_topic')
    record_joint_states = LaunchConfiguration('record_joint_states')

    # -------------------------------------------------------------------------
    # Core simulation pieces
    # -------------------------------------------------------------------------
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

    # -------------------------------------------------------------------------
    # ros2_control controllers
    # -------------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ]
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            controller_name,
            '--controller-manager', '/controller_manager',
        ]
    )

    # Bring controllers up only after the robot exists in Gazebo.
    start_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                LogInfo(msg='[sim_diffdrive] Robot spawned, starting joint_state_broadcaster...'),
                joint_state_broadcaster_spawner,
            ],
        )
    )

    start_drive_controller_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                LogInfo(msg='[sim_diffdrive] joint_state_broadcaster ready, starting drive controller...'),
                diff_drive_controller_spawner,
            ],
        )
    )

    # -------------------------------------------------------------------------
    # Optional structured logger for testing
    # -------------------------------------------------------------------------
    # One file per run:
    #   test_1.txt, test_2.txt, test_3.txt, ...
    #
    # This logger records:
    #   - /odom               (odometry under test)
    #   - /gazebo/model_states (ground truth in simulation)
    #   - /cmd_vel            (what we asked the robot to do)
    #   - optional /joint_states
    #
    # Result:
    #   We can later compare commanded motion, estimated motion, and true motion
    #   in a single plain-text file.
    logger_process = ExecuteProcess(
        condition=IfCondition(record_data),
        output='screen',
        cmd=[
            'python3',
            logger_script,
            '--output-dir', output_dir,
            '--test-index', test_index,
            '--test-phase', test_phase,
            '--robot-entity-name', robot_entity_name,
            '--odom-topic', odom_topic,
            '--cmd-vel-topic', cmd_vel_topic,
            '--ground-truth-topic', ground_truth_topic,
            '--controller-name', controller_name,
            '--record-joint-states', record_joint_states,
        ],
    )

    # -------------------------------------------------------------------------
    # Optional tools for later phases
    # -------------------------------------------------------------------------
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[
            ekf_file,
            {'use_sim_time': use_sim_time},
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # -------------------------------------------------------------------------
    # Testing phases (documented structure)
    # -------------------------------------------------------------------------
    # PHASE 1 — WHEEL ODOMETRY ONLY (ACTIVE DEFAULT)
    # -------------------------------------------------------------------------
    # Keep ONLY the diff-drive controller active.
    # Use its raw /odom directly.
    # Recommended launch:
    #   ros2 launch pta_robot_sim sim_diff_drive.launch.py \
    #       test_phase:=wheel use_ekf:=false test_index:=1
    #
    # Validate:
    #   - standstill drift
    #   - straight-line tracking
    #   - in-place rotation
    #   - square loop closure
    #   - /cmd_vel vs /odom consistency
    #   - /odom vs Gazebo ground truth consistency

    # PHASE 2 — D455 ODOMETRY ONLY (COMMENTED TEMPLATE)
    # -------------------------------------------------------------------------
    # Goal:
    #   Disable wheel odom as the primary odom source and test only the D455
    #   odometry pipeline.
    #
    # How to use later:
    #   1) Keep the base controller for motion generation.
    #   2) Remap raw wheel odom to /wheel_odom for comparison only.
    #   3) Publish D455 odom as the only active /odom source under test.
    #
    # Example placeholder:
    # d455_odom_node = Node(
    #     package='<your_d455_pkg>',
    #     executable='<your_d455_odom_exe>',
    #     name='d455_odom',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     remappings=[
    #         ('/camera/odom/sample', '/odom'),
    #     ]
    # )

    # PHASE 3 — T265 ODOMETRY ONLY (COMMENTED TEMPLATE)
    # -------------------------------------------------------------------------
    # Goal:
    #   Disable wheel odom as the primary odom source and test only the T265
    #   tracking odometry pipeline.
    #
    # How to use later:
    #   1) Keep the base controller for motion generation.
    #   2) Remap raw wheel odom to /wheel_odom for comparison only.
    #   3) Publish T265 odom as the only active /odom source under test.
    #
    # Example placeholder:
    # t265_odom_node = Node(
    #     package='<your_t265_pkg>',
    #     executable='<your_t265_odom_exe>',
    #     name='t265_odom',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     remappings=[
    #         ('/camera/odom/sample', '/odom'),
    #     ]
    # )

    # PHASE 4 — EKF FUSION (KEEP OFF UNTIL RAW SOURCES PASS)
    # -------------------------------------------------------------------------
    # Recommended only after PHASE 1 / 2 / 3 look clean.
    #
    # Example:
    #   ros2 launch pta_robot_sim sim_diff_drive.launch.py \
    #       test_phase:=fusion use_ekf:=true test_index:=4
    #
    # Important:
    #   Avoid running EKF and a raw odom TF publisher at the same time unless
    #   you are sure only one node owns odom -> base_footprint.

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Full path to the Gazebo world file'
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
            default_value='0.03',
            description='Spawn z position (small clearance helps reduce bounce)'
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Spawn yaw in radians'
        ),
        DeclareLaunchArgument(
            'controller_name',
            default_value='diff_drive_controller',
            description='ros2_control controller to spawn for diff-drive testing'
        ),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='false',
            description='Start robot_localization EKF (recommended false during raw odom tests)'
        ),
        DeclareLaunchArgument(
            'ekf_file',
            default_value=default_ekf,
            description='Path to EKF yaml file'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='Path to RViz config file'
        ),
        DeclareLaunchArgument(
            'test_phase',
            default_value='wheel',
            description='Documentation-only tag: wheel | d455 | t265 | fusion'
        ),
        DeclareLaunchArgument(
            'record_data',
            default_value='true',
            description='Record structured odom test data to test_<N>.txt'
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value=default_output_dir,
            description='Directory where test_<N>.txt will be written'
        ),
        DeclareLaunchArgument(
            'test_index',
            default_value='1',
            description='Numeric test index used in the output file name: test_<N>.txt'
        ),
        DeclareLaunchArgument(
            'robot_entity_name',
            default_value='pta_robot',
            description='Gazebo entity/model name used when extracting ground truth'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Odometry topic under test'
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/diff_drive_controller/cmd_vel_unstamped',
            description='Commanded velocity topic to record'
        ),
        DeclareLaunchArgument(
            'ground_truth_topic',
            default_value='/gazebo/model_states',
            description='Gazebo topic used as simulation ground truth'
        ),
        DeclareLaunchArgument(
            'record_joint_states',
            default_value='true',
            description='Also log /joint_states for wheel-level debugging'
        ),

        LogInfo(msg=['[sim_diffdrive] test_phase := ', test_phase]),
        LogInfo(msg=['[sim_diffdrive] controller_name := ', controller_name]),
        LogInfo(msg=['[sim_diffdrive] use_ekf := ', use_ekf]),
        LogInfo(msg=['[sim_diffdrive] record_data := ', record_data]),
        LogInfo(msg=['[sim_diffdrive] output_dir := ', output_dir]),
        LogInfo(msg=['[sim_diffdrive] test_index := ', test_index]),

        logger_process,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        start_jsb_after_spawn,
        start_drive_controller_after_jsb,
        ekf_node,
        rviz_node,
    ])