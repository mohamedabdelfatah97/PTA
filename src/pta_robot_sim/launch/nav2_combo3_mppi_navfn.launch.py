"""
nav2_combo3_mppi_navfn.launch.py — dedicated Combo 3 bringup

Purpose:
  Use one clean Combo 3 path without the manual activate_nav2.sh step.

Backend:
  /cmd_vel_smoothed -> gazebo_ros_planar_move or robot motion plugin -> body motion
  /odom -> ekf_holonomic.yaml -> /odometry/filtered
  Nav2 controller -> velocity_smoother -> /cmd_vel

Notes:
  - mecanum_drive_controller is spawned for wheel joint visualization.
  - AMCL uses the merged /scan from dual_laser_merger.
  - Local costmap should use /scan_front + /d455_depth/points from combo3_mppi_navfn.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _as_bool(value):
    return str(value).lower() in ("true", "1", "yes", "y", "on")


def _check_file(label, path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"{label} not found: {path}")


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("pta_robot_sim")
    gazebo_ros_pkg = get_package_share_directory("gazebo_ros")
    nav2_bringup = get_package_share_directory("nav2_bringup")

    world_file = os.path.join(pkg, "worlds", "test_room.world")
    urdf_file = os.path.join(pkg, "urdf", "pta_robot_mecanum_drive.urdf")
    map_file = os.path.join(pkg, "maps", "test_room.yaml")
    nav2_params = os.path.join(pkg, "config", "nav2", "combo3_mppi_navfn.yaml")
    ekf_params = os.path.join(pkg, "config", "ekf_holonomic.yaml")
    rviz_config = os.path.join(pkg, "rviz", "pta_robot.rviz")

    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time_bool = _as_bool(use_sim_time_str)

    use_rviz_str = LaunchConfiguration("use_rviz").perform(context)
    use_rviz_bool = _as_bool(use_rviz_str)

    _check_file("World file", world_file)
    _check_file("URDF file", urdf_file)
    _check_file("Map file", map_file)
    _check_file("Nav2 params file", nav2_params)
    _check_file("EKF params file", ekf_params)

    with open(urdf_file, "r") as f:
        robot_description = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_file,
            "verbose": "false",
        }.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time_bool,
        }],
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_params,
            {"use_sim_time": use_sim_time_bool},
        ],
    )

    laser_merger = Node(
        package="dual_laser_merger",
        executable="dual_laser_merger_node",
        name="dual_laser_merger",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time_bool,
            "laser_1_topic": "/scan_front",
            "laser_2_topic": "/scan_rear",
            "merged_scan_topic": "/scan",
            "target_frame": "base_link",
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "range_min": 0.15,
            "range_max": 12.0,
        }],
    )

    spawn_robot = TimerAction(
        period=20.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                output="screen",
                arguments=[
                    "-topic", "robot_description",
                    "-entity", "pta_robot",
                    "-x", "0.65",
                    "-y", "1.385",
                    "-z", "0.03",
                    "-Y", "0.0",
                ],
            )
        ],
    )

    spawn_jsb = TimerAction(
        period=25.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
            )
        ],
    )

    spawn_mecanum = TimerAction(
        period=27.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "mecanum_drive_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
            )
        ],
    )

    rviz = TimerAction(
        period=45.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time_bool}],
            )
        ],
    )

    localization = TimerAction(
        period=50.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup, "launch", "localization_launch.py")
                ),
                launch_arguments={
                    "map": map_file,
                    "use_sim_time": use_sim_time_str,
                    "params_file": nav2_params,
                    "autostart": "true",
                }.items(),
            )
        ],
    )

    navigation = TimerAction(
        period=65.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time_str,
                    "params_file": nav2_params,
                    "autostart": "true",
                }.items(),
            )
        ],
    )

    actions = [
        gazebo,
        rsp,
        ekf,
        laser_merger,
        spawn_robot,
        spawn_jsb,
        spawn_mecanum,
    ]

    if use_rviz_bool:
        actions.append(rviz)

    actions.extend([
        localization,
        navigation,
    ])

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])