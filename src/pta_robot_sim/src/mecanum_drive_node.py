#!/usr/bin/env python3
"""
mecanum_drive_node.py — PTA Robot Mecanum Kinematics

Converts /cmd_vel body velocity commands to individual wheel velocity commands
and publishes raw wheel odometry on /odom.

Key design decisions (following ChatGPT review):
  - /cmd_vel callback ONLY stores the latest desired velocity — does NOT publish
  - A 50 Hz timer drives all wheel command publishing every cycle
  - If no fresh /cmd_vel arrives within 0.25s, wheel commands are zeroed
  - Zero wheel commands published on node shutdown
  - Wheel speeds clamped to MAX_WHEEL_VEL for safety

Wheel layout (top view):
    F_L (+x,+y) ---- F_R (+x,-y)
         |                  |
         |                  |
    R_L (-x,+y) ---- R_R (-x,-y)

Robot geometry (from URDF):
    wheel_radius : 0.0508 m
    lx           : 0.34 m   (half wheelbase, front-rear)
    ly           : 0.1775 m (half track width, left-right)
    k = lx + ly  : 0.5175 m (mecanum geometry factor)

Mecanum inverse kinematics (cmd_vel -> wheel velocities):
    w_FL = (vx - vy - k * wz) / r
    w_FR = (vx + vy + k * wz) / r
    w_RL = (vx + vy - k * wz) / r
    w_RR = (vx - vy + k * wz) / r

Mecanum forward kinematics (wheel velocities -> body velocity):
    vx = r/4 * ( w_FL + w_FR + w_RL + w_RR)
    vy = r/4 * (-w_FL + w_FR + w_RL - w_RR)
    wz = r/(4k) * (-w_FL + w_FR - w_RL + w_RR)

Topics:
    Subscribes: /cmd_vel (geometry_msgs/Twist)
    Subscribes: /joint_states (sensor_msgs/JointState)
    Publishes:  /wheel_velocity_controller/commands (std_msgs/Float64MultiArray)
                [F_L, F_R, R_L, R_R] — order matches ros2_controllers.yaml
    Publishes:  /odom (nav_msgs/Odometry) — raw wheel odometry, no TF
                TF odom->base_footprint published by robot_localization EKF
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# ── Robot geometry ──────────────────────────────────────────────────────────
WHEEL_RADIUS  = 0.0508   # m
LX            = 0.34     # m — half wheelbase (front-rear)
LY            = 0.1775   # m — half track width (left-right)
K             = LX + LY  # 0.5175 m

# Safety clamp — max wheel angular velocity (rad/s)
MAX_WHEEL_VEL = 10.0

# Control loop frequency and command timeout
CONTROL_HZ    = 50.0    # Hz — wheel command publish rate
CMD_TIMEOUT_S = 0.25    # s  — zero wheels if no fresh cmd_vel

# Joint order MUST match ros2_controllers.yaml
JOINT_NAMES = [
    'F_L_Wheel_Joint',
    'F_R_Wheel_Joint',
    'R_L_Wheel_Joint',
    'R_R_Wheel_Joint',
]


class MecanumDriveNode(Node):

    def __init__(self):
        super().__init__('mecanum_drive_node')

        # ── Publishers ──────────────────────────────────────────────────────
        self.wheel_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocity_controller/commands',
            10,
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10,
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10,
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
        )

        # ── Desired body velocity (set by cmd_vel callback, read by timer) ──
        self._des_vx = 0.0
        self._des_vy = 0.0
        self._des_wz = 0.0
        self._last_cmd_time: Time = self.get_clock().now()

        # ── Odometry state ───────────────────────────────────────────────────
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0
        self._last_odom_time = None

        # Latest wheel velocities indexed by joint name
        self._wheel_vel = {n: 0.0 for n in JOINT_NAMES}

        # ── 50 Hz control timer ──────────────────────────────────────────────
        self._control_timer = self.create_timer(
            1.0 / CONTROL_HZ,
            self._control_loop,
        )

        self.get_logger().info(
            f'MecanumDriveNode started — '
            f'r={WHEEL_RADIUS} lx={LX} ly={LY} k={K} '
            f'loop={CONTROL_HZ}Hz timeout={CMD_TIMEOUT_S}s'
        )

    # ── cmd_vel callback — ONLY stores, does NOT publish ────────────────────
    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._des_vx = msg.linear.x
        self._des_vy = msg.linear.y
        self._des_wz = msg.angular.z
        self._last_cmd_time = self.get_clock().now()

    # ── 50 Hz control loop — drives all wheel command publishing ─────────────
    def _control_loop(self) -> None:
        now = self.get_clock().now()
        age = (now - self._last_cmd_time).nanoseconds * 1e-9
        if age > CMD_TIMEOUT_S:
            vx, vy, wz = 0.0, 0.0, 0.0
        else:
            vx, vy, wz = self._des_vx, self._des_vy, self._des_wz
        self._publish_wheel_commands(vx, vy, wz)

    def _publish_wheel_commands(self, vx: float, vy: float, wz: float) -> None:
        r = WHEEL_RADIUS
        k = K

        w_fl = (vx - vy - k * wz) / r
        w_fr = (vx + vy + k * wz) / r
        w_rl = (vx + vy - k * wz) / r
        w_rr = (vx - vy + k * wz) / r

        def clamp(v):
            return max(-MAX_WHEEL_VEL, min(MAX_WHEEL_VEL, v))

        cmd = Float64MultiArray()
        cmd.data = [clamp(w_fl), clamp(w_fr), clamp(w_rl), clamp(w_rr)]
        self.wheel_cmd_pub.publish(cmd)

    # ── joint_states callback — computes and publishes odometry ──────────────
    def _joint_state_callback(self, msg: JointState) -> None:
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        for joint in JOINT_NAMES:
            if joint in name_to_idx and msg.velocity:
                self._wheel_vel[joint] = msg.velocity[name_to_idx[joint]]

        w_fl = self._wheel_vel['F_L_Wheel_Joint']
        w_fr = self._wheel_vel['F_R_Wheel_Joint']
        w_rl = self._wheel_vel['R_L_Wheel_Joint']
        w_rr = self._wheel_vel['R_R_Wheel_Joint']

        r = WHEEL_RADIUS
        k = K

        vx = (r / 4.0) * ( w_fl + w_fr + w_rl + w_rr)
        vy = (r / 4.0) * (-w_fl + w_fr + w_rl - w_rr)
        wz = (r / (4.0 * k)) * (-w_fl + w_fr - w_rl + w_rr)

        now = self.get_clock().now()
        if self._last_odom_time is None:
            self._last_odom_time = now
            return

        dt = (now - self._last_odom_time).nanoseconds * 1e-9
        self._last_odom_time = now

        if dt <= 0.0 or dt > 0.5:
            return

        cos_th = math.cos(self._theta)
        sin_th = math.sin(self._theta)
        self._x     += (vx * cos_th - vy * sin_th) * dt
        self._y     += (vx * sin_th + vy * cos_th) * dt
        self._theta += wz * dt

        # Publish odometry — NO TF (EKF handles odom->base_footprint TF)
        qz = math.sin(self._theta / 2.0)
        qw = math.cos(self._theta / 2.0)

        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'

        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = wz

        odom.pose.covariance[0]  = 0.01
        odom.pose.covariance[7]  = 0.01
        odom.pose.covariance[35] = 0.03
        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[7]  = 0.01
        odom.twist.covariance[35] = 0.03

        self.odom_pub.publish(odom)

    # ── Shutdown — publish zero wheel commands before exit ───────────────────
    def destroy_node(self):
        self.get_logger().info('MecanumDriveNode shutting down — zeroing wheels')
        self._publish_wheel_commands(0.0, 0.0, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MecanumDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
