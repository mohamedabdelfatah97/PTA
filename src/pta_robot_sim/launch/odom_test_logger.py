#!/usr/bin/env python3
"""
odom_test_logger.py

Simple ROS 2 logger for differential-drive odometry validation in simulation.

Writes one text file per test run in JSON-lines format so it stays:
- human-readable in any text editor
- easy to parse later with Python / grep / jq
- lightweight compared with a rosbag for quick odometry checks

Each line is either:
- a comment/header line starting with '#'
- or a JSON object describing one message/sample

Topics recorded by default:
- /odom                : odometry under test
- /gazebo/model_states : simulation ground truth
- /cmd_vel             : commanded motion

Optional:
- /joint_states        : useful for wheel / encoder debugging
"""

import argparse
import json
import math
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomTestLogger(Node):
    def __init__(
        self,
        output_dir: str,
        test_index: int,
        test_phase: str,
        robot_entity_name: str,
        odom_topic: str,
        cmd_vel_topic: str,
        ground_truth_topic: str,
        record_joint_states: bool,
        controller_name: str,
    ) -> None:
        super().__init__('odom_test_logger')

        self.output_dir = os.path.abspath(output_dir)
        self.test_index = int(test_index)
        self.test_phase = test_phase
        self.robot_entity_name = robot_entity_name
        self.odom_topic = odom_topic
        self.cmd_vel_topic = cmd_vel_topic
        self.ground_truth_topic = ground_truth_topic
        self.record_joint_states = record_joint_states
        self.controller_name = controller_name

        os.makedirs(self.output_dir, exist_ok=True)
        self.output_path = os.path.join(self.output_dir, f'test_{self.test_index}.txt')

        self.model_index: Optional[int] = None
        self.model_lookup_warned = False
        self.start_time_sec = self.now_sec()
        self.counts = {
            'odom': 0,
            'ground_truth': 0,
            'cmd_vel': 0,
            'joint_states': 0,
        }

        self.file_handle = open(self.output_path, 'w', encoding='utf-8')
        self.write_header()

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 50)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 50)
        self.create_subscription(ModelStates, self.ground_truth_topic, self.model_states_callback, 50)

        if self.record_joint_states:
            self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 50)

        self.get_logger().info(f'Writing odometry test log to: {self.output_path}')

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    def write_header(self) -> None:
        self.file_handle.write('# PTA robot simulation odometry test log\n')
        self.file_handle.write('# Format: JSON lines (one sample per line)\n')
        self.file_handle.write('# Recommended comparison pairs: odom <-> ground_truth, cmd_vel <-> odom\n')
        metadata = {
            'type': 'metadata',
            'test_index': self.test_index,
            'test_phase': self.test_phase,
            'robot_entity_name': self.robot_entity_name,
            'controller_name': self.controller_name,
            'odom_topic': self.odom_topic,
            'cmd_vel_topic': self.cmd_vel_topic,
            'ground_truth_topic': self.ground_truth_topic,
            'record_joint_states': self.record_joint_states,
            'log_path': self.output_path,
            'start_time_sim_sec': self.start_time_sec,
        }
        self.write_json(metadata)

    def write_json(self, payload: dict) -> None:
        self.file_handle.write(json.dumps(payload, sort_keys=False) + '\n')
        self.file_handle.flush()

    def odom_callback(self, msg: Odometry) -> None:
        orientation = msg.pose.pose.orientation
        twist = msg.twist.twist
        pose = msg.pose.pose.position

        entry = {
            'type': 'odom',
            'topic': self.odom_topic,
            'sim_time_sec': self.now_sec(),
            'msg_stamp_sec': self.stamp_to_sec(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'child_frame_id': msg.child_frame_id,
            'pose': {
                'x': pose.x,
                'y': pose.y,
                'z': pose.z,
                'yaw': quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w),
                'qx': orientation.x,
                'qy': orientation.y,
                'qz': orientation.z,
                'qw': orientation.w,
            },
            'twist': {
                'vx': twist.linear.x,
                'vy': twist.linear.y,
                'vz': twist.linear.z,
                'wx': twist.angular.x,
                'wy': twist.angular.y,
                'wz': twist.angular.z,
            },
        }
        self.counts['odom'] += 1
        self.write_json(entry)

    def cmd_vel_callback(self, msg: Twist) -> None:
        entry = {
            'type': 'cmd_vel',
            'topic': self.cmd_vel_topic,
            'sim_time_sec': self.now_sec(),
            'twist': {
                'vx': msg.linear.x,
                'vy': msg.linear.y,
                'vz': msg.linear.z,
                'wx': msg.angular.x,
                'wy': msg.angular.y,
                'wz': msg.angular.z,
            },
        }
        self.counts['cmd_vel'] += 1
        self.write_json(entry)

    def model_states_callback(self, msg: ModelStates) -> None:
        if self.model_index is None:
            try:
                self.model_index = msg.name.index(self.robot_entity_name)
                self.get_logger().info(
                    f'Found robot entity "{self.robot_entity_name}" in {self.ground_truth_topic} '
                    f'at index {self.model_index}'
                )
            except ValueError:
                if not self.model_lookup_warned:
                    self.get_logger().warn(
                        f'Robot entity "{self.robot_entity_name}" not found yet in {self.ground_truth_topic}. '
                        'Ground-truth samples will be skipped until it appears.'
                    )
                    self.model_lookup_warned = True
                return

        pose = msg.pose[self.model_index]
        twist = msg.twist[self.model_index]

        entry = {
            'type': 'ground_truth',
            'topic': self.ground_truth_topic,
            'sim_time_sec': self.now_sec(),
            'entity_name': self.robot_entity_name,
            'pose': {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'yaw': quaternion_to_yaw(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
                'qx': pose.orientation.x,
                'qy': pose.orientation.y,
                'qz': pose.orientation.z,
                'qw': pose.orientation.w,
            },
            'twist': {
                'vx': twist.linear.x,
                'vy': twist.linear.y,
                'vz': twist.linear.z,
                'wx': twist.angular.x,
                'wy': twist.angular.y,
                'wz': twist.angular.z,
            },
        }
        self.counts['ground_truth'] += 1
        self.write_json(entry)

    def joint_states_callback(self, msg: JointState) -> None:
        entry = {
            'type': 'joint_states',
            'topic': '/joint_states',
            'sim_time_sec': self.now_sec(),
            'msg_stamp_sec': self.stamp_to_sec(msg.header.stamp),
            'name': list(msg.name),
            'position': list(msg.position),
            'velocity': list(msg.velocity),
            'effort': list(msg.effort),
        }
        self.counts['joint_states'] += 1
        self.write_json(entry)

    def close(self) -> None:
        summary = {
            'type': 'summary',
            'end_time_sim_sec': self.now_sec(),
            'duration_sec': self.now_sec() - self.start_time_sec,
            'sample_counts': self.counts,
            'output_path': self.output_path,
        }
        self.write_json(summary)
        self.file_handle.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Log odometry test data to a structured text file.')
    parser.add_argument('--output-dir', required=True, help='Directory where test_N.txt will be written')
    parser.add_argument('--test-index', default='1', help='Numeric test index used in file naming')
    parser.add_argument('--test-phase', default='wheel', help='Human-readable phase label')
    parser.add_argument('--robot-entity-name', default='pta_robot', help='Gazebo model/entity name')
    parser.add_argument('--odom-topic', default='/odom', help='Odometry topic under test')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel', help='Commanded velocity topic')
    parser.add_argument('--ground-truth-topic', default='/gazebo/model_states', help='Gazebo ground-truth topic')
    parser.add_argument('--record-joint-states', default='false', help='Whether to also log /joint_states (true/false)')
    parser.add_argument('--controller-name', default='diff_drive_controller', help='Controller metadata tag')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()

    node = OdomTestLogger(
        output_dir=args.output_dir,
        test_index=int(args.test_index),
        test_phase=args.test_phase,
        robot_entity_name=args.robot_entity_name,
        odom_topic=args.odom_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        ground_truth_topic=args.ground_truth_topic,
        record_joint_states=str(args.record_joint_states).lower() in ('1', 'true', 'yes', 'on'),
        controller_name=args.controller_name,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
