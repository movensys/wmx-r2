#!/usr/bin/env python3
# Copyright 2026 Movensys Corporation.
# Licensed under the MIT License. See LICENSE.txt for details.
#
# capture_stream_trace
#
# Records the three positions that matter when characterising the servo
# streaming path, on one common clock, and writes them as CSV:
#
#   setpoint    /movensys_manipulator_arm_controller/joint_trajectory  (40 Hz)
#               what the streamer was ASKED to do
#   pos_cmd     /wmx/axes/status.pos_cmd                               (100 Hz)
#               what WMX3's interpolator actually generated
#   actual_pos  /wmx/axes/status.actual_pos                            (100 Hz)
#               what the arm physically did
#
# The split matters. setpoint -> pos_cmd is the API buffer and interpolation
# path; pos_cmd -> actual_pos is the servo loop underneath it. Comparing only
# setpoint against actual_pos cannot tell you which of the two a tracking error
# came from, which is why the earlier surge measurement could not be resolved.
#
# Timestamps are message header stamps, not receipt times, so the two topics
# share a timebase even though they arrive on different threads.
#
# Usage:
#   ros2 run wmx_r2_package capture_stream_trace.py --seconds 30 --out trace.csv
# or plain python3 with the workspace sourced.

import argparse
import csv

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory
from wmx_r2_message.msg import AxesStatus


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class StreamTraceCapture(Node):
    def __init__(self, args):
        super().__init__('capture_stream_trace')
        self.axes = args.axes
        self.setpoints = []      # (t, [pos per axis])
        self.states = []         # (t, [pos_cmd], [actual_pos])

        self.create_subscription(
            JointTrajectory, args.trajectory_topic, self.on_trajectory, 50)
        self.create_subscription(
            AxesStatus, args.axis_state_topic, self.on_axis_state, 50)

        self.get_logger().info(
            f'capturing {args.seconds}s from {args.trajectory_topic} '
            f'and {args.axis_state_topic}')

    def on_trajectory(self, msg):
        if not msg.points:
            return
        # Last point only, matching what servo_stream_controller consumes.
        pos = list(msg.points[-1].positions)
        self.setpoints.append((stamp_to_sec(msg.header.stamp), pos))

    def on_axis_state(self, msg):
        self.states.append((
            stamp_to_sec(msg.header.stamp),
            list(msg.pos_cmd),
            list(msg.actual_pos)))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--seconds', type=float, default=30.0)
    p.add_argument('--out', default='stream_trace.csv')
    p.add_argument('--axes', type=int, default=6)
    p.add_argument(
        '--trajectory-topic',
        default='/movensys_manipulator_arm_controller/joint_trajectory')
    p.add_argument('--axis-state-topic', default='/wmx/axes/status')
    args = p.parse_args()

    rclpy.init()
    node = StreamTraceCapture(args)
    start = node.get_clock().now().nanoseconds * 1e-9
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds * 1e-9 - start >= args.seconds:
            break

    n = args.axes
    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['source', 't'] + [f'j{i + 1}' for i in range(n)])
        for t, pos in node.setpoints:
            w.writerow(['setpoint', f'{t:.6f}'] + [f'{v:.9f}' for v in pos[:n]])
        for t, cmd, act in node.states:
            w.writerow(['pos_cmd', f'{t:.6f}'] + [f'{v:.9f}' for v in cmd[:n]])
            w.writerow(['actual_pos', f'{t:.6f}'] + [f'{v:.9f}' for v in act[:n]])

    node.get_logger().info(
        f'wrote {args.out}: {len(node.setpoints)} setpoints, '
        f'{len(node.states)} axis-state frames')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
