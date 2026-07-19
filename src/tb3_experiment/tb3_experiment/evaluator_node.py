#!/usr/bin/env python3
"""Phase 3 - Clearance evaluator.

Subscribes to /gazebo/model_states and, at a fixed rate, computes the Euclidean
distance (in the XY plane) between the robot centre and every dynamic obstacle.
The per-timestep clearance, the instantaneous minimum, and the running global
minimum are appended to a CSV file for offline analysis (Phase 5).

CSV columns:
    stamp_sec, robot_x, robot_y, <name>_dist ..., min_clearance, global_min_clearance

Parameters
----------
robot_name          (str)       : robot model name. Default 'turtlebot3_waffle'.
obstacle_names      (str list)  : dynamic obstacle model names.
                                  Default ['pedestrian', 'moving_cylinder'].
rate_hz             (float)     : sampling / logging rate. Default 10.0.
output_csv          (str)       : CSV path. Default '~/tb3_eval/clearance_<mode>.csv'.
mode_label          (str)       : label appended to default filename ('A'/'B').
"""

import os
import csv
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from gazebo_msgs.msg import ModelStates


class EvaluatorNode(Node):
    def __init__(self):
        super().__init__('evaluator_node')

        self.declare_parameter('robot_name', 'turtlebot3_waffle')
        self.declare_parameter('obstacle_names', ['pedestrian', 'moving_cylinder'])
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('mode_label', 'B')
        self.declare_parameter('output_csv', '')

        self.robot_name = self.get_parameter('robot_name').value
        self.obstacle_names = list(self.get_parameter('obstacle_names').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.mode_label = self.get_parameter('mode_label').value

        out = self.get_parameter('output_csv').value
        if not out:
            out = os.path.expanduser(
                f'~/tb3_eval/clearance_{self.mode_label}.csv')
        self.output_csv = out
        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)

        self._latest = None  # (robot_xy, {name: (x, y)})
        self._global_min = math.inf

        self._csv_file = open(self.output_csv, 'w', newline='')
        self._writer = csv.writer(self._csv_file)
        header = ['stamp_sec', 'robot_x', 'robot_y']
        header += [f'{n}_dist' for n in self.obstacle_names]
        header += ['min_clearance', 'global_min_clearance']
        self._writer.writerow(header)
        self._csv_file.flush()

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.on_model_states, qos)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)
        self.get_logger().info(
            f"evaluator_node up: mode={self.mode_label}, rate={self.rate_hz} Hz, "
            f"obstacles={self.obstacle_names}, csv='{self.output_csv}'.")

    def on_model_states(self, msg: ModelStates):
        name_to_pose = dict(zip(msg.name, msg.pose))
        if self.robot_name not in name_to_pose:
            return
        rp = name_to_pose[self.robot_name].position
        obstacles = {}
        for n in self.obstacle_names:
            if n in name_to_pose:
                p = name_to_pose[n].position
                obstacles[n] = (p.x, p.y)
        self._latest = ((rp.x, rp.y), obstacles)

    def on_timer(self):
        if self._latest is None:
            return
        (rx, ry), obstacles = self._latest

        dists = []
        row_dists = []
        for n in self.obstacle_names:
            if n in obstacles:
                ox, oy = obstacles[n]
                d = math.hypot(rx - ox, ry - oy)
                dists.append(d)
                row_dists.append(f'{d:.4f}')
            else:
                row_dists.append('')

        min_clearance = min(dists) if dists else math.inf
        if min_clearance < self._global_min:
            self._global_min = min_clearance

        stamp = self.get_clock().now().nanoseconds * 1e-9
        row = [f'{stamp:.4f}', f'{rx:.4f}', f'{ry:.4f}']
        row += row_dists
        row += [
            f'{min_clearance:.4f}' if math.isfinite(min_clearance) else '',
            f'{self._global_min:.4f}' if math.isfinite(self._global_min) else '',
        ]
        self._writer.writerow(row)
        self._csv_file.flush()

    def destroy_node(self):
        try:
            if math.isfinite(self._global_min):
                self.get_logger().info(
                    f"[mode {self.mode_label}] Minimum clearance over run: "
                    f"{self._global_min:.3f} m")
            if not self._csv_file.closed:
                self._csv_file.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
