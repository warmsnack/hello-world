#!/usr/bin/env python3
"""Phase 4 - Automatic goal dispatcher.

A Nav2 NavigateToPose action client that, instead of a manual RViz "2D Goal
Pose", automatically sends a fixed straight-ahead goal (default: 5 m forward)
so both experiment modes (A / ground truth, B / EKF) are driven identically.

Parameters
----------
goal_x   (float) : goal x in map frame. Default 5.0.
goal_y   (float) : goal y in map frame. Default 0.0.
goal_yaw (float) : goal yaw [rad]. Default 0.0.
frame_id (str)   : goal frame. Default 'map'.
delay_s  (float) : wait before sending (let the stack settle). Default 5.0.
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GoalPoseClient(Node):
    def __init__(self):
        super().__init__('goal_pose_client')

        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('delay_s', 5.0)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_yaw = float(self.get_parameter('goal_yaw').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.delay_s = float(self.get_parameter('delay_s').value)

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._sent = False
        self._timer = self.create_timer(self.delay_s, self._send_goal_once)
        self.get_logger().info(
            f"goal_pose_client will send ({self.goal_x}, {self.goal_y}) "
            f"in frame '{self.frame_id}' after {self.delay_s}s.")

    def _send_goal_once(self):
        if self._sent:
            return
        self._sent = True
        self._timer.cancel()

        self.get_logger().info('Waiting for navigate_to_pose action server...')
        if not self._client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('navigate_to_pose server not available.')
            return

        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = self.goal_x
        ps.pose.position.y = self.goal_y
        qx, qy, qz, qw = yaw_to_quat(self.goal_yaw)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        goal.pose = ps

        self.get_logger().info(
            f'Sending goal x={self.goal_x} y={self.goal_y} yaw={self.goal_yaw}')
        future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'distance_remaining={remaining:.2f} m',
                               throttle_duration_sec=2.0)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the server.')
            return
        self.get_logger().info('Goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status
        self.get_logger().info(f'Navigation finished with status={status}.')


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseClient()
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
