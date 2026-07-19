#!/usr/bin/env python3
"""Phase 3 - Ground-truth bridge.

Subscribes to /gazebo/model_states (the simulator's absolute truth) and:
    * republishes the TurtleBot3's absolute pose as nav_msgs/Odometry on
      /gt_odom  (this is the reference trajectory for evaluation, and in
      Mode A it is the "perfect estimator" that feeds Nav2);
    * optionally broadcasts the odom -> base_footprint TF from ground truth,
      so that with use_gt:=true the robot is localized perfectly (Mode A) and
      the EKF is bypassed.

Parameters
----------
robot_name      (str)  : model name of the robot in Gazebo. Default 'turtlebot3_waffle'.
odom_frame      (str)  : parent frame for /gt_odom + TF. Default 'odom'.
base_frame      (str)  : child frame. Default 'base_footprint'.
publish_tf      (bool) : broadcast odom->base_footprint from GT. Default True.
publish_rate_hz (float): max republish rate (throttles model_states). Default 50.0.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class GtBridgeNode(Node):
    def __init__(self):
        super().__init__('gt_bridge_node')

        self.declare_parameter('robot_name', 'turtlebot3_waffle')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate_hz', 50.0)

        self.robot_name = self.get_parameter('robot_name').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._min_period = 1.0 / rate if rate > 0.0 else 0.0
        self._last_pub_time = None

        self.odom_pub = self.create_publisher(Odometry, 'gt_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        # /gazebo/model_states is best-effort (volatile) in gazebo_ros.
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.on_model_states, qos)

        self._warned_missing = False
        self.get_logger().info(
            f"gt_bridge_node up: tracking model '{self.robot_name}', "
            f"publishing /gt_odom (publish_tf={self.publish_tf}).")

    def on_model_states(self, msg: ModelStates):
        try:
            idx = msg.name.index(self.robot_name)
        except ValueError:
            if not self._warned_missing:
                self.get_logger().warn(
                    f"Model '{self.robot_name}' not found in /gazebo/model_states. "
                    f"Available: {list(msg.name)}")
                self._warned_missing = True
            return

        now = self.get_clock().now()
        # Throttle to publish_rate_hz.
        if self._min_period > 0.0 and self._last_pub_time is not None:
            dt = (now - self._last_pub_time).nanoseconds * 1e-9
            if dt < self._min_period:
                return
        self._last_pub_time = now

        pose = msg.pose[idx]
        twist = msg.twist[idx]

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = pose
        odom.twist.twist = twist
        # Ground truth: near-zero covariance on the diagonal.
        odom.pose.covariance[0] = 1e-6
        odom.pose.covariance[7] = 1e-6
        odom.pose.covariance[14] = 1e-6
        odom.pose.covariance[21] = 1e-6
        odom.pose.covariance[28] = 1e-6
        odom.pose.covariance[35] = 1e-6
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation = pose.orientation
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = GtBridgeNode()
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
