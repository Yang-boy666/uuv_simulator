"""Publish actual and nominal pipeline trajectories as RViz Path messages."""

from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions


DEFAULT_REFERENCE = [
    -8.5, -5.0, -4.0,
    -3.0, -5.0, -4.0,
    0.0, -2.0, -4.0,
    0.0, 4.0, -4.0,
    3.0, 7.0, -4.0,
    8.5, 7.0, -4.0,
]


class TrajectoryPublisher(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_publisher")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("min_point_distance", 0.03)
        self.declare_parameter("max_points", 20000)
        self.declare_parameter("reference_path_xyz", DEFAULT_REFERENCE)

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.actual_pub = self.create_publisher(Path, "trajectory/actual", latched_qos)
        self.reference_pub = self.create_publisher(
            Path, "trajectory/reference", latched_qos
        )
        self.actual_path = Path()
        self.actual_path.header.frame_id = str(self.get_parameter("frame_id").value)
        self.last_position: Optional[tuple[float, float, float]] = None
        self.create_subscription(Odometry, "odom", self._odom_callback, 20)
        self.reference_path = self._make_reference_path()
        self.timer = self.create_timer(1.0, self._republish)
        self.reference_pub.publish(self.reference_path)
        self.get_logger().info("Trajectory publisher ready")

    def _make_reference_path(self) -> Path:
        values = list(self.get_parameter("reference_path_xyz").value)
        if len(values) % 3 != 0:
            raise ValueError("reference_path_xyz length must be divisible by three")
        path = Path()
        path.header.frame_id = str(self.get_parameter("frame_id").value)
        path.header.stamp = self.get_clock().now().to_msg()
        for index in range(0, len(values), 3):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(values[index])
            pose.pose.position.y = float(values[index + 1])
            pose.pose.position.z = float(values[index + 2])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        current = (position.x, position.y, position.z)
        minimum = float(self.get_parameter("min_point_distance").value)
        if self.last_position is not None and math.dist(current, self.last_position) < minimum:
            return
        pose = PoseStamped()
        pose.header.stamp = message.header.stamp
        pose.header.frame_id = self.actual_path.header.frame_id
        pose.pose = message.pose.pose
        self.actual_path.poses.append(pose)
        maximum = int(self.get_parameter("max_points").value)
        if len(self.actual_path.poses) > maximum:
            self.actual_path.poses = self.actual_path.poses[-maximum:]
        self.last_position = current
        self.actual_path.header.stamp = message.header.stamp
        self.actual_pub.publish(self.actual_path)

    def _republish(self) -> None:
        now = self.get_clock().now().to_msg()
        self.reference_path.header.stamp = now
        for pose in self.reference_path.poses:
            pose.header.stamp = now
        self.reference_pub.publish(self.reference_path)
        if self.actual_path.poses:
            self.actual_pub.publish(self.actual_path)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
