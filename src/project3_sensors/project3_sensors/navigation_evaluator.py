"""Online trajectory comparison with timestamp-nearest ground truth."""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray


TimedPosition = Tuple[float, float, float, float]


class NavigationEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("navigation_evaluator")
        self.declare_parameter("ground_truth_topic", "/ground_truth/odom")
        self.declare_parameter("estimate_topic", "/navigation/odom")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("max_time_difference", 0.08)
        self.declare_parameter("warmup_seconds", 2.0)
        self.declare_parameter("max_points", 20000)

        self.truth: Deque[TimedPosition] = deque(maxlen=1000)
        self.sum_squared_error = 0.0
        self.sum_error = 0.0
        self.max_error = 0.0
        self.count = 0
        self.last_log_time = 0.0
        self.first_estimate_time: Optional[float] = None

        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.truth_path_pub = self.create_publisher(Path, "/trajectory/ground_truth", path_qos)
        self.estimate_path_pub = self.create_publisher(Path, "/trajectory/navigation", path_qos)
        self.metrics_pub = self.create_publisher(
            Float64MultiArray, "/navigation/metrics", 10
        )
        self.truth_path = self._new_path()
        self.estimate_path = self._new_path()
        self.create_subscription(
            Odometry,
            str(self.get_parameter("ground_truth_topic").value),
            self._truth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("estimate_topic").value),
            self._estimate_callback,
            50,
        )
        self.get_logger().info("Navigation evaluator ready (ATE RMSE/mean/max)")

    def _new_path(self) -> Path:
        path = Path()
        path.header.frame_id = str(self.get_parameter("frame_id").value)
        return path

    @staticmethod
    def _time(message: Odometry) -> float:
        return message.header.stamp.sec + 1.0e-9 * message.header.stamp.nanosec

    @staticmethod
    def _position(message: Odometry) -> tuple[float, float, float]:
        position = message.pose.pose.position
        return position.x, position.y, position.z

    def _append_path(self, path: Path, message: Odometry) -> None:
        pose = PoseStamped()
        pose.header = message.header
        pose.header.frame_id = path.header.frame_id
        pose.pose = message.pose.pose
        path.header.stamp = message.header.stamp
        path.poses.append(pose)
        maximum = int(self.get_parameter("max_points").value)
        if len(path.poses) > maximum:
            path.poses = path.poses[-maximum:]

    def _truth_callback(self, message: Odometry) -> None:
        x, y, z = self._position(message)
        self.truth.append((self._time(message), x, y, z))
        self._append_path(self.truth_path, message)
        self.truth_path_pub.publish(self.truth_path)

    def _nearest_truth(self, stamp: float) -> Optional[TimedPosition]:
        if not self.truth:
            return None
        nearest = min(self.truth, key=lambda item: abs(item[0] - stamp))
        if abs(nearest[0] - stamp) > float(
            self.get_parameter("max_time_difference").value
        ):
            return None
        return nearest

    def _estimate_callback(self, message: Odometry) -> None:
        self._append_path(self.estimate_path, message)
        self.estimate_path_pub.publish(self.estimate_path)
        nearest = self._nearest_truth(self._time(message))
        if nearest is None:
            return
        stamp = self._time(message)
        if self.first_estimate_time is None:
            self.first_estimate_time = stamp
        if stamp - self.first_estimate_time < float(
            self.get_parameter("warmup_seconds").value
        ):
            return
        x, y, z = self._position(message)
        error = math.dist((x, y, z), nearest[1:])
        self.count += 1
        self.sum_error += error
        self.sum_squared_error += error * error
        self.max_error = max(self.max_error, error)
        rmse = math.sqrt(self.sum_squared_error / self.count)
        mean = self.sum_error / self.count
        metrics = Float64MultiArray()
        # [current_3d_error, ATE_RMSE, ATE_mean, ATE_max, sample_count]
        metrics.data = [error, rmse, mean, self.max_error, float(self.count)]
        self.metrics_pub.publish(metrics)

        now = self.get_clock().now().nanoseconds * 1.0e-9
        if now - self.last_log_time >= 5.0:
            self.get_logger().info(
                f"ATE samples={self.count}, current={error:.3f} m, "
                f"RMSE={rmse:.3f} m, max={self.max_error:.3f} m"
            )
            self.last_log_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
