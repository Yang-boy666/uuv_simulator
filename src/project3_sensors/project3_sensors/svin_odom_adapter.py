"""Align SVIn's local origin with the configured simulation/mission start pose."""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from okvis_ros.msg import SvinHealth
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from .math_utils import (
    normalize_quaternion,
    quaternion_conjugate,
    quaternion_multiply,
    rotate_vector,
)


class SvinOdomAdapter(Node):
    def __init__(self) -> None:
        super().__init__("svin_odom_adapter")
        self.declare_parameter("input_topic", "/okvis_odometry")
        self.declare_parameter("output_topic", "/svin/aligned_odom")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("body_frame", "pipeline_uuv/base_link")
        self.declare_parameter("health_topic", "/svin_health")
        self.declare_parameter("require_tracking_health", True)
        self.declare_parameter("fusion_reference_topic", "/navigation/dead_reckoning")
        self.declare_parameter("reacquisition_gate_distance", 0.35)
        self.declare_parameter("innovation_window_seconds", 10.0)
        self.declare_parameter("reference_timeout", 0.6)
        self.declare_parameter("max_visual_speed", 1.0)
        self.declare_parameter("visual_jump_margin", 0.12)
        self.declare_parameter("max_unreferenced_gap", 2.0)
        self.declare_parameter("initial_x", -8.5)
        self.declare_parameter("initial_y", -5.0)
        self.declare_parameter("initial_z", -4.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("max_path_points", 20000)

        self.initial_position = np.array(
            [
                self.get_parameter("initial_x").value,
                self.get_parameter("initial_y").value,
                self.get_parameter("initial_z").value,
            ],
            dtype=float,
        )
        yaw = float(self.get_parameter("initial_yaw").value)
        self.initial_orientation = normalize_quaternion(
            (0.0, 0.0, np.sin(0.5 * yaw), np.cos(0.5 * yaw))
        )
        self.origin_position: Optional[np.ndarray] = None
        self.origin_orientation = (0.0, 0.0, 0.0, 1.0)
        self.health_received = False
        self.raw_tracking_ok = False
        self.output_tracking_ok = False
        self.ever_accepted = False
        self.reference_gate_armed = False
        self.latest_reference_position: Optional[np.ndarray] = None
        self.latest_reference_arrival: Optional[float] = None
        self.reference_anchor: Optional[np.ndarray] = None
        self.visual_anchor: Optional[np.ndarray] = None
        self.anchor_stamp: Optional[float] = None
        self.innovation_rejected = False
        self.last_accepted_position: Optional[np.ndarray] = None
        self.last_accepted_stamp: Optional[float] = None

        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 20
        )
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "/trajectory/svin", path_qos)
        self.tracking_pub = self.create_publisher(Bool, "/svin/tracking_ok", 10)
        self.path = Path()
        self.path.header.frame_id = str(self.get_parameter("world_frame").value)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_topic").value),
            self._callback,
            50,
        )
        self.create_subscription(
            SvinHealth,
            str(self.get_parameter("health_topic").value),
            self._health_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("fusion_reference_topic").value),
            self._reference_callback,
            20,
        )
        self.get_logger().info(
            "SVIn origin adapter ready: local SVIn origin -> configured mission start"
        )

    def _health_callback(self, message: SvinHealth) -> None:
        previous = self.raw_tracking_ok
        self.health_received = True
        self.raw_tracking_ok = bool(message.is_tracking_ok)
        if not self.raw_tracking_ok:
            if previous or self.output_tracking_ok:
                self.get_logger().warning(
                    "SVIn tracking lost; visual pose rejected, DVL/IMU/depth continue"
                )
            self._publish_tracking_status(False)
        elif not previous:
            self.get_logger().info(
                "SVIn reports tracking recovery; validating against fused navigation"
            )

    def _publish_tracking_status(self, accepted: bool) -> None:
        self.output_tracking_ok = accepted
        status = Bool()
        status.data = accepted
        self.tracking_pub.publish(status)

    def _reference_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.latest_reference_position = np.array(
            [position.x, position.y, position.z], dtype=float
        )
        self.latest_reference_arrival = self.get_clock().now().nanoseconds * 1.0e-9

    @staticmethod
    def _quaternion(message) -> tuple[float, float, float, float]:
        return normalize_quaternion((message.x, message.y, message.z, message.w))

    def _callback(self, message: Odometry) -> None:
        source_position = np.array(
            [
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            ],
            dtype=float,
        )
        source_orientation = self._quaternion(message.pose.pose.orientation)
        if self.origin_position is None:
            self.origin_position = source_position.copy()
            self.origin_orientation = source_orientation
            self.get_logger().info("Received first SVIn odometry; alignment initialized")

        if bool(self.get_parameter("require_tracking_health").value) and (
            not self.health_received or not self.raw_tracking_ok
        ):
            return

        q_alignment = quaternion_multiply(
            self.initial_orientation, quaternion_conjugate(self.origin_orientation)
        )
        relative_position = source_position - self.origin_position
        aligned_position = self.initial_position + rotate_vector(q_alignment, relative_position)
        aligned_orientation = quaternion_multiply(q_alignment, source_orientation)

        stamp = message.header.stamp.sec + 1.0e-9 * message.header.stamp.nanosec
        now = self.get_clock().now().nanoseconds * 1.0e-9
        reference_fresh = (
            self.latest_reference_position is not None
            and self.latest_reference_arrival is not None
            and now - self.latest_reference_arrival
            <= float(self.get_parameter("reference_timeout").value)
        )
        if self.ever_accepted and reference_fresh:
            gate = float(self.get_parameter("reacquisition_gate_distance").value)
            assert self.latest_reference_position is not None
            absolute_innovation = float(
                np.linalg.norm(aligned_position - self.latest_reference_position)
            )
            # Arm only after both independently initialized trajectories agree.
            if not self.reference_gate_armed and absolute_innovation <= gate:
                self.reference_gate_armed = True
                self.visual_anchor = aligned_position.copy()
                self.reference_anchor = self.latest_reference_position.copy()
                self.anchor_stamp = stamp

            relative_innovation = 0.0
            if (
                self.reference_gate_armed
                and self.visual_anchor is not None
                and self.reference_anchor is not None
            ):
                visual_delta = aligned_position - self.visual_anchor
                reference_delta = self.latest_reference_position - self.reference_anchor
                relative_innovation = float(np.linalg.norm(visual_delta - reference_delta))

            if self.reference_gate_armed and relative_innovation > gate:
                self._publish_tracking_status(False)
                if not self.innovation_rejected:
                    self.get_logger().warning(
                        f"SVIn pose rejected: independent DR innovation "
                        f"{relative_innovation:.2f} m > {gate:.2f} m"
                    )
                self.innovation_rejected = True
                return

            # Sliding anchors compare local motion instead of accumulated drift.
            window = float(self.get_parameter("innovation_window_seconds").value)
            if (
                self.reference_gate_armed
                and not self.innovation_rejected
                and self.anchor_stamp is not None
                and stamp - self.anchor_stamp >= window
            ):
                self.visual_anchor = aligned_position.copy()
                self.reference_anchor = self.latest_reference_position.copy()
                self.anchor_stamp = stamp

        elif self.ever_accepted and self.last_accepted_position is not None:
            # Never compare visual odometry against a frozen DVL trajectory.
            # During bottom-lock loss use a local kinematic continuity gate;
            # long outages wait for an independent reference to return.
            assert self.last_accepted_stamp is not None
            dt = stamp - self.last_accepted_stamp
            maximum_gap = float(self.get_parameter("max_unreferenced_gap").value)
            displacement = float(np.linalg.norm(aligned_position - self.last_accepted_position))
            allowed = (
                float(self.get_parameter("visual_jump_margin").value)
                + float(self.get_parameter("max_visual_speed").value) * max(0.0, dt)
            )
            if dt <= 0.0 or dt > maximum_gap or displacement > allowed:
                self._publish_tracking_status(False)
                if not self.innovation_rejected:
                    self.get_logger().warning(
                        f"SVIn continuity rejected without fresh DVL reference: "
                        f"jump={displacement:.2f} m, dt={dt:.2f} s"
                    )
                self.innovation_rejected = True
                return

        if not self.output_tracking_ok:
            self.get_logger().info("SVIn visual pose passed health/innovation gates")
        self.innovation_rejected = False
        self.ever_accepted = True
        self.last_accepted_position = aligned_position.copy()
        self.last_accepted_stamp = stamp
        self._publish_tracking_status(True)

        output = Odometry()
        output.header.stamp = message.header.stamp
        output.header.frame_id = str(self.get_parameter("world_frame").value)
        output.child_frame_id = str(self.get_parameter("body_frame").value)
        output.pose.pose.position.x = float(aligned_position[0])
        output.pose.pose.position.y = float(aligned_position[1])
        output.pose.pose.position.z = float(aligned_position[2])
        output.pose.pose.orientation.x = aligned_orientation[0]
        output.pose.pose.orientation.y = aligned_orientation[1]
        output.pose.pose.orientation.z = aligned_orientation[2]
        output.pose.pose.orientation.w = aligned_orientation[3]
        output.pose.covariance = list(message.pose.covariance)
        if max(abs(value) for value in output.pose.covariance) < 1.0e-12:
            output.pose.covariance[0] = 0.04
            output.pose.covariance[7] = 0.04
            output.pose.covariance[14] = 0.09
            output.pose.covariance[21] = 0.01
            output.pose.covariance[28] = 0.01
            output.pose.covariance[35] = 0.02
        output.twist = message.twist
        self.odom_pub.publish(output)

        pose = PoseStamped()
        pose.header = output.header
        pose.pose = output.pose.pose
        self.path.header.stamp = output.header.stamp
        self.path.poses.append(pose)
        maximum = int(self.get_parameter("max_path_points").value)
        if len(self.path.poses) > maximum:
            self.path.poses = self.path.poses[-maximum:]
        self.path_pub.publish(self.path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SvinOdomAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
