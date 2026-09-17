"""Generate DVL and pressure-depth measurements from Gazebo ground truth.

Ground truth is intentionally confined to this sensor model and the evaluator.
Navigation nodes only consume the noisy standard sensor topics.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, LaserScan
from std_msgs.msg import Bool, Float64, Float64MultiArray

from .math_utils import quaternion_conjugate, rotate_vector


class SensorSimulator(Node):
    def __init__(self) -> None:
        super().__init__("sensor_simulator")
        defaults = {
            "ground_truth_topic": "/pipeline_uuv/odom",
            "ground_truth_output_topic": "/ground_truth/odom",
            "body_frame": "pipeline_uuv/base_link",
            "world_frame": "world",
            "dvl_rate": 10.0,
            "dvl_velocity_noise_std": 0.003,
            "dvl_bias_x": 0.0,
            "dvl_bias_y": 0.0,
            "dvl_bias_z": 0.0,
            "dvl_dropout_probability": 0.01,
            "dvl_bottom_lock_min_range": 0.08,
            "dvl_bottom_lock_max_range": 5.0,
            "dvl_beam_topics": [
                "/dvl/beam1",
                "/dvl/beam2",
                "/dvl/beam3",
                "/dvl/beam4",
            ],
            "dvl_required_beams": 3,
            "dvl_beam_timeout": 0.35,
            "dvl_beam_angle_from_vertical_deg": 30.0,
            "dvl_lock_acquire_max_range": 1.5,
            "dvl_max_range_jump": 0.45,
            "dvl_reacquire_samples": 3,
            "depth_rate": 20.0,
            "depth_noise_std": 0.01,
            "depth_zero_bias": 0.02,
            "depth_drift_std_per_sqrt_s": 0.0005,
            "water_density": 1000.0,
            "gravity": 9.80665,
            "atmospheric_pressure": 101325.0,
            "random_seed": 7,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.rng = np.random.default_rng(int(self.get_parameter("random_seed").value))
        self.latest_odom: Optional[Odometry] = None
        self.previous_position: Optional[np.ndarray] = None
        self.previous_stamp: Optional[float] = None
        self.world_velocity = np.zeros(3, dtype=float)
        self.depth_drift = 0.0
        self.beam_ranges: dict[str, float] = {}
        self.beam_stamps: dict[str, float] = {}
        self.dvl_bottom_locked = False
        self.dvl_last_locked_range: Optional[float] = None
        self.dvl_reacquire_count = 0

        self.gt_pub = self.create_publisher(
            Odometry, str(self.get_parameter("ground_truth_output_topic").value), 20
        )
        self.dvl_pub = self.create_publisher(
            TwistWithCovarianceStamped, "/dvl/velocity", qos_profile_sensor_data
        )
        self.dvl_valid_pub = self.create_publisher(Bool, "/dvl/valid", 10)
        self.dvl_altitude_pub = self.create_publisher(Float64, "/dvl/altitude", 10)
        self.dvl_beam_pub = self.create_publisher(
            Float64MultiArray, "/dvl/beam_ranges", 10
        )
        self.depth_pub = self.create_publisher(
            FluidPressure, "/depth", qos_profile_sensor_data
        )
        self.depth_value_pub = self.create_publisher(Float64, "/depth/value", 10)
        self.depth_odom_pub = self.create_publisher(Odometry, "/depth/odom", 10)

        self.create_subscription(
            Odometry,
            str(self.get_parameter("ground_truth_topic").value),
            self._ground_truth_callback,
            qos_profile_sensor_data,
        )
        self.beam_subscriptions = []
        for topic in self.get_parameter("dvl_beam_topics").value:
            topic_name = str(topic)
            self.beam_subscriptions.append(
                self.create_subscription(
                    LaserScan,
                    topic_name,
                    lambda message, name=topic_name: self._beam_callback(name, message),
                    qos_profile_sensor_data,
                )
            )
        self.create_timer(1.0 / float(self.get_parameter("dvl_rate").value), self._publish_dvl)
        self.create_timer(1.0 / float(self.get_parameter("depth_rate").value), self._publish_depth)
        self.get_logger().info(
            "Four-beam DVL/depth simulator ready; bottom lock comes from Gazebo ray collisions"
        )

    @staticmethod
    def _stamp_seconds(message: Odometry) -> float:
        return float(message.header.stamp.sec) + 1.0e-9 * float(message.header.stamp.nanosec)

    def _ground_truth_callback(self, message: Odometry) -> None:
        position = np.array(
            [
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            ],
            dtype=float,
        )
        stamp = self._stamp_seconds(message)
        if self.previous_position is not None and self.previous_stamp is not None:
            dt = stamp - self.previous_stamp
            if 1.0e-4 < dt < 0.5:
                measured = (position - self.previous_position) / dt
                self.world_velocity = 0.65 * self.world_velocity + 0.35 * measured
        self.previous_position = position
        self.previous_stamp = stamp
        self.latest_odom = message

        alias = Odometry()
        alias.header = message.header
        alias.header.frame_id = str(self.get_parameter("world_frame").value)
        alias.child_frame_id = str(self.get_parameter("body_frame").value) + "_gt"
        alias.pose = message.pose
        alias.twist = message.twist
        self.gt_pub.publish(alias)

    def _beam_callback(self, topic: str, message: LaserScan) -> None:
        finite = [
            float(value)
            for value in message.ranges
            if math.isfinite(value)
            and float(self.get_parameter("dvl_bottom_lock_min_range").value)
            <= value
            <= float(self.get_parameter("dvl_bottom_lock_max_range").value)
        ]
        self.beam_ranges[topic] = float(np.median(finite)) if finite else math.nan
        self.beam_stamps[topic] = (
            float(message.header.stamp.sec) + 1.0e-9 * float(message.header.stamp.nanosec)
        )

    def _publish_dvl(self) -> None:
        if self.latest_odom is None:
            return
        stamp = self._stamp_seconds(self.latest_odom)
        timeout = float(self.get_parameter("dvl_beam_timeout").value)
        topics = [str(topic) for topic in self.get_parameter("dvl_beam_topics").value]
        ranges = []
        for topic in topics:
            value = self.beam_ranges.get(topic, math.nan)
            beam_stamp = self.beam_stamps.get(topic, -math.inf)
            ranges.append(value if stamp - beam_stamp <= timeout else math.nan)
        valid_ranges = [value for value in ranges if math.isfinite(value)]
        in_range = len(valid_ranges) >= int(self.get_parameter("dvl_required_beams").value)
        median_range = float(np.median(valid_ranges)) if valid_ranges else math.nan
        geometrically_consistent = in_range
        if self.dvl_bottom_locked and math.isfinite(median_range):
            if (
                self.dvl_last_locked_range is not None
                and abs(median_range - self.dvl_last_locked_range)
                > float(self.get_parameter("dvl_max_range_jump").value)
            ):
                self.dvl_bottom_locked = False
                self.dvl_reacquire_count = 0
                geometrically_consistent = False
                self.get_logger().warning(
                    "DVL bottom lock rejected: implausible beam-range discontinuity"
                )
        if not self.dvl_bottom_locked:
            can_acquire = (
                in_range
                and math.isfinite(median_range)
                and median_range
                <= float(self.get_parameter("dvl_lock_acquire_max_range").value)
            )
            self.dvl_reacquire_count = self.dvl_reacquire_count + 1 if can_acquire else 0
            if self.dvl_reacquire_count >= int(
                self.get_parameter("dvl_reacquire_samples").value
            ):
                self.dvl_bottom_locked = True
                self.dvl_reacquire_count = 0
        if self.dvl_bottom_locked and in_range:
            self.dvl_last_locked_range = median_range
        dropout = self.rng.random() < float(
            self.get_parameter("dvl_dropout_probability").value
        )
        valid = in_range and geometrically_consistent and self.dvl_bottom_locked and not dropout

        ranges_message = Float64MultiArray()
        ranges_message.data = ranges
        self.dvl_beam_pub.publish(ranges_message)
        altitude_message = Float64()
        if valid and valid_ranges:
            beam_angle = math.radians(
                float(self.get_parameter("dvl_beam_angle_from_vertical_deg").value)
            )
            altitude_message.data = float(np.median(valid_ranges) * math.cos(beam_angle))
        else:
            altitude_message.data = math.nan
        self.dvl_altitude_pub.publish(altitude_message)

        valid_message = Bool()
        valid_message.data = valid
        self.dvl_valid_pub.publish(valid_message)
        if not valid:
            return

        orientation = self.latest_odom.pose.pose.orientation
        q_world_body = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        body_velocity = rotate_vector(
            quaternion_conjugate(q_world_body), self.world_velocity
        )
        noise_std = float(self.get_parameter("dvl_velocity_noise_std").value)
        bias = np.array(
            [
                self.get_parameter("dvl_bias_x").value,
                self.get_parameter("dvl_bias_y").value,
                self.get_parameter("dvl_bias_z").value,
            ],
            dtype=float,
        )
        measurement = body_velocity + bias + self.rng.normal(0.0, noise_std, 3)

        output = TwistWithCovarianceStamped()
        output.header.stamp = self.latest_odom.header.stamp
        output.header.frame_id = str(self.get_parameter("body_frame").value)
        output.twist.twist.linear.x = float(measurement[0])
        output.twist.twist.linear.y = float(measurement[1])
        output.twist.twist.linear.z = float(measurement[2])
        variance = noise_std * noise_std
        output.twist.covariance[0] = variance
        output.twist.covariance[7] = variance
        output.twist.covariance[14] = variance
        output.twist.covariance[21] = 1.0e6
        output.twist.covariance[28] = 1.0e6
        output.twist.covariance[35] = 1.0e6
        self.dvl_pub.publish(output)

    def _publish_depth(self) -> None:
        if self.latest_odom is None:
            return
        rate = float(self.get_parameter("depth_rate").value)
        drift_density = float(self.get_parameter("depth_drift_std_per_sqrt_s").value)
        self.depth_drift += float(self.rng.normal(0.0, drift_density / math.sqrt(rate)))
        noise_std = float(self.get_parameter("depth_noise_std").value)
        true_depth = max(0.0, -float(self.latest_odom.pose.pose.position.z))
        measured_depth = max(
            0.0,
            true_depth
            + float(self.get_parameter("depth_zero_bias").value)
            + self.depth_drift
            + float(self.rng.normal(0.0, noise_std)),
        )

        density = float(self.get_parameter("water_density").value)
        gravity = float(self.get_parameter("gravity").value)
        pressure = (
            float(self.get_parameter("atmospheric_pressure").value)
            + density * gravity * measured_depth
        )
        pressure_message = FluidPressure()
        pressure_message.header.stamp = self.latest_odom.header.stamp
        pressure_message.header.frame_id = str(self.get_parameter("body_frame").value)
        pressure_message.fluid_pressure = pressure
        pressure_message.variance = (density * gravity * noise_std) ** 2
        self.depth_pub.publish(pressure_message)

        depth_message = Float64()
        depth_message.data = measured_depth
        self.depth_value_pub.publish(depth_message)

        depth_odom = Odometry()
        depth_odom.header.stamp = self.latest_odom.header.stamp
        depth_odom.header.frame_id = str(self.get_parameter("world_frame").value)
        depth_odom.child_frame_id = str(self.get_parameter("body_frame").value)
        depth_odom.pose.pose.position.z = -measured_depth
        depth_odom.pose.pose.orientation.w = 1.0
        depth_odom.pose.covariance = [1.0e6] * 36
        depth_odom.pose.covariance[14] = noise_std * noise_std
        self.depth_odom_pub.publish(depth_odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorSimulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
