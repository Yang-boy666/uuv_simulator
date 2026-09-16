"""Independent DVL/IMU/depth dead reckoning used to validate SVIn updates."""

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
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from .math_utils import normalize_quaternion, quaternion_multiply, rotate_vector


class DeadReckoningGuard(Node):
    """Propagate a pose without consuming any visual or Gazebo truth topic."""

    def __init__(self) -> None:
        super().__init__("dead_reckoning_guard")
        defaults = {
            "dvl_topic": "/dvl/velocity",
            "imu_topic": "/imu/data_raw",
            "depth_topic": "/depth/value",
            "output_topic": "/navigation/dead_reckoning",
            "world_frame": "world",
            "body_frame": "pipeline_uuv/base_link",
            "initial_x": -8.5,
            "initial_y": -5.0,
            "initial_z": -4.0,
            "initial_yaw": 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.position = np.array(
            [
                self.get_parameter("initial_x").value,
                self.get_parameter("initial_y").value,
                self.get_parameter("initial_z").value,
            ],
            dtype=float,
        )
        yaw = float(self.get_parameter("initial_yaw").value)
        self.orientation = normalize_quaternion(
            (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))
        )
        self.last_imu_stamp: Optional[float] = None
        self.last_dvl_stamp: Optional[float] = None
        self.latest_depth: Optional[float] = None
        self.distance = 0.0

        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 20
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistWithCovarianceStamped,
            str(self.get_parameter("dvl_topic").value),
            self._dvl_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("depth_topic").value),
            self._depth_callback,
            20,
        )
        self.get_logger().info(
            "Independent DVL/IMU/depth dead-reckoning guard ready"
        )

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)

    def _imu_callback(self, message: Imu) -> None:
        stamp = self._stamp_seconds(message.header.stamp)
        if self.last_imu_stamp is not None:
            dt = stamp - self.last_imu_stamp
            if 0.0 < dt < 0.1:
                omega = np.array(
                    [
                        message.angular_velocity.x,
                        message.angular_velocity.y,
                        message.angular_velocity.z,
                    ],
                    dtype=float,
                )
                speed = float(np.linalg.norm(omega))
                if speed > 1.0e-12:
                    angle = speed * dt
                    axis = omega / speed
                    half_sine = math.sin(0.5 * angle)
                    delta = normalize_quaternion(
                        (
                            axis[0] * half_sine,
                            axis[1] * half_sine,
                            axis[2] * half_sine,
                            math.cos(0.5 * angle),
                        )
                    )
                    self.orientation = quaternion_multiply(self.orientation, delta)
        self.last_imu_stamp = stamp

    def _depth_callback(self, message: Float64) -> None:
        self.latest_depth = float(message.data)

    def _dvl_callback(self, message: TwistWithCovarianceStamped) -> None:
        stamp = self._stamp_seconds(message.header.stamp)
        body_velocity = np.array(
            [
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.linear.z,
            ],
            dtype=float,
        )
        if self.last_dvl_stamp is not None:
            dt = stamp - self.last_dvl_stamp
            if 0.0 < dt < 0.5:
                world_velocity = rotate_vector(self.orientation, body_velocity)
                displacement = world_velocity * dt
                self.position += displacement
                self.distance += float(np.linalg.norm(displacement))
        self.last_dvl_stamp = stamp
        if self.latest_depth is not None:
            self.position[2] = -self.latest_depth

        output = Odometry()
        output.header = message.header
        output.header.frame_id = str(self.get_parameter("world_frame").value)
        output.child_frame_id = str(self.get_parameter("body_frame").value)
        output.pose.pose.position.x = float(self.position[0])
        output.pose.pose.position.y = float(self.position[1])
        output.pose.pose.position.z = float(self.position[2])
        output.pose.pose.orientation.x = self.orientation[0]
        output.pose.pose.orientation.y = self.orientation[1]
        output.pose.pose.orientation.z = self.orientation[2]
        output.pose.pose.orientation.w = self.orientation[3]
        # Conservative covariance grows with travelled distance. This output is
        # a consistency reference, not a second absolute position measurement.
        horizontal_variance = 0.01 + 0.0025 * self.distance
        output.pose.covariance[0] = horizontal_variance
        output.pose.covariance[7] = horizontal_variance
        output.pose.covariance[14] = 0.0004
        output.pose.covariance[21] = 0.01
        output.pose.covariance[28] = 0.01
        output.pose.covariance[35] = 0.02 + 0.001 * self.distance
        output.twist = message.twist
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DeadReckoningGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
