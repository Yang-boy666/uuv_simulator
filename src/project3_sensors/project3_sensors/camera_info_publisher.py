"""Publish deterministic calibration matching the simulated stereo cameras."""

from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraInfoPublisher(Node):
    def __init__(self) -> None:
        super().__init__("camera_info_publisher")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 400)
        self.declare_parameter("horizontal_fov", 1.047)
        self.declare_parameter("baseline", 0.12)

        self.left_pub = self.create_publisher(
            CameraInfo, "/camera/left/camera_info", qos_profile_sensor_data
        )
        self.right_pub = self.create_publisher(
            CameraInfo, "/camera/right/camera_info", qos_profile_sensor_data
        )
        self.create_subscription(
            Image,
            "/camera/left/image_raw",
            lambda message: self._publish(message, False),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/camera/right/image_raw",
            lambda message: self._publish(message, True),
            qos_profile_sensor_data,
        )
        self.get_logger().info("Stereo CameraInfo publisher ready")

    def _publish(self, image: Image, right: bool) -> None:
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        fov = float(self.get_parameter("horizontal_fov").value)
        fx = width / (2.0 * math.tan(0.5 * fov))
        fy = fx
        cx = 0.5 * width
        cy = 0.5 * height

        info = CameraInfo()
        info.header = image.header
        info.width = width
        info.height = height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        tx = -fx * float(self.get_parameter("baseline").value) if right else 0.0
        info.p = [fx, 0.0, cx, tx, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        (self.right_pub if right else self.left_pub).publish(info)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
