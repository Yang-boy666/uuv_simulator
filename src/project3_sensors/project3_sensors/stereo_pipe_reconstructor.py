"""Incremental coloured stereo point-cloud reconstruction.

Only calibrated stereo images and the fused navigation pose are used.  Ground
truth and the pipe/world description are deliberately not subscribed.
"""

from __future__ import annotations

from collections import deque
import math
import os
import struct
from typing import Deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .math_utils import normalize_quaternion, rotate_vector


class StereoPipeReconstructor(Node):
    def __init__(self) -> None:
        super().__init__("stereo_pipe_reconstructor")
        defaults = {
            "left_topic": "/cam0/image_raw",
            "right_topic": "/cam1/image_raw",
            "odom_topic": "/navigation/odom",
            "cloud_topic": "/reconstruction/pipe_cloud",
            "focal_length": 381.3611,
            "principal_x": 320.0,
            "principal_y": 200.0,
            "baseline": 0.12,
            "camera_body_x": 0.212,
            "camera_body_y": -0.021,
            "camera_body_z": -0.018,
            "minimum_depth": 0.25,
            "maximum_depth": 4.0,
            "pixel_step": 5,
            "voxel_size": 0.04,
            "max_voxels": 300000,
            "process_rate": 1.5,
            "publish_rate": 0.5,
            "sync_tolerance": 0.03,
            "pose_tolerance": 0.20,
            "num_disparities": 96,
            "output_path": "/home/zhuchao/uuv_sim_ws/reconstruction/pipe_model.ply",
            "minimum_save_points": 100,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        disparities = int(self.get_parameter("num_disparities").value)
        disparities = max(16, ((disparities + 15) // 16) * 16)
        block = 5
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=disparities,
            blockSize=block,
            P1=8 * block * block,
            P2=32 * block * block,
            disp12MaxDiff=2,
            uniquenessRatio=8,
            speckleWindowSize=100,
            speckleRange=2,
            preFilterCap=31,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        self.bridge = CvBridge()
        self.left: Image | None = None
        self.right: Image | None = None
        self.odometry: Deque[tuple[float, Odometry]] = deque(maxlen=300)
        # key -> running [sum_x, sum_y, sum_z, sum_r, sum_g, sum_b, count]
        self.voxels: dict[tuple[int, int, int], np.ndarray] = {}
        self.last_stamp = -1
        self.last_process_time = -1.0
        self.control_was_enabled = False

        cloud_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("cloud_topic").value), cloud_qos
        )
        self.status_pub = self.create_publisher(String, "/reconstruction/status", 10)
        self.create_subscription(Image, str(self.get_parameter("left_topic").value), self._left, qos_profile_sensor_data)
        self.create_subscription(Image, str(self.get_parameter("right_topic").value), self._right, qos_profile_sensor_data)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._odom, 50)
        self.create_subscription(Bool, "/pipeline_uuv/control_enabled", self._control, 10)
        self.create_service(Trigger, "/reconstruction/save", self._save_service)
        self.create_timer(1.0 / float(self.get_parameter("publish_rate").value), self._publish_cloud)
        self.get_logger().info(
            "Stereo pipe reconstruction ready; save with /reconstruction/save"
        )

    @staticmethod
    def _stamp(message) -> float:
        return message.header.stamp.sec + 1.0e-9 * message.header.stamp.nanosec

    @staticmethod
    def _stamp_ns(message) -> int:
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def _odom(self, message: Odometry) -> None:
        self.odometry.append((self._stamp(message), message))

    def _left(self, message: Image) -> None:
        self.left = message
        self._process()

    def _right(self, message: Image) -> None:
        self.right = message
        self._process()

    def _nearest_pose(self, stamp: float) -> Odometry | None:
        if not self.odometry:
            return None
        difference, message = min(
            ((abs(item_stamp - stamp), item) for item_stamp, item in self.odometry),
            key=lambda pair: pair[0],
        )
        if difference > float(self.get_parameter("pose_tolerance").value):
            return None
        return message

    def _process(self) -> None:
        if self.left is None or self.right is None:
            return
        left_stamp = self._stamp(self.left)
        right_stamp = self._stamp(self.right)
        if abs(left_stamp - right_stamp) > float(self.get_parameter("sync_tolerance").value):
            return
        stamp_ns = max(self._stamp_ns(self.left), self._stamp_ns(self.right))
        if stamp_ns == self.last_stamp:
            return
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if self.last_process_time >= 0.0 and now - self.last_process_time < 1.0 / float(
            self.get_parameter("process_rate").value
        ):
            return
        pose = self._nearest_pose(max(left_stamp, right_stamp))
        if pose is None:
            return
        self.last_stamp = stamp_ns
        self.last_process_time = now
        try:
            left_gray = self.bridge.imgmsg_to_cv2(self.left, "mono8")
            right_gray = self.bridge.imgmsg_to_cv2(self.right, "mono8")
            left_color = self.bridge.imgmsg_to_cv2(self.left, "bgr8")
        except Exception as error:
            self.get_logger().warning(f"Reconstruction image conversion failed: {error}")
            return
        disparity = self.matcher.compute(left_gray, right_gray).astype(np.float32) / 16.0
        step = max(1, int(self.get_parameter("pixel_step").value))
        rows = np.arange(step, disparity.shape[0] - step, step)
        cols = np.arange(step, disparity.shape[1] - step, step)
        vv, uu = np.meshgrid(rows, cols, indexing="ij")
        dd = disparity[vv, uu]
        focal = float(self.get_parameter("focal_length").value)
        baseline = float(self.get_parameter("baseline").value)
        depth = focal * baseline / np.maximum(dd, 1.0e-6)
        valid = (
            (dd > 0.75)
            & (depth >= float(self.get_parameter("minimum_depth").value))
            & (depth <= float(self.get_parameter("maximum_depth").value))
        )
        if not np.any(valid):
            return
        u = uu[valid].astype(np.float64)
        v = vv[valid].astype(np.float64)
        z_camera = depth[valid].astype(np.float64)
        x_camera = (u - float(self.get_parameter("principal_x").value)) * z_camera / focal
        y_camera = (v - float(self.get_parameter("principal_y").value)) * z_camera / focal
        # ROS optical frame (right, down, forward) -> vehicle frame
        body_points = np.column_stack(
            (
                z_camera + float(self.get_parameter("camera_body_x").value),
                -x_camera + float(self.get_parameter("camera_body_y").value),
                -y_camera + float(self.get_parameter("camera_body_z").value),
            )
        )
        q = pose.pose.pose.orientation
        quaternion = normalize_quaternion((q.x, q.y, q.z, q.w))
        position = pose.pose.pose.position
        translation = np.array([position.x, position.y, position.z], dtype=float)
        world_points = np.array(
            [rotate_vector(quaternion, point) for point in body_points], dtype=float
        ) + translation
        colors_bgr = left_color[vv[valid], uu[valid]]
        voxel_size = float(self.get_parameter("voxel_size").value)
        maximum = int(self.get_parameter("max_voxels").value)
        for point, bgr in zip(world_points, colors_bgr):
            key = tuple(np.floor(point / voxel_size).astype(np.int64).tolist())
            sample = np.array(
                [point[0], point[1], point[2], bgr[2], bgr[1], bgr[0], 1.0], dtype=float
            )
            if key in self.voxels:
                self.voxels[key] += sample
            elif len(self.voxels) < maximum:
                self.voxels[key] = sample
        status = String()
        status.data = f"accumulating {len(self.voxels)} coloured voxels"
        self.status_pub.publish(status)

    def _points(self):
        for accumulator in self.voxels.values():
            count = max(1.0, accumulator[6])
            red, green, blue = (int(np.clip(value / count, 0, 255)) for value in accumulator[3:6])
            packed = (red << 16) | (green << 8) | blue
            rgb_float = struct.unpack("f", struct.pack("I", packed))[0]
            yield (
                float(accumulator[0] / count),
                float(accumulator[1] / count),
                float(accumulator[2] / count),
                rgb_float,
            )

    def _publish_cloud(self) -> None:
        if not self.voxels:
            return
        header = self.left.header if self.left is not None else None
        if header is None:
            return
        header.frame_id = "world"
        header.stamp = self.get_clock().now().to_msg()
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.cloud_pub.publish(point_cloud2.create_cloud(header, fields, list(self._points())))

    def _control(self, message: Bool) -> None:
        enabled = bool(message.data)
        if self.control_was_enabled and not enabled and len(self.voxels) >= int(
            self.get_parameter("minimum_save_points").value
        ):
            self._save()
        self.control_was_enabled = enabled

    def _save_service(self, request, response: Trigger.Response) -> Trigger.Response:
        del request
        try:
            path = self._save()
            response.success = True
            response.message = f"saved {len(self.voxels)} points to {path}"
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def _save(self) -> str:
        path = os.path.abspath(os.path.expanduser(str(self.get_parameter("output_path").value)))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        points = list(self._points())
        with open(path, "w", encoding="utf-8") as output:
            output.write("ply\nformat ascii 1.0\n")
            output.write(f"element vertex {len(points)}\n")
            output.write("property float x\nproperty float y\nproperty float z\n")
            output.write("property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
            for x, y, z, rgb_float in points:
                packed = struct.unpack("I", struct.pack("f", rgb_float))[0]
                red, green, blue = (packed >> 16) & 255, (packed >> 8) & 255, packed & 255
                output.write(f"{x:.5f} {y:.5f} {z:.5f} {red} {green} {blue}\n")
        status = String()
        status.data = f"saved {len(points)} points: {path}"
        self.status_pub.publish(status)
        self.get_logger().info(status.data)
        return path


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StereoPipeReconstructor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
