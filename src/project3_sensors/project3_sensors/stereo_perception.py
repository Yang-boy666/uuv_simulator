"""Stereo free-space perception for route-independent pipe guidance.

The node uses only the two camera images and calibration parameters.  It does
not consume the simulated world, pipe radius, route waypoints or ground truth.
"""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray


class StereoPerception(Node):
    def __init__(self) -> None:
        super().__init__("stereo_perception")
        defaults = {
            "left_topic": "/cam0/image_raw",
            "right_topic": "/cam1/image_raw",
            "guidance_topic": "/vision/corridor_guidance",
            "debug_topic": "/vision/disparity_guidance",
            "sync_tolerance": 0.03,
            "max_rate": 5.0,
            "num_disparities": 96,
            "block_size": 5,
            "minimum_valid_fraction": 0.015,
            "camera_fx": 381.361,
            "stereo_baseline": 0.12,
            "range_percentile": 80.0,
            "range_min_pixels": 120,
            "range_max": 8.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        disparities = int(self.get_parameter("num_disparities").value)
        disparities = max(16, ((disparities + 15) // 16) * 16)
        block = int(self.get_parameter("block_size").value)
        block = block if block % 2 else block + 1
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=disparities,
            blockSize=max(3, block),
            P1=8 * block * block,
            P2=32 * block * block,
            disp12MaxDiff=2,
            uniquenessRatio=7,
            speckleWindowSize=80,
            speckleRange=2,
            preFilterCap=31,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        self.bridge = CvBridge()
        self.left: Image | None = None
        self.right: Image | None = None
        self.last_stamp = -1
        self.last_time = -1.0
        self.guidance_pub = self.create_publisher(
            Float64MultiArray, str(self.get_parameter("guidance_topic").value), 10
        )
        self.debug_pub = self.create_publisher(
            Image, str(self.get_parameter("debug_topic").value), 3
        )
        self.create_subscription(
            Image, str(self.get_parameter("left_topic").value), self._left, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, str(self.get_parameter("right_topic").value), self._right, qos_profile_sensor_data
        )
        self.get_logger().info(
            "Stereo free-space perception ready (no route or pipe geometry input)"
        )

    @staticmethod
    def _seconds(message: Image) -> float:
        return message.header.stamp.sec + 1.0e-9 * message.header.stamp.nanosec

    @staticmethod
    def _nanoseconds(message: Image) -> int:
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def _left(self, message: Image) -> None:
        self.left = message
        self._process()

    def _right(self, message: Image) -> None:
        self.right = message
        self._process()

    def _process(self) -> None:
        if self.left is None or self.right is None:
            return
        if abs(self._seconds(self.left) - self._seconds(self.right)) > float(
            self.get_parameter("sync_tolerance").value
        ):
            return
        stamp = max(self._nanoseconds(self.left), self._nanoseconds(self.right))
        if stamp == self.last_stamp:
            return
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if self.last_time >= 0.0 and now - self.last_time < 1.0 / float(
            self.get_parameter("max_rate").value
        ):
            return
        self.last_stamp = stamp
        self.last_time = now
        try:
            left = self.bridge.imgmsg_to_cv2(self.left, "mono8")
            right = self.bridge.imgmsg_to_cv2(self.right, "mono8")
        except Exception as error:
            self.get_logger().warning(f"Stereo conversion failed: {error}")
            return

        disparity = self.matcher.compute(left, right).astype(np.float32) / 16.0
        height, width = disparity.shape
        # Ignore image edges and the lower vehicle/camera housing region.
        roi = np.zeros_like(disparity, dtype=bool)
        roi[int(0.08 * height):int(0.88 * height), int(0.08 * width):int(0.92 * width)] = True
        valid = (disparity > 0.75) & roi
        valid_fraction = float(np.count_nonzero(valid)) / float(max(1, np.count_nonzero(roi)))

        # Low disparity is distant/open space.  Robustly select the farthest
        # third of valid stereo pixels and use its weighted centroid.
        horizontal_error = 0.0
        opening_vertical_error = 0.0
        wall_vertical_error = 0.0
        vertical_error = 0.0
        confidence = 0.0
        opening = np.zeros_like(valid)
        if valid_fraction >= float(self.get_parameter("minimum_valid_fraction").value):
            cutoff = float(np.percentile(disparity[valid], 35.0))
            opening = valid & (disparity <= cutoff)
            ys, xs = np.nonzero(opening)
            if xs.size:
                weights = 1.0 / np.maximum(disparity[ys, xs], 0.75)
                center_x = float(np.average(xs, weights=weights))
                center_y = float(np.average(ys, weights=weights))
                horizontal_error = (center_x - 0.5 * width) / (0.5 * width)
                # Positive is an opening above image center, matching positive
                # world/body Z command in the level-vehicle controller.
                opening_vertical_error = (0.5 * height - center_y) / (0.5 * height)
                confidence = min(1.0, 5.0 * float(xs.size) / float(width * height))

            # Near-wall balance is more stable than the far-pixel centroid in
            # textured pipes.  If the lower image has greater disparity, the
            # lower wall is closer and the demanded correction is upward.
            top = disparity[int(0.10 * height):int(0.38 * height), int(0.18 * width):int(0.82 * width)]
            bottom = disparity[int(0.62 * height):int(0.88 * height), int(0.18 * width):int(0.82 * width)]
            top_values = top[top > 0.75]
            bottom_values = bottom[bottom > 0.75]
            if top_values.size > 100 and bottom_values.size > 100:
                top_near = float(np.percentile(top_values, 65.0))
                bottom_near = float(np.percentile(bottom_values, 65.0))
                wall_vertical_error = (bottom_near - top_near) / max(
                    1.0, bottom_near + top_near
                )
                vertical_error = float(
                    np.clip(
                        0.60 * wall_vertical_error + 0.40 * opening_vertical_error,
                        -1.0,
                        1.0,
                    )
                )
            else:
                vertical_error = opening_vertical_error

        # Calibrated stereo obstacle ranges.  A high disparity percentile is a
        # robust near-surface estimate; depth follows Z = fx * baseline / d.
        # The middle-height crop excludes most of the pipe floor and ceiling.
        range_min_pixels = int(self.get_parameter("range_min_pixels").value)
        range_max = float(self.get_parameter("range_max").value)

        def region_clearance(x0: float, x1: float) -> tuple[float, float]:
            region = disparity[
                int(0.22 * height):int(0.78 * height),
                int(x0 * width):int(x1 * width),
            ]
            values = region[region > 0.75]
            if values.size < range_min_pixels:
                return range_max, 0.0
            near_disparity = float(
                np.percentile(
                    values, float(self.get_parameter("range_percentile").value)
                )
            )
            distance = (
                float(self.get_parameter("camera_fx").value)
                * float(self.get_parameter("stereo_baseline").value)
                / max(near_disparity, 0.75)
            )
            confidence_value = min(1.0, float(values.size) / (4.0 * range_min_pixels))
            return min(range_max, distance), confidence_value

        visual_front, front_range_confidence = region_clearance(0.20, 0.80)
        image_left, image_left_confidence = region_clearance(0.08, 0.43)
        image_right, image_right_confidence = region_clearance(0.57, 0.92)
        # The Gazebo camera image x-axis is opposite the vehicle's +Y-left
        # convention.  Publish all guidance ranges in the body convention.
        visual_left, left_range_confidence = image_right, image_right_confidence
        visual_right, right_range_confidence = image_left, image_left_confidence
        range_confidence = min(
            front_range_confidence,
            max(left_range_confidence, right_range_confidence),
        )

        guidance = Float64MultiArray()
        # [horizontal_error, vertical_error, corridor_confidence, valid_fraction,
        #  front_range, left_range, right_range, range_confidence]
        guidance.data = [
            horizontal_error,
            vertical_error,
            confidence,
            valid_fraction,
            visual_front,
            visual_left,
            visual_right,
            range_confidence,
        ]
        self.guidance_pub.publish(guidance)

        visual = cv2.cvtColor(left, cv2.COLOR_GRAY2BGR)
        visual[opening] = (0, 180, 0)
        target = (
            int(0.5 * width * (1.0 + horizontal_error)),
            int(0.5 * height * (1.0 - opening_vertical_error)),
        )
        cv2.drawMarker(visual, target, (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
        for x0, x1, color in (
            (0.08, 0.43, (255, 180, 0)),
            (0.20, 0.80, (0, 220, 255)),
            (0.57, 0.92, (255, 80, 180)),
        ):
            cv2.rectangle(
                visual,
                (int(x0 * width), int(0.22 * height)),
                (int(x1 * width), int(0.78 * height)),
                color,
                1,
            )
        cv2.putText(
            visual,
            f"stereo x={horizontal_error:+.2f} z={vertical_error:+.2f} "
            f"wall={wall_vertical_error:+.2f} conf={confidence:.2f}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            visual,
            f"visual range L/F/R={visual_left:.2f}/{visual_front:.2f}/{visual_right:.2f}m "
            f"q={range_confidence:.2f}",
            (10, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        output = self.bridge.cv2_to_imgmsg(visual, "bgr8")
        output.header = self.left.header
        self.debug_pub.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StereoPerception()
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
