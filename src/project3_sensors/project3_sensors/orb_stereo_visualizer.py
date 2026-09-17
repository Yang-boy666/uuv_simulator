"""Publish a side-by-side ORB stereo feature/match diagnostic image."""

from __future__ import annotations

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray


class OrbStereoVisualizer(Node):
    """Independent ORB diagnostic; SVIn itself continues to use BRISK/OKVIS."""

    def __init__(self) -> None:
        super().__init__("orb_stereo_visualizer")
        self.declare_parameter("left_topic", "/cam0/image_raw")
        self.declare_parameter("right_topic", "/cam1/image_raw")
        self.declare_parameter("output_topic", "/vision/orb_matches")
        self.declare_parameter("max_features", 500)
        self.declare_parameter("max_matches", 80)
        self.declare_parameter("max_rate", 5.0)
        self.declare_parameter("sync_tolerance", 0.03)

        self.bridge = CvBridge()
        self.orb = cv2.ORB_create(nfeatures=int(self.get_parameter("max_features").value))
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.left: Image | None = None
        self.right: Image | None = None
        self.last_processed_stamp = -1
        self.last_process_time = -1.0

        self.image_pub = self.create_publisher(
            Image, str(self.get_parameter("output_topic").value), 3
        )
        self.stats_pub = self.create_publisher(
            Float64MultiArray, "/vision/orb_stats", 10
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("left_topic").value),
            self._left_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("right_topic").value),
            self._right_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "ORB stereo comparison ready: green lines are left/right descriptor matches"
        )

    @staticmethod
    def _stamp_seconds(message: Image) -> float:
        return float(message.header.stamp.sec) + 1.0e-9 * float(message.header.stamp.nanosec)

    @staticmethod
    def _stamp_nanoseconds(message: Image) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

    def _left_callback(self, message: Image) -> None:
        self.left = message
        self._process_if_synchronized()

    def _right_callback(self, message: Image) -> None:
        self.right = message
        self._process_if_synchronized()

    def _process_if_synchronized(self) -> None:
        if self.left is None or self.right is None:
            return
        left_time = self._stamp_seconds(self.left)
        right_time = self._stamp_seconds(self.right)
        if abs(left_time - right_time) > float(self.get_parameter("sync_tolerance").value):
            return
        stamp = max(self._stamp_nanoseconds(self.left), self._stamp_nanoseconds(self.right))
        if stamp == self.last_processed_stamp:
            return
        now = self.get_clock().now().nanoseconds * 1.0e-9
        maximum_rate = float(self.get_parameter("max_rate").value)
        if self.last_process_time >= 0.0 and now - self.last_process_time < 1.0 / maximum_rate:
            return
        self.last_processed_stamp = stamp
        self.last_process_time = now

        try:
            left = self.bridge.imgmsg_to_cv2(self.left, desired_encoding="mono8")
            right = self.bridge.imgmsg_to_cv2(self.right, desired_encoding="mono8")
        except Exception as error:  # cv_bridge reports several concrete exception types
            self.get_logger().warning(f"Cannot convert stereo image: {error}")
            return

        keypoints_left, descriptors_left = self.orb.detectAndCompute(left, None)
        keypoints_right, descriptors_right = self.orb.detectAndCompute(right, None)
        matches = []
        if descriptors_left is not None and descriptors_right is not None:
            matches = sorted(
                self.matcher.match(descriptors_left, descriptors_right),
                key=lambda match: match.distance,
            )
        shown = matches[: int(self.get_parameter("max_matches").value)]
        comparison = cv2.drawMatches(
            left,
            keypoints_left,
            right,
            keypoints_right,
            shown,
            None,
            matchColor=(0, 255, 0),
            singlePointColor=(0, 180, 255),
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
        )
        label = (
            f"ORB left={len(keypoints_left)} right={len(keypoints_right)} "
            f"matches={len(matches)} shown={len(shown)}"
        )
        cv2.rectangle(comparison, (0, 0), (comparison.shape[1], 30), (0, 0, 0), -1)
        cv2.putText(
            comparison,
            label,
            (10, 21),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        output = self.bridge.cv2_to_imgmsg(comparison, encoding="bgr8")
        output.header = self.left.header
        self.image_pub.publish(output)
        stats = Float64MultiArray()
        stats.data = [
            float(len(keypoints_left)),
            float(len(keypoints_right)),
            float(len(matches)),
            float(sum(match.distance for match in shown) / len(shown)) if shown else -1.0,
        ]
        self.stats_pub.publish(stats)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrbStereoVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
