"""Render a live DVL/INS, visual-inertial and fused navigation comparison."""

from __future__ import annotations

from collections import deque
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, Float64MultiArray, String


class FusionDashboard(Node):
    def __init__(self) -> None:
        super().__init__("fusion_dashboard")
        self.declare_parameter("max_points", 5000)
        self.declare_parameter("publish_rate", 4.0)
        self.declare_parameter("output_topic", "/navigation/fusion_dashboard")
        maximum = int(self.get_parameter("max_points").value)
        self.tracks = {
            "DVL+INS+depth": deque(maxlen=maximum),
            "SVIn vision": deque(maxlen=maximum),
            "EKF fused": deque(maxlen=maximum),
            "Ground truth": deque(maxlen=maximum),
        }
        self.latest: dict[str, Odometry] = {}
        self.dvl_ok = False
        self.svin_ok = False
        self.altitude = math.nan
        self.metrics: list[float] = []
        self.planner_state = "WAITING"
        self.obstacle_source = "NONE"
        self.guidance: list[float] = []
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image, str(self.get_parameter("output_topic").value), 3
        )
        topics = {
            "DVL+INS+depth": "/navigation/dead_reckoning",
            "SVIn vision": "/svin/aligned_odom",
            "EKF fused": "/navigation/odom",
            "Ground truth": "/ground_truth/odom",
        }
        for label, topic in topics.items():
            self.create_subscription(
                Odometry, topic, lambda msg, key=label: self._odom(key, msg), 20
            )
        self.create_subscription(Bool, "/dvl/valid", self._dvl, 10)
        self.create_subscription(Bool, "/svin/tracking_ok", self._svin, 10)
        self.create_subscription(Float64, "/dvl/altitude", self._altitude, 10)
        self.create_subscription(Float64MultiArray, "/navigation/metrics", self._metrics, 10)
        self.create_subscription(
            Float64MultiArray,
            "/pipeline_uuv/pipe_guidance/diagnostics",
            self._guidance,
            10,
        )
        self.create_subscription(
            String,
            "/pipeline_uuv/pipe_guidance/planner_state",
            self._planner,
            10,
        )
        self.create_subscription(
            String,
            "/pipeline_uuv/pipe_guidance/obstacle_source",
            self._obstacle_source,
            10,
        )
        self.create_timer(1.0 / float(self.get_parameter("publish_rate").value), self._render)
        self.get_logger().info("Navigation fusion comparison dashboard ready")

    def _odom(self, label: str, message: Odometry) -> None:
        p = message.pose.pose.position
        point = (float(p.x), float(p.y), float(p.z))
        track = self.tracks[label]
        if not track or math.dist(track[-1], point) >= 0.01:
            track.append(point)
        self.latest[label] = message

    def _dvl(self, message: Bool) -> None:
        self.dvl_ok = bool(message.data)

    def _svin(self, message: Bool) -> None:
        self.svin_ok = bool(message.data)

    def _altitude(self, message: Float64) -> None:
        self.altitude = float(message.data)

    def _metrics(self, message: Float64MultiArray) -> None:
        self.metrics = list(message.data)

    def _guidance(self, message: Float64MultiArray) -> None:
        self.guidance = list(message.data)

    def _planner(self, message: String) -> None:
        self.planner_state = message.data

    def _obstacle_source(self, message: String) -> None:
        self.obstacle_source = message.data

    def _render(self) -> None:
        canvas = np.full((720, 1100, 3), (25, 30, 38), dtype=np.uint8)
        plot_x0, plot_y0, plot_w, plot_h = 35, 55, 735, 620
        cv2.rectangle(canvas, (plot_x0, plot_y0), (plot_x0 + plot_w, plot_y0 + plot_h), (75, 80, 88), 1)
        all_points = [point for track in self.tracks.values() for point in track]
        if all_points:
            xs = [p[0] for p in all_points]
            ys = [p[1] for p in all_points]
            margin = 1.0
            xmin, xmax = min(xs) - margin, max(xs) + margin
            ymin, ymax = min(ys) - margin, max(ys) + margin
            span_x, span_y = max(2.0, xmax - xmin), max(2.0, ymax - ymin)
            scale = min(plot_w / span_x, plot_h / span_y)

            def pixel(point):
                return (
                    int(plot_x0 + (point[0] - xmin) * scale),
                    int(plot_y0 + plot_h - (point[1] - ymin) * scale),
                )

            colors = {
                "DVL+INS+depth": (0, 165, 255),
                "SVIn vision": (220, 80, 255),
                "EKF fused": (255, 190, 0),
                "Ground truth": (70, 230, 80),
            }
            for label, track in self.tracks.items():
                if len(track) >= 2:
                    points = np.array([pixel(p) for p in track], dtype=np.int32)
                    cv2.polylines(canvas, [points], False, colors[label], 2, cv2.LINE_AA)
                if track:
                    cv2.circle(canvas, pixel(track[-1]), 5, colors[label], -1)
            for index, label in enumerate(colors):
                y = 25
                x = 35 + index * 180
                cv2.line(canvas, (x, y), (x + 25, y), colors[label], 3)
                cv2.putText(canvas, label, (x + 32, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (235, 235, 235), 1)
        else:
            cv2.putText(canvas, "Waiting for navigation topics...", (230, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 180, 180), 2)

        panel_x = 800
        cv2.putText(canvas, "MULTI-SENSOR NAVIGATION", (panel_x, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
        statuses = [("DVL bottom lock", self.dvl_ok), ("SVIn tracking", self.svin_ok)]
        y = 100
        for label, ok in statuses:
            color = (80, 220, 80) if ok else (60, 60, 230)
            cv2.circle(canvas, (panel_x + 8, y - 5), 7, color, -1)
            cv2.putText(canvas, f"{label}: {'OK' if ok else 'LOST'}", (panel_x + 25, y), cv2.FONT_HERSHEY_SIMPLEX, 0.53, color, 1)
            y += 34
        altitude_text = f"DVL altitude: {self.altitude:.3f} m" if math.isfinite(self.altitude) else "DVL altitude: --"
        cv2.putText(canvas, altitude_text, (panel_x, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (220, 220, 220), 1)
        y += 55

        for label in ("DVL+INS+depth", "SVIn vision", "EKF fused", "Ground truth"):
            message = self.latest.get(label)
            cv2.putText(canvas, label, (panel_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (245, 245, 245), 1)
            y += 22
            if message is None:
                value = "x --   y --   z --"
            else:
                p = message.pose.pose.position
                value = f"x {p.x:+7.2f}  y {p.y:+7.2f}  z {p.z:+6.2f}"
            cv2.putText(canvas, value, (panel_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 205, 225), 1)
            y += 35
        if len(self.metrics) >= 4:
            y += 10
            cv2.putText(canvas, "Ground-truth evaluation only", (panel_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (140, 190, 240), 1)
            y += 25
            cv2.putText(canvas, f"Current 3D error  {self.metrics[0]:.3f} m", (panel_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (220, 220, 220), 1)
            y += 24
            cv2.putText(canvas, f"ATE RMSE          {self.metrics[1]:.3f} m", (panel_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (220, 220, 220), 1)
            y += 24
            cv2.putText(canvas, f"ATE max           {self.metrics[3]:.3f} m", (panel_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (220, 220, 220), 1)
        planner_color = (80, 220, 80) if self.planner_state == "CRUISE" else (0, 190, 255)
        planner_text = (
            f"Local planner: {self.planner_state} | trigger: {self.obstacle_source}"
        )
        cv2.putText(canvas, planner_text, (25, 666), cv2.FONT_HERSHEY_SIMPLEX, 0.5, planner_color, 1)
        range_text = "Waiting for obstacle ranges..."
        if len(self.guidance) >= 20:
            range_text = (
                f"Sonar L/F/R {self.guidance[0]:.2f}/{self.guidance[19]:.2f}/"
                f"{self.guidance[1]:.2f} m"
            )
        if len(self.guidance) >= 25:
            range_text += (
                f" | Vision L/F/R {self.guidance[21]:.2f}/{self.guidance[20]:.2f}/"
                f"{self.guidance[22]:.2f} m q={self.guidance[23]:.2f}"
            )
        cv2.putText(canvas, range_text, (25, 688), cv2.FONT_HERSHEY_SIMPLEX, 0.46, planner_color, 1)
        cv2.putText(canvas, "Navigation: stereo + sonar + IMU + DVL + depth | planning uses no map/GT", (25, 710), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (170, 180, 195), 1)
        output = self.bridge.cv2_to_imgmsg(canvas, "bgr8")
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "world"
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FusionDashboard()
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
