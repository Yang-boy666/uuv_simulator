"""Reactive pipe-centering guidance using a 360 degree sonar scan and depth."""

from __future__ import annotations

import math
import statistics
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import SetBool


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class PipeGuidance(Node):
    """Outer loop: sonar/depth observations to desired body velocities."""

    def __init__(self) -> None:
        super().__init__("pipe_guidance")
        self._declare_parameters()
        self.enabled = bool(self.get_parameter("enabled").value)
        self.scan: Optional[LaserScan] = None
        self.odom: Optional[Odometry] = None
        self.scan_time: Optional[float] = None
        self.odom_time: Optional[float] = None
        self.filtered_heading = 0.0

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.enabled_pub = self.create_publisher(Bool, "control_enabled", 10)
        self.debug_pub = self.create_publisher(
            Float64MultiArray, "pipe_guidance/diagnostics", 10
        )
        self.create_subscription(
            LaserScan, "sonar", self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "odom", self._odom_callback, qos_profile_sensor_data
        )
        self.create_service(SetBool, "enable_pipe_following", self._enable_callback)
        rate = float(self.get_parameter("control_rate").value)
        self.timer = self.create_timer(1.0 / rate, self._control)
        self.get_logger().info(
            "Pipe guidance ready; use enable_pipe_following or enabled:=true to start"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "enabled": False,
            "target_depth_z": -4.0,
            "sensor_timeout": 0.75,
            "control_rate": 10.0,
            "cruise_speed": 0.22,
            "minimum_turn_speed": 0.03,
            "caution_distance": 1.0,
            "stop_distance": 0.38,
            "side_stop_distance": 0.16,
            "sonar_no_return_range": 20.0,
            "sonar_sensor_y_offset": -0.108,
            "side_sector_half_width_deg": 8.0,
            "forward_sector_half_width_deg": 6.0,
            "heading_search_limit_deg": 110.0,
            "heading_window_half_width_deg": 5.0,
            "heading_search_step_deg": 2.0,
            "heading_penalty_per_rad": 0.35,
            "heading_filter_alpha": 0.35,
            "center_gain": 0.75,
            "max_sway_speed": 0.18,
            "depth_gain": 1.0,
            "max_heave_speed": 0.35,
            "heading_gain": 0.9,
            "max_yaw_rate": 0.65,
            "stop_at_goal": True,
            "goal_x": 8.5,
            "goal_y": 7.0,
            "goal_tolerance": 0.35,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _scan_callback(self, message: LaserScan) -> None:
        self.scan = message
        self.scan_time = self._now()

    def _odom_callback(self, message: Odometry) -> None:
        self.odom = message
        self.odom_time = self._now()

    def _enable_callback(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self.enabled = bool(request.data)
        if not self.enabled:
            self.cmd_pub.publish(Twist())
        response.success = True
        response.message = "pipe following enabled" if self.enabled else "pipe following stopped"
        self.get_logger().info(response.message)
        return response

    def _range_value(self, raw: float) -> float:
        assert self.scan is not None
        substitute = float(self.get_parameter("sonar_no_return_range").value)
        maximum = self.scan.range_max if self.scan.range_max > 0.0 else substitute
        if not math.isfinite(raw):
            return min(maximum, substitute)
        if raw < self.scan.range_min:
            return self.scan.range_min
        return min(raw, maximum, substitute)

    def _sector(self, center: float, half_width: float) -> List[float]:
        assert self.scan is not None
        values: List[float] = []
        for index, raw in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + index * self.scan.angle_increment
            if abs(wrap_pi(angle - center)) <= half_width:
                values.append(self._range_value(raw))
        return values

    def _sector_median(self, center: float, half_width: float) -> float:
        values = self._sector(center, half_width)
        return statistics.median(values) if values else 0.0

    def _sector_lower_quartile(self, center: float, half_width: float) -> float:
        values = sorted(self._sector(center, half_width))
        if not values:
            return 0.0
        return values[int(0.25 * (len(values) - 1))]

    def _open_heading(self) -> float:
        limit = math.radians(float(self.get_parameter("heading_search_limit_deg").value))
        window = math.radians(
            float(self.get_parameter("heading_window_half_width_deg").value)
        )
        step = math.radians(float(self.get_parameter("heading_search_step_deg").value))
        penalty = float(self.get_parameter("heading_penalty_per_rad").value)
        best_heading = 0.0
        best_score = -math.inf
        candidate = -limit
        while candidate <= limit + 1.0e-9:
            clearance = self._sector_lower_quartile(candidate, window)
            score = clearance - penalty * abs(candidate)
            if score > best_score:
                best_score = score
                best_heading = candidate
            candidate += step
        alpha = float(self.get_parameter("heading_filter_alpha").value)
        self.filtered_heading = wrap_pi(
            self.filtered_heading
            + alpha * wrap_pi(best_heading - self.filtered_heading)
        )
        return self.filtered_heading

    def _sensors_fresh(self) -> bool:
        if self.scan_time is None or self.odom_time is None:
            return False
        timeout = float(self.get_parameter("sensor_timeout").value)
        now = self._now()
        return now - self.scan_time <= timeout and now - self.odom_time <= timeout

    def _control(self) -> None:
        if self.enabled and self.odom is not None and bool(
            self.get_parameter("stop_at_goal").value
        ):
            dx = self.odom.pose.pose.position.x - float(
                self.get_parameter("goal_x").value
            )
            dy = self.odom.pose.pose.position.y - float(
                self.get_parameter("goal_y").value
            )
            if math.hypot(dx, dy) <= float(
                self.get_parameter("goal_tolerance").value
            ):
                self.enabled = False
                self.cmd_pub.publish(Twist())
                self.get_logger().info("Pipeline goal reached; controller stopped")

        enabled_message = Bool()
        enabled_message.data = self.enabled and self._sensors_fresh()
        self.enabled_pub.publish(enabled_message)
        if not self.enabled or not self._sensors_fresh():
            self.cmd_pub.publish(Twist())
            return
        assert self.scan is not None and self.odom is not None

        side_half = math.radians(
            float(self.get_parameter("side_sector_half_width_deg").value)
        )
        forward_half = math.radians(
            float(self.get_parameter("forward_sector_half_width_deg").value)
        )
        left = self._sector_median(math.pi / 2.0, side_half)
        right = self._sector_median(-math.pi / 2.0, side_half)
        forward = self._sector_lower_quartile(0.0, forward_half)
        heading_error = self._open_heading()

        # Correct the Ping360's -0.108 m lateral mounting offset so that the
        # vehicle center, rather than the sonar head, follows the pipe center.
        center_error = (
            0.5 * (left - right)
            + float(self.get_parameter("sonar_sensor_y_offset").value)
        )
        depth_error = (
            float(self.get_parameter("target_depth_z").value)
            - self.odom.pose.pose.position.z
        )

        stop = float(self.get_parameter("stop_distance").value)
        caution = float(self.get_parameter("caution_distance").value)
        cruise = float(self.get_parameter("cruise_speed").value)
        if forward <= stop:
            speed_scale = 0.0
        else:
            speed_scale = min(1.0, (forward - stop) / max(caution - stop, 1.0e-6))
        speed_scale *= max(0.15, math.cos(abs(heading_error)))
        surge = cruise * speed_scale
        if abs(heading_error) > math.radians(35.0) and surge > 0.0:
            surge = max(
                surge, float(self.get_parameter("minimum_turn_speed").value)
            )
        if min(left, right) < float(
            self.get_parameter("side_stop_distance").value
        ):
            surge = 0.0

        command = Twist()
        command.linear.x = surge
        command.linear.y = clamp(
            float(self.get_parameter("center_gain").value) * center_error,
            float(self.get_parameter("max_sway_speed").value),
        )
        command.linear.z = clamp(
            float(self.get_parameter("depth_gain").value) * depth_error,
            float(self.get_parameter("max_heave_speed").value),
        )
        command.angular.z = clamp(
            float(self.get_parameter("heading_gain").value) * heading_error,
            float(self.get_parameter("max_yaw_rate").value),
        )
        self.cmd_pub.publish(command)

        debug = Float64MultiArray()
        debug.data = [
            left,
            right,
            forward,
            center_error,
            depth_error,
            heading_error,
            command.linear.x,
            command.linear.y,
            command.linear.z,
            command.angular.z,
        ]
        self.debug_pub.publish(debug)

    def stop(self) -> None:
        for _ in range(5):
            self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PipeGuidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
