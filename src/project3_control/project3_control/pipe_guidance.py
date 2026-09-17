"""Reactive pipe-centering guidance using a 360 degree sonar scan and depth."""

from __future__ import annotations

import math
import statistics
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
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
        self.visual_guidance: Optional[List[float]] = None
        self.visual_time: Optional[float] = None
        self.visual_obstacle_hits = 0
        self.last_obstacle_source = "NONE"
        self.last_dvl_valid_time: Optional[float] = None
        self.dvl_currently_valid = False
        self.dvl_altitude: Optional[float] = None
        self.dvl_altitude_time: Optional[float] = None
        self.last_svin_valid_time: Optional[float] = None
        self.filtered_heading = 0.0
        self.planner_state = "CRUISE"
        self.planner_state_since = self._now()
        self.clear_since: Optional[float] = None
        self.avoid_direction = 1.0
        self.avoid_entry_yaw = 0.0
        self.avoid_entry_position = (0.0, 0.0)
        self.progress_anchor: Optional[Tuple[float, float]] = None
        self.progress_anchor_time = self._now()
        self.last_commanded_surge = 0.0
        self.route_progress = 0
        values = list(self.get_parameter("reference_path_xyz").value)
        if len(values) % 3 != 0:
            raise ValueError("reference_path_xyz length must be divisible by three")
        self.reference_route = [
            (float(values[index]), float(values[index + 1]), float(values[index + 2]))
            for index in range(0, len(values), 3)
        ]

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.enabled_pub = self.create_publisher(Bool, "control_enabled", 10)
        self.debug_pub = self.create_publisher(
            Float64MultiArray, "pipe_guidance/diagnostics", 10
        )
        self.planner_state_pub = self.create_publisher(
            String, "pipe_guidance/planner_state", 10
        )
        self.obstacle_source_pub = self.create_publisher(
            String, "pipe_guidance/obstacle_source", 10
        )
        self.create_subscription(
            LaserScan, "sonar", self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "odom", self._odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Float64MultiArray,
            "/vision/corridor_guidance",
            self._visual_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(Bool, "/dvl/valid", self._dvl_callback, 10)
        self.create_subscription(Float64, "/dvl/altitude", self._dvl_altitude_callback, 10)
        self.create_subscription(Bool, "/svin/tracking_ok", self._svin_callback, 10)
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
            "obstacle_sector_half_width_deg": 22.0,
            "obstacle_trigger_distance": 0.72,
            "sonar_obstacle_trigger_distance": 0.72,
            "obstacle_clear_distance": 1.05,
            "avoidance_turn_angle_deg": 70.0,
            "avoidance_turn_rate": 0.30,
            "avoidance_creep_speed": 0.025,
            "avoidance_bypass_speed": 0.08,
            "avoidance_bypass_heading_offset_deg": 15.0,
            "avoidance_min_duration": 1.0,
            "avoidance_clear_hold": 0.6,
            "avoidance_side_clear_distance": 0.38,
            "avoidance_pass_distance": 1.15,
            "max_avoidance_heading_change_deg": 105.0,
            "reacquire_speed": 0.06,
            "reacquire_heading_tolerance_deg": 12.0,
            "reacquire_min_turn_rate": 0.16,
            "stuck_timeout": 3.0,
            "stuck_min_command": 0.08,
            "stuck_min_progress": 0.06,
            "recovery_reverse_speed": 0.07,
            "recovery_reverse_duration": 1.2,
            "center_gain": 0.75,
            "max_sway_speed": 0.18,
            "depth_gain": 1.0,
            "max_heave_speed": 0.35,
            "heading_gain": 0.9,
            "max_yaw_rate": 0.65,
            "use_reference_route": False,
            "use_visual_corridor": True,
            "use_visual_avoidance": True,
            "visual_timeout": 0.75,
            "visual_min_confidence": 0.08,
            "visual_range_min_confidence": 0.20,
            "visual_obstacle_trigger_distance": 0.90,
            "visual_obstacle_confirmations": 3,
            "visual_decision_weight": 0.45,
            "visual_no_return_range": 8.0,
            "visual_heading_gain": 0.50,
            "visual_vertical_gain": 0.35,
            "minimum_dvl_altitude": 0.30,
            "hard_dvl_altitude": 0.20,
            "hard_clearance_heave_speed": 0.12,
            "dvl_altitude_gain": 3.0,
            "max_perception_descent_speed": 0.12,
            "dvl_loss_recovery_delay": 0.5,
            "dvl_loss_heave_speed": 0.06,
            "navigation_health_timeout": 1.0,
            "max_navigation_xy_variance": 4.0,
            "route_lookahead": 0.9,
            "route_depth_lookahead": 0.15,
            "route_heading_weight": 0.65,
            "reference_path_xyz": [
                -8.5, -5.0, -4.0,
                -3.0, -5.0, -4.0,
                0.0, -2.0, -4.0,
                0.0, 0.0, -5.3,
                0.0, 1.0, -5.3,
                0.0, 4.0, -3.2,
                3.0, 7.0, -3.2,
                8.5, 7.0, -3.2,
            ],
            "stop_at_goal": False,
            "goal_x": 8.5,
            "goal_y": 7.0,
            "goal_z": -3.2,
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

    def _visual_callback(self, message: Float64MultiArray) -> None:
        if len(message.data) >= 4:
            self.visual_guidance = list(message.data)
            self.visual_time = self._now()
            if len(message.data) >= 8 and message.data[7] >= float(
                self.get_parameter("visual_range_min_confidence").value
            ):
                if message.data[4] <= float(
                    self.get_parameter("visual_obstacle_trigger_distance").value
                ):
                    self.visual_obstacle_hits += 1
                elif message.data[4] >= float(
                    self.get_parameter("obstacle_clear_distance").value
                ):
                    self.visual_obstacle_hits = 0

    def _dvl_callback(self, message: Bool) -> None:
        self.dvl_currently_valid = bool(message.data)
        if message.data:
            self.last_dvl_valid_time = self._now()

    def _dvl_altitude_callback(self, message: Float64) -> None:
        if math.isfinite(message.data):
            self.dvl_altitude = float(message.data)
            self.dvl_altitude_time = self._now()

    def _svin_callback(self, message: Bool) -> None:
        if message.data:
            self.last_svin_valid_time = self._now()

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

    def _best_turn_direction(self) -> float:
        """Choose and latch the clearer side using only the current sonar scan."""
        window = math.radians(
            float(self.get_parameter("heading_window_half_width_deg").value)
        )
        step = math.radians(float(self.get_parameter("heading_search_step_deg").value))
        limit = math.radians(float(self.get_parameter("heading_search_limit_deg").value))
        start = math.radians(25.0)

        def best_clearance(sign: float) -> float:
            candidate = start
            best = 0.0
            while candidate <= limit + 1.0e-9:
                best = max(best, self._sector_lower_quartile(sign * candidate, window))
                candidate += step
            return best

        left_score = best_clearance(1.0)
        right_score = best_clearance(-1.0)
        if self._visual_obstacle_fresh():
            assert self.visual_guidance is not None
            weight = float(self.get_parameter("visual_decision_weight").value)
            score_limit = 3.0
            left_score = (1.0 - weight) * min(left_score, score_limit) + weight * min(
                float(self.visual_guidance[5]), score_limit
            )
            right_score = (1.0 - weight) * min(right_score, score_limit) + weight * min(
                float(self.visual_guidance[6]), score_limit
            )
        # Keep the previous choice for an almost symmetric T-junction.  This
        # avoids left/right oscillation from measurement noise.
        if abs(left_score - right_score) < 0.12:
            return self.avoid_direction
        return 1.0 if left_score > right_score else -1.0

    def _set_planner_state(self, state: str) -> None:
        if state == self.planner_state:
            return
        previous = self.planner_state
        self.planner_state = state
        self.planner_state_since = self._now()
        self.clear_since = None
        self.get_logger().info(f"Local planner: {previous} -> {state}")

    def _update_progress_watchdog(self) -> bool:
        """Return true when commanded forward motion produced no XY progress."""
        assert self.odom is not None
        position = self.odom.pose.pose.position
        current = (float(position.x), float(position.y))
        now = self._now()
        minimum_command = float(self.get_parameter("stuck_min_command").value)
        if self.last_commanded_surge < minimum_command:
            self.progress_anchor = current
            self.progress_anchor_time = now
            return False
        if self.progress_anchor is None:
            self.progress_anchor = current
            self.progress_anchor_time = now
            return False
        if math.dist(self.progress_anchor, current) >= float(
            self.get_parameter("stuck_min_progress").value
        ):
            self.progress_anchor = current
            self.progress_anchor_time = now
            return False
        return now - self.progress_anchor_time >= float(
            self.get_parameter("stuck_timeout").value
        )

    def _sensors_fresh(self) -> bool:
        if self.scan_time is None or self.odom_time is None:
            return False
        timeout = float(self.get_parameter("sensor_timeout").value)
        now = self._now()
        if not (now - self.scan_time <= timeout and now - self.odom_time <= timeout):
            return False
        health_timeout = float(self.get_parameter("navigation_health_timeout").value)
        dvl_healthy = (
            self.last_dvl_valid_time is not None
            and now - self.last_dvl_valid_time <= health_timeout
        )
        svin_healthy = (
            self.last_svin_valid_time is not None
            and now - self.last_svin_valid_time <= health_timeout
        )
        if not (dvl_healthy or svin_healthy):
            return False
        if self.odom is not None:
            maximum = float(self.get_parameter("max_navigation_xy_variance").value)
            if max(self.odom.pose.covariance[0], self.odom.pose.covariance[7]) > maximum:
                return False
        return True

    def _visual_fresh(self) -> bool:
        return (
            self.visual_guidance is not None
            and self.visual_time is not None
            and self._now() - self.visual_time
            <= float(self.get_parameter("visual_timeout").value)
            and self.visual_guidance[2]
            >= float(self.get_parameter("visual_min_confidence").value)
        )

    def _visual_obstacle_fresh(self) -> bool:
        return (
            bool(self.get_parameter("use_visual_avoidance").value)
            and self.visual_guidance is not None
            and len(self.visual_guidance) >= 8
            and self.visual_time is not None
            and self._now() - self.visual_time
            <= float(self.get_parameter("visual_timeout").value)
            and self.visual_guidance[7]
            >= float(self.get_parameter("visual_range_min_confidence").value)
        )

    def _dvl_recently_valid(self) -> bool:
        return (
            self.last_dvl_valid_time is not None
            and self._now() - self.last_dvl_valid_time
            <= float(self.get_parameter("dvl_loss_recovery_delay").value)
        )

    def _vehicle_yaw(self) -> float:
        assert self.odom is not None
        q = self.odom.pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _route_point_ahead(self, lookahead: float) -> Tuple[float, float, float]:
        """Project onto the ordered 3-D route and walk forward by lookahead."""
        assert self.odom is not None
        position = self.odom.pose.pose.position
        current = (position.x, position.y, position.z)
        if len(self.reference_route) < 2:
            return (current[0], current[1], float(self.get_parameter("target_depth_z").value))

        # Advance at most one segment and only near its endpoint.  This prevents
        # a one-frame VIO initialization jump from skipping several route legs.
        current_end = self.reference_route[self.route_progress + 1]
        horizontal_distance_to_end = math.hypot(
            current[0] - current_end[0], current[1] - current_end[1]
        )
        if (
            self.route_progress < len(self.reference_route) - 2
            and horizontal_distance_to_end < 0.75
        ):
            self.route_progress += 1

        best_index = self.route_progress
        start = self.reference_route[best_index]
        end = self.reference_route[best_index + 1]
        vector = tuple(end[axis] - start[axis] for axis in range(3))
        # Project by XY only.  Including Z creates positive feedback in a
        # vertical bend: a depth error falsely advances the route depth target.
        horizontal_length_squared = vector[0] * vector[0] + vector[1] * vector[1]
        best_fraction = 0.0
        if horizontal_length_squared > 1.0e-9:
            best_fraction = max(
                0.0,
                min(
                    1.0,
                    ((current[0] - start[0]) * vector[0] + (current[1] - start[1]) * vector[1])
                    / horizontal_length_squared,
                ),
            )
        segment_length = math.dist(start, end)
        remaining = max(0.0, lookahead)
        distance_on_segment = best_fraction * segment_length
        index = best_index

        while index < len(self.reference_route) - 1:
            start = self.reference_route[index]
            end = self.reference_route[index + 1]
            segment_length = math.dist(start, end)
            available = segment_length - distance_on_segment
            if remaining <= available or index == len(self.reference_route) - 2:
                fraction = min(1.0, (distance_on_segment + remaining) / max(segment_length, 1.0e-9))
                return tuple(start[axis] + fraction * (end[axis] - start[axis]) for axis in range(3))
            remaining -= available
            index += 1
            distance_on_segment = 0.0
        return self.reference_route[-1]

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
            dz = self.odom.pose.pose.position.z - float(
                self.get_parameter("goal_z").value
            )
            if math.sqrt(dx * dx + dy * dy + dz * dz) <= float(
                self.get_parameter("goal_tolerance").value
            ):
                self.enabled = False
                self.cmd_pub.publish(Twist())
                self.get_logger().info("Pipeline goal reached; controller stopped")

        enabled_message = Bool()
        enabled_message.data = self.enabled and self._sensors_fresh()
        self.enabled_pub.publish(enabled_message)
        if not self.enabled or not self._sensors_fresh():
            status = String()
            status.data = "DISABLED" if not self.enabled else "SENSOR_HOLD"
            self.planner_state_pub.publish(status)
            self.last_commanded_surge = 0.0
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
        obstacle_half = math.radians(
            float(self.get_parameter("obstacle_sector_half_width_deg").value)
        )
        sonar_obstacle_forward = self._sector_lower_quartile(0.0, obstacle_half)
        visual_front = float(self.get_parameter("visual_no_return_range").value)
        visual_left = visual_front
        visual_right = visual_front
        visual_range_confidence = 0.0
        visual_range_fresh = self._visual_obstacle_fresh()
        if visual_range_fresh:
            assert self.visual_guidance is not None
            visual_front = float(self.visual_guidance[4])
            visual_left = float(self.visual_guidance[5])
            visual_right = float(self.visual_guidance[6])
            visual_range_confidence = float(self.visual_guidance[7])
        obstacle_forward = min(sonar_obstacle_forward, visual_front)
        visual_obstacle_confirmed = (
            visual_range_fresh
            and self.visual_obstacle_hits
            >= int(self.get_parameter("visual_obstacle_confirmations").value)
        )
        sonar_obstacle_detected = sonar_obstacle_forward <= float(
            self.get_parameter("sonar_obstacle_trigger_distance").value
        )
        obstacle_detected = sonar_obstacle_detected or visual_obstacle_confirmed
        sonar_heading_error = self._open_heading()
        heading_error = sonar_heading_error
        target_depth = float(self.get_parameter("target_depth_z").value)
        route_heading_error = 0.0
        if bool(self.get_parameter("use_reference_route").value):
            route_target = self._route_point_ahead(
                float(self.get_parameter("route_lookahead").value)
            )
            depth_target = self._route_point_ahead(
                float(self.get_parameter("route_depth_lookahead").value)
            )
            position = self.odom.pose.pose.position
            desired_yaw = math.atan2(
                route_target[1] - position.y, route_target[0] - position.x
            )
            route_heading_error = wrap_pi(desired_yaw - self._vehicle_yaw())
            weight = float(self.get_parameter("route_heading_weight").value)
            heading_error = math.atan2(
                (1.0 - weight) * math.sin(sonar_heading_error)
                + weight * math.sin(route_heading_error),
                (1.0 - weight) * math.cos(sonar_heading_error)
                + weight * math.cos(route_heading_error),
            )
            target_depth = depth_target[2]

        # Formal route-independent mode: use the far/open region of calibrated
        # stereo disparity to follow horizontal and vertical bends.  No pipe
        # radius, centerline or target world depth is supplied to this branch.
        visual_vertical_error = 0.0
        visual_horizontal_error = 0.0
        if bool(self.get_parameter("use_visual_corridor").value) and self._visual_fresh():
            assert self.visual_guidance is not None
            visual_horizontal_error = float(self.visual_guidance[0])
            visual_vertical_error = float(self.visual_guidance[1])
            if not bool(self.get_parameter("use_reference_route").value):
                heading_error = wrap_pi(
                    sonar_heading_error
                    - float(self.get_parameter("visual_heading_gain").value)
                    * visual_horizontal_error
                )

        # Correct the Ping360's -0.108 m lateral mounting offset so that the
        # vehicle center, rather than the sonar head, follows the pipe center.
        center_error = (
            0.5 * (left - right)
            + float(self.get_parameter("sonar_sensor_y_offset").value)
        )
        if bool(self.get_parameter("use_reference_route").value):
            depth_error = target_depth - self.odom.pose.pose.position.z
        elif self._visual_fresh():
            depth_error = (
                float(self.get_parameter("visual_vertical_gain").value)
                * visual_vertical_error
            )
        else:
            depth_error = 0.0
        # A real DVL supplies altitude together with bottom-track velocity.
        # Enforce only a local minimum clearance; no floor/world Z is known.
        if (
            not bool(self.get_parameter("use_reference_route").value)
            and self.dvl_altitude is not None
            and self.dvl_altitude_time is not None
            and self._now() - self.dvl_altitude_time
            <= float(self.get_parameter("sensor_timeout").value)
        ):
            clearance_error = max(
                0.0,
                float(self.get_parameter("minimum_dvl_altitude").value)
                - self.dvl_altitude,
            )
            depth_error += (
                float(self.get_parameter("dvl_altitude_gain").value)
                * clearance_error
            )
            if self.dvl_altitude < float(self.get_parameter("hard_dvl_altitude").value):
                depth_error = max(
                    depth_error,
                    float(self.get_parameter("hard_clearance_heave_speed").value),
                )
        if not bool(self.get_parameter("use_reference_route").value):
            # Never descend without a current acoustic clearance measurement.
            if not self._dvl_recently_valid():
                depth_error = max(
                    float(self.get_parameter("dvl_loss_heave_speed").value),
                    depth_error,
                )
            depth_error = max(
                -float(self.get_parameter("max_perception_descent_speed").value),
                depth_error,
            )

        stop = float(self.get_parameter("stop_distance").value)
        caution = float(self.get_parameter("caution_distance").value)
        cruise = float(self.get_parameter("cruise_speed").value)

        # Sensor-driven local planner.  A wide forward sonar cone triggers a
        # latched avoidance turn before the narrow collision-stop cone is hit.
        # The decision does not consume world coordinates or pipe geometry.
        now = self._now()
        if self.planner_state == "CRUISE":
            if obstacle_detected:
                if sonar_obstacle_detected and visual_obstacle_confirmed:
                    self.last_obstacle_source = "BOTH"
                elif visual_obstacle_confirmed:
                    self.last_obstacle_source = "VISION"
                else:
                    self.last_obstacle_source = "SONAR"
                self.get_logger().info(
                    "Obstacle trigger source=%s sonar=%.3f m vision=%.3f m"
                    % (
                        self.last_obstacle_source,
                        sonar_obstacle_forward,
                        visual_front,
                    )
                )
                self.avoid_direction = self._best_turn_direction()
                self.avoid_entry_yaw = self._vehicle_yaw()
                position = self.odom.pose.pose.position
                self.avoid_entry_position = (float(position.x), float(position.y))
                self._set_planner_state(
                    "AVOID_LEFT" if self.avoid_direction > 0.0 else "AVOID_RIGHT"
                )
            elif self._update_progress_watchdog():
                self.avoid_direction = self._best_turn_direction()
                self.avoid_entry_yaw = self._vehicle_yaw()
                position = self.odom.pose.pose.position
                self.avoid_entry_position = (float(position.x), float(position.y))
                self._set_planner_state("RECOVER_REVERSE")
        elif self.planner_state == "RECOVER_REVERSE":
            if now - self.planner_state_since >= float(
                self.get_parameter("recovery_reverse_duration").value
            ):
                self._set_planner_state(
                    "AVOID_LEFT" if self.avoid_direction > 0.0 else "AVOID_RIGHT"
                )
        elif self.planner_state in ("AVOID_LEFT", "AVOID_RIGHT"):
            minimum_duration_met = now - self.planner_state_since >= float(
                self.get_parameter("avoidance_min_duration").value
            )
            # The wide sectors deliberately include side surfaces and remain
            # conservative while turning.  The narrow forward sonar confirms
            # that the new travel direction is open; stereo must also be above
            # the emergency stop distance when it has a valid range.
            path_clear = forward >= float(
                self.get_parameter("obstacle_clear_distance").value
            ) and (
                not visual_range_fresh or visual_front > stop
            )
            heading_change = abs(wrap_pi(self._vehicle_yaw() - self.avoid_entry_yaw))
            maximum_change = math.radians(
                float(self.get_parameter("max_avoidance_heading_change_deg").value)
            )
            if minimum_duration_met and path_clear:
                self._set_planner_state(
                    "BYPASS_LEFT" if self.avoid_direction > 0.0 else "BYPASS_RIGHT"
                )
            elif heading_change >= maximum_change:
                self.avoid_direction *= -1.0
                self._set_planner_state("RECOVER_REVERSE")
        elif self.planner_state in ("BYPASS_LEFT", "BYPASS_RIGHT"):
            # Progress is measured by fused odometry along the corridor heading
            # captured before avoidance.  It is relative distance only: no map
            # coordinate or obstacle ground truth enters the planner.
            position = self.odom.pose.pose.position
            entry_dx = float(position.x) - self.avoid_entry_position[0]
            entry_dy = float(position.y) - self.avoid_entry_position[1]
            bypass_progress = (
                entry_dx * math.cos(self.avoid_entry_yaw)
                + entry_dy * math.sin(self.avoid_entry_yaw)
            )
            passed_obstacle = bypass_progress >= float(
                self.get_parameter("avoidance_pass_distance").value
            )
            obstacle_side = right if self.avoid_direction > 0.0 else left
            side_clear = obstacle_side >= float(
                self.get_parameter("avoidance_side_clear_distance").value
            )
            if passed_obstacle and side_clear:
                if self.clear_since is None:
                    self.clear_since = now
                elif now - self.clear_since >= float(
                    self.get_parameter("avoidance_clear_hold").value
                ):
                    self._set_planner_state("REACQUIRE_CORRIDOR")
                    self.filtered_heading = 0.0
                    self.progress_anchor = None
            else:
                self.clear_since = None
        elif self.planner_state == "REACQUIRE_CORRIDOR":
            reacquire_error = wrap_pi(self.avoid_entry_yaw - self._vehicle_yaw())
            aligned = abs(reacquire_error) <= math.radians(
                float(self.get_parameter("reacquire_heading_tolerance_deg").value)
            )
            path_clear = obstacle_forward >= float(
                self.get_parameter("obstacle_trigger_distance").value
            )
            if aligned and path_clear:
                if self.clear_since is None:
                    self.clear_since = now
                elif now - self.clear_since >= float(
                    self.get_parameter("avoidance_clear_hold").value
                ):
                    self._set_planner_state("CRUISE")
                    self.progress_anchor = None
            else:
                self.clear_since = None

        collision_forward = min(forward, visual_front) if visual_range_fresh else forward
        if collision_forward <= stop:
            speed_scale = 0.0
        else:
            speed_scale = min(
                1.0,
                (collision_forward - stop) / max(caution - stop, 1.0e-6),
            )
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
        if (
            not bool(self.get_parameter("use_reference_route").value)
            and not self._dvl_recently_valid()
        ):
            surge = 0.0

        if self.planner_state == "RECOVER_REVERSE":
            surge = -float(self.get_parameter("recovery_reverse_speed").value)
            heading_error = self.avoid_direction * math.radians(25.0)
        elif self.planner_state in ("AVOID_LEFT", "AVOID_RIGHT"):
            heading_error = self.avoid_direction * math.radians(
                float(self.get_parameter("avoidance_turn_angle_deg").value)
            )
            surge = 0.0 if obstacle_forward <= stop else min(
                surge, float(self.get_parameter("avoidance_creep_speed").value)
            )
        elif self.planner_state in ("BYPASS_LEFT", "BYPASS_RIGHT"):
            bypass_yaw = self.avoid_entry_yaw + self.avoid_direction * math.radians(
                float(self.get_parameter("avoidance_bypass_heading_offset_deg").value)
            )
            heading_error = wrap_pi(bypass_yaw - self._vehicle_yaw())
            surge = (
                min(surge, float(self.get_parameter("avoidance_bypass_speed").value))
                if collision_forward > stop
                else 0.0
            )
        elif self.planner_state == "REACQUIRE_CORRIDOR":
            heading_error = wrap_pi(self.avoid_entry_yaw - self._vehicle_yaw())
            surge = (
                min(surge, float(self.get_parameter("reacquire_speed").value))
                if obstacle_forward > stop
                else 0.0
            )

        command = Twist()
        command.linear.x = surge
        command.linear.y = 0.0 if self.planner_state != "CRUISE" else clamp(
            float(self.get_parameter("center_gain").value) * center_error,
            float(self.get_parameter("max_sway_speed").value),
        )
        command.linear.z = clamp(
            float(self.get_parameter("depth_gain").value) * depth_error,
            float(self.get_parameter("max_heave_speed").value),
        )
        yaw_command = float(self.get_parameter("heading_gain").value) * heading_error
        if (
            self.planner_state in ("AVOID_LEFT", "AVOID_RIGHT")
        ):
            yaw_command = self.avoid_direction * max(
                abs(yaw_command), float(self.get_parameter("avoidance_turn_rate").value)
            )
        elif self.planner_state == "REACQUIRE_CORRIDOR" and abs(heading_error) > math.radians(2.0):
            yaw_command = math.copysign(
                max(
                    abs(yaw_command),
                    float(self.get_parameter("reacquire_min_turn_rate").value),
                ),
                heading_error,
            )
        command.angular.z = clamp(yaw_command, float(self.get_parameter("max_yaw_rate").value))
        self.cmd_pub.publish(command)
        self.last_commanded_surge = command.linear.x

        status = String()
        status.data = self.planner_state
        self.planner_state_pub.publish(status)
        source_status = String()
        source_status.data = self.last_obstacle_source
        self.obstacle_source_pub.publish(source_status)

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
            target_depth,
            route_heading_error,
            float(self.route_progress),
            visual_horizontal_error,
            visual_vertical_error,
            float(self.visual_guidance[2]) if self.visual_guidance is not None else 0.0,
            self.dvl_altitude if self.dvl_altitude is not None else math.nan,
            obstacle_forward,
            {"CRUISE": 0.0, "AVOID_LEFT": 1.0, "AVOID_RIGHT": -1.0,
             "BYPASS_LEFT": 1.5, "BYPASS_RIGHT": -1.5,
             "RECOVER_REVERSE": 2.0, "REACQUIRE_CORRIDOR": 3.0}.get(
                 self.planner_state, 9.0
             ),
            sonar_obstacle_forward,
            visual_front,
            visual_left,
            visual_right,
            visual_range_confidence,
            1.0 if visual_obstacle_confirmed else 0.0,
            1.0 if sonar_obstacle_detected else 0.0,
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
