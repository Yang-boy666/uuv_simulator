"""Body-velocity PI controller producing a commanded body wrench."""

from __future__ import annotations

from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Twist, Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float64MultiArray


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class VelocityController(Node):
    """Inner loop kept separate from perception and thruster allocation."""

    AXES = ("surge", "sway", "heave", "yaw")

    def __init__(self) -> None:
        super().__init__("velocity_controller")
        self._declare_parameters()
        self.command: Optional[Twist] = None
        self.odom: Optional[Odometry] = None
        self.command_time: Optional[float] = None
        self.odom_time: Optional[float] = None
        self.last_control_time: Optional[float] = None
        self.enabled = False
        self.integrals: Dict[str, float] = {axis: 0.0 for axis in self.AXES}

        self.wrench_pub = self.create_publisher(Wrench, "cmd_wrench", 10)
        self.debug_pub = self.create_publisher(
            Float64MultiArray, "velocity_controller/diagnostics", 10
        )
        self.create_subscription(Twist, "cmd_vel", self._command_callback, 10)
        self.create_subscription(
            Odometry, "odom", self._odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(Bool, "control_enabled", self._enabled_callback, 10)
        rate = float(self.get_parameter("control_rate").value)
        self.timer = self.create_timer(1.0 / rate, self._control)
        self.get_logger().info("Velocity PI controller ready")

    def _declare_parameters(self) -> None:
        defaults = {
            "control_rate": 20.0,
            "command_timeout": 0.75,
            "odom_timeout": 0.75,
            "kp_surge": 7.0,
            "ki_surge": 0.8,
            "kp_sway": 7.0,
            "ki_sway": 0.8,
            "kp_heave": 25.0,
            "ki_heave": 6.0,
            "kp_yaw": 4.0,
            "ki_yaw": 0.4,
            "integral_limit": 1.5,
            "max_force_x": 4.0,
            "max_force_y": 3.0,
            "max_force_z": 12.0,
            "max_torque_z": 1.2,
            # Calibrated vehicle-only term: the model is slightly positively
            # buoyant.  This is not an environmental depth or route prior.
            "heave_feedforward": -4.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _command_callback(self, message: Twist) -> None:
        self.command = message
        self.command_time = self._now()

    def _odom_callback(self, message: Odometry) -> None:
        self.odom = message
        self.odom_time = self._now()

    def _enabled_callback(self, message: Bool) -> None:
        self.enabled = bool(message.data)
        if not self.enabled:
            self._reset_integrals()

    def _reset_integrals(self) -> None:
        for axis in self.AXES:
            self.integrals[axis] = 0.0

    def _fresh(self) -> bool:
        if self.command_time is None or self.odom_time is None:
            return False
        now = self._now()
        return (
            now - self.command_time
            <= float(self.get_parameter("command_timeout").value)
            and now - self.odom_time
            <= float(self.get_parameter("odom_timeout").value)
        )

    def _pi(self, axis: str, error: float, dt: float, output_limit: float) -> float:
        integral_limit = float(self.get_parameter("integral_limit").value)
        self.integrals[axis] = clamp(
            self.integrals[axis] + error * dt, integral_limit
        )
        effort = (
            float(self.get_parameter(f"kp_{axis}").value) * error
            + float(self.get_parameter(f"ki_{axis}").value) * self.integrals[axis]
        )
        return clamp(effort, output_limit)

    def _control(self) -> None:
        now = self._now()
        dt = 0.0 if self.last_control_time is None else max(0.0, min(0.2, now - self.last_control_time))
        self.last_control_time = now
        if not self.enabled or not self._fresh():
            self._reset_integrals()
            self.wrench_pub.publish(Wrench())
            return
        assert self.command is not None and self.odom is not None

        measured = self.odom.twist.twist
        errors = {
            "surge": self.command.linear.x - measured.linear.x,
            "sway": self.command.linear.y - measured.linear.y,
            "heave": self.command.linear.z - measured.linear.z,
            "yaw": self.command.angular.z - measured.angular.z,
        }
        wrench = Wrench()
        wrench.force.x = self._pi(
            "surge", errors["surge"], dt, float(self.get_parameter("max_force_x").value)
        )
        wrench.force.y = self._pi(
            "sway", errors["sway"], dt, float(self.get_parameter("max_force_y").value)
        )
        maximum_z = float(self.get_parameter("max_force_z").value)
        wrench.force.z = clamp(
            self._pi("heave", errors["heave"], dt, maximum_z)
            + float(self.get_parameter("heave_feedforward").value),
            maximum_z,
        )
        wrench.torque.z = self._pi(
            "yaw", errors["yaw"], dt, float(self.get_parameter("max_torque_z").value)
        )
        self.wrench_pub.publish(wrench)

        debug = Float64MultiArray()
        debug.data = [
            errors["surge"],
            errors["sway"],
            errors["heave"],
            errors["yaw"],
            wrench.force.x,
            wrench.force.y,
            wrench.force.z,
            wrench.torque.z,
        ]
        self.debug_pub.publish(debug)

    def stop(self) -> None:
        for _ in range(5):
            self.wrench_pub.publish(Wrench())


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = VelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
