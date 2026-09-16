"""BlueROV2 thrust allocation using the pseudoinverse of tau = B f."""

from __future__ import annotations

from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Wrench
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float64, Float64MultiArray


DEFAULT_MATRIX = [
    0.7071068, 0.7071068, 0.7071068, 0.7071068, 0.0, 0.0,
    0.7071068, -0.7071068, -0.7071068, 0.7071068, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, -1.0,
    0.0, 0.0, 0.0, 0.0, -0.1105, -0.1105,
    0.0, 0.0, 0.0, 0.0, -0.0025, 0.0025,
    0.1665, -0.1665, 0.1750, -0.1750, 0.0, 0.0,
]


class ThrusterAllocator(Node):
    def __init__(self) -> None:
        super().__init__("thruster_allocator")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("wrench_timeout", 0.5)
        self.declare_parameter("max_thrust", 8.0)
        self.declare_parameter("allocation_matrix", DEFAULT_MATRIX)

        flat_matrix = list(self.get_parameter("allocation_matrix").value)
        if len(flat_matrix) != 36:
            raise ValueError("allocation_matrix must contain 36 row-major values")
        self.matrix = np.asarray(flat_matrix, dtype=float).reshape((6, 6))
        self.pseudoinverse = np.linalg.pinv(self.matrix)
        self.enabled = False
        self.wrench: Optional[Wrench] = None
        self.wrench_time: Optional[float] = None

        self.thruster_publishers = [
            self.create_publisher(Float64, f"cmd_thruster{index}", 10)
            for index in range(1, 7)
        ]
        self.debug_pub = self.create_publisher(
            Float64MultiArray, "thruster_allocator/commands", 10
        )
        self.create_subscription(Wrench, "cmd_wrench", self._wrench_callback, 10)
        self.create_subscription(Bool, "control_enabled", self._enabled_callback, 10)
        rate = float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"Thruster allocator ready; B rank={np.linalg.matrix_rank(self.matrix)}"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _wrench_callback(self, message: Wrench) -> None:
        self.wrench = message
        self.wrench_time = self._now()

    def _enabled_callback(self, message: Bool) -> None:
        self.enabled = bool(message.data)

    def _commands(self) -> np.ndarray:
        timeout = float(self.get_parameter("wrench_timeout").value)
        if (
            not self.enabled
            or self.wrench is None
            or self.wrench_time is None
            or self._now() - self.wrench_time > timeout
        ):
            return np.zeros(6)
        wrench = np.asarray(
            [
                self.wrench.force.x,
                self.wrench.force.y,
                self.wrench.force.z,
                self.wrench.torque.x,
                self.wrench.torque.y,
                self.wrench.torque.z,
            ],
            dtype=float,
        )
        commands = self.pseudoinverse @ wrench
        max_thrust = float(self.get_parameter("max_thrust").value)
        peak = float(np.max(np.abs(commands)))
        if peak > max_thrust:
            commands *= max_thrust / peak
        return commands

    def _send(self, commands: np.ndarray) -> None:
        for publisher, value in zip(self.thruster_publishers, commands):
            message = Float64()
            message.data = float(value)
            publisher.publish(message)
        debug = Float64MultiArray()
        debug.data = [float(value) for value in commands]
        self.debug_pub.publish(debug)

    def _publish(self) -> None:
        self._send(self._commands())

    def stop(self) -> None:
        zero = np.zeros(6)
        for _ in range(5):
            self._send(zero)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ThrusterAllocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
