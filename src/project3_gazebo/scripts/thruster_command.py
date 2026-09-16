#!/usr/bin/env python3
"""Publish synchronized, time-limited manual commands to six BlueROV2 thrusters."""

from __future__ import annotations

import argparse
import time
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ThrusterCommander(Node):
    def __init__(self, namespace: str) -> None:
        super().__init__("project3_thruster_command")
        namespace = namespace.strip("/")
        prefix = f"/{namespace}" if namespace else ""
        self.thruster_publishers = [
            self.create_publisher(Float64, f"{prefix}/cmd_thruster{index}", 10)
            for index in range(1, 7)
        ]

    def publish(self, commands: List[float]) -> None:
        for publisher, value in zip(self.thruster_publishers, commands):
            message = Float64()
            message.data = float(value)
            publisher.publish(message)


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def allocate(args: argparse.Namespace) -> List[float]:
    """Map simple body commands to the upstream BlueROV2 thruster signs.

    Inputs are per-thruster force contributions in newtons, not body wrench.
    This mixer is intended for safe manual checks. The final controller will
    use a calibrated thrust allocation matrix.
    """
    commands = [
        args.surge + args.sway + args.yaw,
        args.surge - args.sway - args.yaw,
        args.surge - args.sway + args.yaw,
        args.surge + args.sway - args.yaw,
        args.heave - args.roll,
        -args.heave - args.roll,
    ]
    return [clamp(value, args.max_force) for value in commands]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Safely command BlueROV2 thrusters. Positive axes follow ROS ENU: "
            "surge=forward, sway=left, heave=up, yaw=counter-clockwise."
        )
    )
    parser.add_argument("--namespace", default="bluerov2")
    parser.add_argument("--surge", type=float, default=0.0, help="Forward contribution [N]")
    parser.add_argument("--sway", type=float, default=0.0, help="Left contribution [N]")
    parser.add_argument("--heave", type=float, default=0.0, help="Up contribution [N]")
    parser.add_argument("--yaw", type=float, default=0.0, help="CCW yaw contribution [N]")
    parser.add_argument("--roll", type=float, default=0.0, help="Positive roll contribution [N]")
    parser.add_argument("--duration", type=float, default=2.0, help="Command duration [s]")
    parser.add_argument("--rate", type=float, default=20.0, help="Publish rate [Hz]")
    parser.add_argument("--max-force", type=float, default=10.0, help="Per-thruster safety clamp [N]")
    parser.add_argument("--stop", action="store_true", help="Publish zero to all thrusters and exit")
    parser.add_argument("--dry-run", action="store_true", help="Print allocation without publishing")
    args = parser.parse_args()
    if args.duration < 0.0:
        parser.error("--duration must be non-negative")
    if args.rate <= 0.0:
        parser.error("--rate must be positive")
    if not 0.0 < args.max_force <= 40.0:
        parser.error("--max-force must be in (0, 40]")
    return args


def main() -> None:
    args = parse_args()
    commands = [0.0] * 6 if args.stop else allocate(args)
    print("thruster commands [N]: " + ", ".join(f"T{i + 1}={v:.3f}" for i, v in enumerate(commands)))
    if args.dry_run:
        return

    rclpy.init()
    node = ThrusterCommander(args.namespace)
    zero = [0.0] * 6
    try:
        # Allow DDS discovery before the first command.
        discovery_end = time.monotonic() + 0.5
        while time.monotonic() < discovery_end:
            rclpy.spin_once(node, timeout_sec=0.05)

        if args.stop:
            for _ in range(5):
                node.publish(zero)
                rclpy.spin_once(node, timeout_sec=0.02)
            return

        end_time = time.monotonic() + args.duration
        period = 1.0 / args.rate
        while rclpy.ok() and time.monotonic() < end_time:
            node.publish(commands)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        # Repeated zeros make shutdown robust to one dropped DDS sample.
        for _ in range(5):
            node.publish(zero)
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
