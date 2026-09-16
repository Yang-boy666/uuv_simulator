"""Small quaternion helpers using ROS (x, y, z, w) ordering."""

from __future__ import annotations

import math
from typing import Iterable, Tuple

import numpy as np


Quaternion = Tuple[float, float, float, float]


def normalize_quaternion(values: Iterable[float]) -> Quaternion:
    q = np.asarray(tuple(values), dtype=float)
    norm = float(np.linalg.norm(q))
    if norm < 1.0e-12:
        return 0.0, 0.0, 0.0, 1.0
    q /= norm
    return tuple(float(value) for value in q)  # type: ignore[return-value]


def quaternion_conjugate(q: Quaternion) -> Quaternion:
    return -q[0], -q[1], -q[2], q[3]


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return normalize_quaternion(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def rotate_vector(q: Quaternion, vector: Iterable[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion(q)
    vector_array = np.asarray(tuple(vector), dtype=float)
    q_vector = np.array([x, y, z], dtype=float)
    return (
        vector_array
        + 2.0 * w * np.cross(q_vector, vector_array)
        + 2.0 * np.cross(q_vector, np.cross(q_vector, vector_array))
    )


def quaternion_to_yaw(q: Quaternion) -> float:
    x, y, z, w = normalize_quaternion(q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
