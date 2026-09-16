#!/usr/bin/env python3
"""Generate the 24 m x 18 m x 8 m pool with a hollow bent inspection pipe."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple


Point2 = Tuple[float, float]


POOL_LENGTH = 24.0
POOL_WIDTH = 18.0
POOL_DEPTH = 8.0
PIPE_CENTER_Z = -4.0
PIPE_INNER_RADIUS = 0.60
PIPE_WALL_THICKNESS = 0.12
PIPE_SIDES = 8
STRAIGHT_STEP = 0.75
ELBOW_STEPS = 12
PANEL_OVERLAP = 0.06


def line_points(start: Point2, end: Point2, step: float) -> List[Point2]:
    distance = math.dist(start, end)
    count = max(1, math.ceil(distance / step))
    return [
        (
            start[0] + (end[0] - start[0]) * i / count,
            start[1] + (end[1] - start[1]) * i / count,
        )
        for i in range(count + 1)
    ]


def arc_points(
    center: Point2, radius: float, start_angle: float, end_angle: float, count: int
) -> List[Point2]:
    return [
        (
            center[0] + radius * math.cos(start_angle + (end_angle - start_angle) * i / count),
            center[1] + radius * math.sin(start_angle + (end_angle - start_angle) * i / count),
        )
        for i in range(count + 1)
    ]


def append_without_duplicate(route: List[Point2], points: Iterable[Point2]) -> None:
    for point in points:
        if not route or math.dist(route[-1], point) > 1.0e-9:
            route.append(point)


def build_route() -> List[Point2]:
    """Return a continuous route with two horizontal 90 degree elbows."""
    route: List[Point2] = []
    append_without_duplicate(route, line_points((-9.0, -5.0), (-3.0, -5.0), STRAIGHT_STEP))
    append_without_duplicate(
        route,
        arc_points((-3.0, -2.0), 3.0, -math.pi / 2.0, 0.0, ELBOW_STEPS),
    )
    append_without_duplicate(route, line_points((0.0, -2.0), (0.0, 4.0), STRAIGHT_STEP))
    append_without_duplicate(
        route,
        arc_points((3.0, 4.0), 3.0, math.pi, math.pi / 2.0, ELBOW_STEPS),
    )
    append_without_duplicate(route, line_points((3.0, 7.0), (9.0, 7.0), STRAIGHT_STEP))
    return route


def fmt(value: float) -> str:
    if abs(value) < 5.0e-10:
        value = 0.0
    return f"{value:.6f}"


def material(name: str, rgba: Sequence[float]) -> str:
    values = " ".join(fmt(value) for value in rgba)
    return (
        f"<material><ambient>{values}</ambient><diffuse>{values}</diffuse>"
        f"<specular>0.15 0.15 0.15 1</specular></material>"
    )


def box_geometry(size: Sequence[float]) -> str:
    return f"<geometry><box><size>{' '.join(fmt(value) for value in size)}</size></box></geometry>"


def add_box(
    lines: List[str],
    name: str,
    pose: Sequence[float],
    size: Sequence[float],
    rgba: Sequence[float],
    collision: bool = True,
) -> None:
    pose_text = " ".join(fmt(value) for value in pose)
    geometry = box_geometry(size)
    if collision:
        lines.extend(
            [
                f'      <collision name="{name}_collision">',
                f"        <pose>{pose_text}</pose>",
                f"        {geometry}",
                "      </collision>",
            ]
        )
    lines.extend(
        [
            f'      <visual name="{name}_visual">',
            f"        <pose>{pose_text}</pose>",
            f"        {geometry}",
            f"        {material(name, rgba)}",
            "      </visual>",
        ]
    )


def add_emissive_box(
    lines: List[str],
    name: str,
    pose: Sequence[float],
    size: Sequence[float],
    rgba: Sequence[float],
) -> None:
    """Add a collision-free box that stays visible in the dark pipe interior."""
    pose_text = " ".join(fmt(value) for value in pose)
    geometry = box_geometry(size)
    values = " ".join(fmt(value) for value in rgba)
    lines.extend(
        [
            f'      <visual name="{name}_visual">',
            f"        <pose>{pose_text}</pose>",
            f"        {geometry}",
            "        <cast_shadows>false</cast_shadows>",
            "        <material>",
            f"          <ambient>{values}</ambient>",
            f"          <diffuse>{values}</diffuse>",
            f"          <emissive>{values}</emissive>",
            "        </material>",
            "      </visual>",
        ]
    )


def pipe_panels(route: Sequence[Point2]) -> List[str]:
    lines: List[str] = []
    panel_radius = PIPE_INNER_RADIUS + PIPE_WALL_THICKNESS / 2.0
    panel_width = 2.0 * panel_radius * math.tan(math.pi / PIPE_SIDES) + 0.02

    for segment_index, (start, end) in enumerate(zip(route, route[1:])):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy) + PANEL_OVERLAP
        yaw = math.atan2(dy, dx)
        center_x = (start[0] + end[0]) / 2.0
        center_y = (start[1] + end[1]) / 2.0

        for side in range(PIPE_SIDES):
            roll = 2.0 * math.pi * side / PIPE_SIDES
            # Rz(yaw) * Rx(roll) * local_z gives the panel's outward normal.
            radial_x = math.sin(yaw) * math.sin(roll)
            radial_y = -math.cos(yaw) * math.sin(roll)
            radial_z = math.cos(roll)
            x = center_x + panel_radius * radial_x
            y = center_y + panel_radius * radial_y
            z = PIPE_CENTER_Z + panel_radius * radial_z

            at_elbow = 8 <= segment_index < 20 or 28 <= segment_index < 40
            rgba = (0.68, 0.32, 0.08, 0.58) if at_elbow else (0.28, 0.33, 0.36, 0.52)
            add_box(
                lines,
                f"pipe_{segment_index:03d}_{side:02d}",
                (x, y, z, roll, 0.0, yaw),
                (length, panel_width, PIPE_WALL_THICKNESS),
                rgba,
            )

    return lines


def route_markers(route: Sequence[Point2]) -> List[str]:
    lines: List[str] = []
    marker_indices = [1, 8, 14, 20, 28, 34, 40, len(route) - 2]
    for marker_id, route_index in enumerate(marker_indices):
        x, y = route[route_index]
        add_box(
            lines,
            f"route_marker_{marker_id:02d}",
            (x, y, PIPE_CENTER_Z - PIPE_INNER_RADIUS + 0.015, 0.0, 0.0, 0.0),
            (0.20, 0.20, 0.025),
            (0.95, 0.12, 0.05, 1.0),
            collision=False,
        )
    return lines


def visual_feature_markers(route: Sequence[Point2]) -> List[str]:
    """Add non-periodic, high-contrast patches for stereo/VIO evaluation.

    Smooth, repeating pipe panels are a deliberately difficult visual scene.
    These collision-free patches create the textured condition used by the
    normal SVIn experiment.  Removing this model gives the degraded-vision
    experiment without changing the navigation stack.
    """
    lines: List[str] = []
    colors = [
        (0.96, 0.92, 0.10, 1.0),
        (0.08, 0.82, 0.95, 1.0),
        (0.95, 0.12, 0.42, 1.0),
        (0.92, 0.92, 0.92, 1.0),
        (0.18, 0.95, 0.28, 1.0),
    ]
    radial_distance = PIPE_INNER_RADIUS - 0.012

    for index, (start, end) in enumerate(zip(route, route[1:])):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        yaw = math.atan2(dy, dx)
        center_x = 0.5 * (start[0] + end[0])
        center_y = 0.5 * (start[1] + end[1])
        color = colors[(index * 3 + index // 4) % len(colors)]

        # Vary patch dimensions and longitudinal offsets so the visual pattern
        # cannot be mistaken for one perfectly periodic pipe ring.
        length = 0.10 + 0.035 * (index % 4)
        width = 0.08 + 0.025 * ((index * 3) % 5)
        along = (-0.16, 0.0, 0.13)[index % 3]
        offset_x = along * math.cos(yaw)
        offset_y = along * math.sin(yaw)

        # Floor marker.
        add_box(
            lines,
            f"vio_floor_{index:03d}",
            (
                center_x + offset_x,
                center_y + offset_y,
                PIPE_CENTER_Z - radial_distance,
                0.0,
                0.0,
                yaw,
            ),
            (length, width, 0.018),
            color,
            collision=False,
        )

        # Alternating side-wall marker.  roll rotates the thin box normal onto
        # the pipe radial direction.
        side_roll = math.pi / 2.0 if index % 2 == 0 else -math.pi / 2.0
        radial_x = math.sin(yaw) * math.sin(side_roll)
        radial_y = -math.cos(yaw) * math.sin(side_roll)
        add_box(
            lines,
            f"vio_side_{index:03d}",
            (
                center_x + radial_distance * radial_x - 0.5 * offset_x,
                center_y + radial_distance * radial_y - 0.5 * offset_y,
                PIPE_CENTER_Z + 0.12 * ((index % 3) - 1),
                side_roll,
                0.0,
                yaw,
            ),
            (width, length, 0.018),
            colors[(index + 2) % len(colors)],
            collision=False,
        )

        # A smaller ceiling patch every second segment breaks rotational
        # symmetry during bends.
        if index % 2 == 0:
            add_box(
                lines,
                f"vio_ceiling_{index:03d}",
                (
                    center_x - offset_x,
                    center_y - offset_y,
                    PIPE_CENTER_Z + radial_distance,
                    0.0,
                    0.0,
                    yaw,
                ),
                (0.07 + 0.02 * (index % 3), 0.12, 0.018),
                colors[(index * 2 + 1) % len(colors)],
                collision=False,
            )
    return lines


def fiducial_boards(route: Sequence[Point2]) -> List[str]:
    """Place camera-facing inspection tags near, but clear of, the pipe wall."""
    lines: List[str] = []
    backgrounds = [
        (0.98, 0.92, 0.08, 1.0),
        (0.05, 0.85, 0.95, 1.0),
        (0.95, 0.08, 0.30, 1.0),
        (0.15, 0.95, 0.25, 1.0),
    ]
    dark = (0.01, 0.01, 0.01, 1.0)
    bright = (0.98, 0.98, 0.98, 1.0)

    for index, (start, end) in enumerate(zip(route, route[1:])):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        yaw = math.atan2(dy, dx)
        center_x = 0.5 * (start[0] + end[0])
        center_y = 0.5 * (start[1] + end[1])
        # Lateral axis of the route in world coordinates.
        left_x = -math.sin(yaw)
        left_y = math.cos(yaw)
        side = 1.0 if index % 2 == 0 else -1.0
        lateral = 0.39 * side
        z_offset = (-0.16, 0.0, 0.16)[index % 3]
        board_x = center_x + lateral * left_x
        board_y = center_y + lateral * left_y
        board_z = PIPE_CENTER_Z + z_offset

        # Thin local-X dimension means the broad YZ face looks toward vehicles
        # travelling along the route.
        add_emissive_box(
            lines,
            f"vio_tag_bg_{index:03d}",
            (board_x, board_y, board_z, 0.0, 0.0, yaw),
            (0.018, 0.24, 0.22),
            backgrounds[(index * 3) % len(backgrounds)],
        )

        # Two asymmetric dark/bright cells make every board yield strong BRISK
        # corners and avoid a single repeated template.
        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)
        face_offset = -0.013
        cell_shift = 0.055 if index % 4 < 2 else -0.055
        add_emissive_box(
            lines,
            f"vio_tag_dark_{index:03d}",
            (
                board_x + face_offset * forward_x + cell_shift * left_x,
                board_y + face_offset * forward_y + cell_shift * left_y,
                board_z + (0.045 if index % 2 == 0 else -0.045),
                0.0,
                0.0,
                yaw,
            ),
            (0.006, 0.075, 0.075),
            dark,
        )
        add_emissive_box(
            lines,
            f"vio_tag_bright_{index:03d}",
            (
                board_x + face_offset * forward_x - cell_shift * left_x,
                board_y + face_offset * forward_y - cell_shift * left_y,
                board_z - (0.045 if index % 2 == 0 else -0.045),
                0.0,
                0.0,
                yaw,
            ),
            (0.006, 0.055 + 0.01 * (index % 3), 0.055),
            bright,
        )
    return lines


def generate_world() -> str:
    route = build_route()
    lines = [
        '<?xml version="1.0"?>',
        '<sdf version="1.8">',
        '  <world name="pipeline_pool">',
        '    <gravity>0 0 -9.81</gravity>',
        '    <physics name="10ms" type="ode">',
        '      <max_step_size>0.01</max_step_size>',
        '      <real_time_factor>1.0</real_time_factor>',
        '      <real_time_update_rate>100</real_time_update_rate>',
        '    </physics>',
        '    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>',
        '    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>',
        '    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>',
        '    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>',
        '    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">',
        '      <render_engine>ogre2</render_engine>',
        '    </plugin>',
        '    <plugin filename="ignition-gazebo-buoyancy-system" name="ignition::gazebo::systems::Buoyancy">',
        '      <graded_buoyancy>',
        '        <default_density>1000</default_density>',
        '        <density_change><above_depth>0</above_depth><density>1</density></density_change>',
        '      </graded_buoyancy>',
        '    </plugin>',
        '    <scene>',
        '      <ambient>0.28 0.35 0.42 1</ambient>',
        '      <background>0.03 0.09 0.14 1</background>',
        '      <shadows>true</shadows>',
        '      <grid>false</grid>',
        '    </scene>',
        '    <light type="directional" name="surface_light">',
        '      <pose>0 0 10 0 0 0</pose>',
        '      <diffuse>0.8 0.86 0.92 1</diffuse>',
        '      <specular>0.25 0.25 0.25 1</specular>',
        '      <direction>-0.35 0.2 -0.92</direction>',
        '      <cast_shadows>true</cast_shadows>',
        '    </light>',
        '    <model name="pool">',
        '      <static>true</static>',
        '      <link name="walls">',
    ]
    add_box(lines, "pool_bottom", (0.0, 0.0, -8.1, 0.0, 0.0, 0.0), (24.4, 18.4, 0.2), (0.22, 0.32, 0.36, 1.0))
    add_box(lines, "pool_wall_west", (-12.1, 0.0, -4.0, 0.0, 0.0, 0.0), (0.2, 18.4, 8.0), (0.34, 0.48, 0.52, 0.72))
    add_box(lines, "pool_wall_east", (12.1, 0.0, -4.0, 0.0, 0.0, 0.0), (0.2, 18.4, 8.0), (0.34, 0.48, 0.52, 0.72))
    add_box(lines, "pool_wall_south", (0.0, -9.1, -4.0, 0.0, 0.0, 0.0), (24.0, 0.2, 8.0), (0.34, 0.48, 0.52, 0.72))
    add_box(lines, "pool_wall_north", (0.0, 9.1, -4.0, 0.0, 0.0, 0.0), (24.0, 0.2, 8.0), (0.34, 0.48, 0.52, 0.72))
    lines.extend(
        [
            '      </link>',
            '    </model>',
            '    <model name="water_surface">',
            '      <static>true</static>',
            '      <link name="surface">',
            '        <visual name="water_surface_visual">',
            '          <pose>0 0 0.01 0 0 0</pose>',
            '          <geometry><plane><normal>0 0 1</normal><size>24 18</size></plane></geometry>',
            '          <material><ambient>0.02 0.24 0.40 0.28</ambient><diffuse>0.02 0.32 0.52 0.28</diffuse></material>',
            '        </visual>',
            '      </link>',
            '    </model>',
            '    <model name="inspection_pipeline">',
            '      <static>true</static>',
            '      <link name="pipe_shell">',
        ]
    )
    lines.extend(pipe_panels(route))
    lines.extend(route_markers(route))
    lines.extend(visual_feature_markers(route))
    lines.extend(fiducial_boards(route))
    lines.extend(
        [
            '      </link>',
            '    </model>',
            '  </world>',
            '</sdf>',
            '',
        ]
    )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "worlds" / "pipeline_pool.sdf",
    )
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(generate_world(), encoding="utf-8")
    print(f"Generated {args.output}")


if __name__ == "__main__":
    main()
