"""Small trajectory helpers shared by the arm and its tests."""

from __future__ import annotations

from ci_testbed.geometry import Vec2
from ci_testbed.robot.interface import JointPositions


def interpolate_joints(
    start: JointPositions, goal: JointPositions, steps: int
) -> tuple[JointPositions, ...]:
    """Linearly interpolate joint waypoints, including both endpoints."""

    if steps < 2:
        raise ValueError("steps must be at least 2")
    return tuple(
        (
            start[0] + (goal[0] - start[0]) * index / (steps - 1),
            start[1] + (goal[1] - start[1]) * index / (steps - 1),
        )
        for index in range(steps)
    )


def interpolate_cartesian(start: Vec2, goal: Vec2, steps: int) -> tuple[Vec2, ...]:
    """Linearly interpolate Cartesian waypoints, including both endpoints."""

    if steps < 2:
        raise ValueError("steps must be at least 2")
    return tuple(
        Vec2(
            start.x + (goal.x - start.x) * index / (steps - 1),
            start.y + (goal.y - start.y) * index / (steps - 1),
        )
        for index in range(steps)
    )


def joint_duration(start: JointPositions, goal: JointPositions, max_joint_speed: float) -> float:
    """Return the time needed for the slowest joint to complete the move."""

    if max_joint_speed <= 0:
        raise ValueError("max_joint_speed must be positive")
    largest_delta = max(abs(goal[0] - start[0]), abs(goal[1] - start[1]))
    return largest_delta / max_joint_speed
