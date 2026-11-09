"""Robot abstraction implemented by the simulator and future ROS2 adapters."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from ci_testbed.geometry import Pose2D, Vec2

JointPositions = tuple[float, float]


@runtime_checkable
class RobotArm(Protocol):
    """Small subset of a robot arm API used by the pick-and-place controller."""

    def get_joint_positions(self) -> JointPositions:
        """Return the current shoulder and elbow angles in radians."""

    def get_tcp_pose(self) -> Pose2D:
        """Return the current tool centre point pose."""

    def move_joints(self, target: JointPositions, speed: float | None = None) -> None:
        """Move to joint angles, raising ``JointLimitError`` when invalid."""

    def move_to_pose(self, target: Vec2, speed: float | None = None) -> None:
        """Move the tool centre point to a world position using inverse kinematics."""

    def home(self) -> None:
        """Move to the configured home position."""

    def stop(self) -> None:
        """Stop motion immediately."""

    def set_gripper(self, closed: bool) -> None:
        """Open or close the gripper."""

    @property
    def is_moving(self) -> bool:
        """Whether the arm is currently executing a motion."""

    @property
    def gripper_closed(self) -> bool:
        """Whether the gripper is closed."""
