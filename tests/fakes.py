from __future__ import annotations

from ci_testbed.errors import RobotFault
from ci_testbed.geometry import Pose2D, Vec2
from ci_testbed.robot.interface import JointPositions, RobotArm


class FaultyArm:
    """Test double that injects a robot fault on a chosen motion command."""

    def __init__(self, delegate: RobotArm, *, fail_on_move: int = 1) -> None:
        if fail_on_move < 1:
            raise ValueError("fail_on_move must be positive")
        self._delegate = delegate
        self._fail_on_move = fail_on_move
        self.move_attempts = 0
        self.stop_calls = 0

    def _record_move(self) -> None:
        self.move_attempts += 1
        if self.move_attempts == self._fail_on_move:
            raise RobotFault("injected robot motion fault")

    def get_joint_positions(self) -> JointPositions:
        return self._delegate.get_joint_positions()

    def get_tcp_pose(self) -> Pose2D:
        return self._delegate.get_tcp_pose()

    def move_joints(self, target: JointPositions, speed: float | None = None) -> None:
        self._record_move()
        self._delegate.move_joints(target, speed)

    def move_to_pose(self, target: Vec2, speed: float | None = None) -> None:
        self._record_move()
        self._delegate.move_to_pose(target, speed)

    def home(self) -> None:
        self._record_move()
        self._delegate.home()

    def stop(self) -> None:
        self.stop_calls += 1
        self._delegate.stop()

    def set_gripper(self, closed: bool) -> None:
        self._delegate.set_gripper(closed)

    @property
    def is_moving(self) -> bool:
        return self._delegate.is_moving

    @property
    def gripper_closed(self) -> bool:
        return self._delegate.gripper_closed


class StuckArm:
    """Test double that accepts motion commands without moving."""

    def __init__(self, delegate: RobotArm) -> None:
        self._delegate = delegate
        self.stop_calls = 0

    def get_joint_positions(self) -> JointPositions:
        return self._delegate.get_joint_positions()

    def get_tcp_pose(self) -> Pose2D:
        return self._delegate.get_tcp_pose()

    def move_joints(self, target: JointPositions, speed: float | None = None) -> None:
        self._delegate.move_joints(self._delegate.get_joint_positions(), speed)

    def move_to_pose(self, target: Vec2, speed: float | None = None) -> None:
        self._delegate.move_joints(self._delegate.get_joint_positions(), speed)

    def home(self) -> None:
        self._delegate.home()

    def stop(self) -> None:
        self.stop_calls += 1
        self._delegate.stop()

    def set_gripper(self, closed: bool) -> None:
        self._delegate.set_gripper(closed)

    @property
    def is_moving(self) -> bool:
        return self._delegate.is_moving

    @property
    def gripper_closed(self) -> bool:
        return self._delegate.gripper_closed
