from __future__ import annotations

import math

import pytest

from ci_testbed.clock import FakeClock
from ci_testbed.config import ArmConfig
from ci_testbed.errors import JointLimitError, UnreachableTargetError
from ci_testbed.geometry import Vec2
from ci_testbed.robot.sim_arm import SimulatedArm

pytestmark = pytest.mark.unit


def test_forward_kinematics_at_zero() -> None:
    arm = SimulatedArm(ArmConfig(link1=0.3, link2=0.2), FakeClock())

    pose = arm.forward_kinematics((0.0, 0.0))

    assert pose.position.x == pytest.approx(0.5)
    assert pose.position.y == pytest.approx(0.0)
    assert pose.theta == pytest.approx(0.0)


def test_forward_kinematics_at_ninety_degrees() -> None:
    arm = SimulatedArm(ArmConfig(link1=0.3, link2=0.2), FakeClock())

    pose = arm.forward_kinematics((math.pi / 2.0, 0.0))

    assert pose.position.x == pytest.approx(0.0, abs=1e-12)
    assert pose.position.y == pytest.approx(0.5)
    assert pose.theta == pytest.approx(math.pi / 2.0)


@pytest.mark.parametrize(
    "target",
    [Vec2(0.4, 0.0), Vec2(0.25, 0.2), Vec2(0.1, -0.2), Vec2(0.0, 0.4), Vec2(0.3, 0.3)],
)
def test_inverse_kinematics_round_trip(target: Vec2) -> None:
    arm = SimulatedArm(ArmConfig(), FakeClock())

    joints = arm.inverse_kinematics(target)
    pose = arm.forward_kinematics(joints)

    assert pose.position.distance_to(target) == pytest.approx(0.0, abs=1e-9)


def test_inverse_kinematics_elbow_up() -> None:
    arm = SimulatedArm(ArmConfig(), FakeClock())

    joints = arm.inverse_kinematics(Vec2(0.3, 0.2), elbow_up=True)

    assert joints[1] >= 0.0


def test_unreachable_target_is_rejected() -> None:
    arm = SimulatedArm(ArmConfig(), FakeClock())

    with pytest.raises(UnreachableTargetError):
        arm.inverse_kinematics(Vec2(1.0, 0.0))


@pytest.mark.parametrize("target", [Vec2(0.1, 0.0), Vec2(0.0, 0.0)])
def test_target_inside_inner_workspace_is_rejected(target: Vec2) -> None:
    arm = SimulatedArm(ArmConfig(link1=0.4, link2=0.2), FakeClock())

    with pytest.raises(UnreachableTargetError):
        arm.inverse_kinematics(target)


def test_move_joints_updates_state_and_clock() -> None:
    clock = FakeClock()
    arm = SimulatedArm(ArmConfig(), clock)

    arm.move_joints((0.0, 0.0))

    assert arm.get_joint_positions() == pytest.approx((0.0, 0.0))
    assert arm.move_count == 1
    assert clock.now() > 0.0
    assert not arm.is_moving


def test_move_joints_rejects_limit_violation() -> None:
    clock = FakeClock()
    arm = SimulatedArm(ArmConfig(), clock)

    with pytest.raises(JointLimitError):
        arm.move_joints((10.0, 0.0))

    assert arm.get_joint_positions() == pytest.approx(arm.config.home_joints)
    assert arm.move_count == 0
    assert clock.now() == 0.0


@pytest.mark.parametrize("speed", [0.0, -1.0])
def test_move_joints_rejects_invalid_speed(speed: float) -> None:
    arm = SimulatedArm(ArmConfig(), FakeClock())

    with pytest.raises(ValueError):
        arm.move_joints((0.0, 0.0), speed)


def test_move_to_pose_reaches_target() -> None:
    arm = SimulatedArm(ArmConfig(), FakeClock())
    target = Vec2(0.3, 0.2)

    arm.move_to_pose(target)

    assert arm.get_tcp_pose().position.distance_to(target) == pytest.approx(0.0, abs=1e-9)


def test_home_and_gripper_and_stop() -> None:
    arm = SimulatedArm(ArmConfig(), FakeClock())

    arm.move_joints((0.0, 0.0))
    arm.home()
    arm.set_gripper(True)
    arm.stop()

    assert arm.get_joint_positions() == pytest.approx(arm.config.home_joints)
    assert arm.gripper_closed
    assert arm.stop_count == 1
    assert arm.move_count == 2
