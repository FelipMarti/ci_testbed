"""Run the simulated pick-and-place cell from the command line."""

from __future__ import annotations

import argparse
import json
from collections.abc import Sequence
from dataclasses import replace
from pathlib import Path

from ci_testbed.clock import Clock, FakeClock, SystemClock
from ci_testbed.config import CellConfig
from ci_testbed.control.pick_place import PickPlaceController
from ci_testbed.geometry import Vec2
from ci_testbed.perception.blob_detector import BlobDetector
from ci_testbed.robot.sim_arm import SimulatedArm
from ci_testbed.sensors.synthetic_camera import SyntheticCamera
from ci_testbed.sensors.synthetic_range import SyntheticRangeSensor


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-x", type=float, default=0.35)
    parser.add_argument("--target-y", type=float, default=0.30)
    parser.add_argument("--noise-std", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--camera-faults", type=int, default=0)
    parser.add_argument("--range-dropouts", type=int, default=0)
    parser.add_argument("--obstacle-distance", type=float, default=None)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument(
        "--real-time",
        action="store_true",
        help="sleep using the wall clock instead of the deterministic fake clock",
    )
    return parser


def run_simulation(args: argparse.Namespace) -> dict[str, object]:
    base = CellConfig()
    config = CellConfig(
        camera=replace(base.camera, noise_std=args.noise_std, seed=args.seed),
        range_sensor=replace(base.range_sensor, seed=args.seed + 1),
        target_world=Vec2(args.target_x, args.target_y),
    )
    clock: Clock = SystemClock() if args.real_time else FakeClock()

    camera = SyntheticCamera(config.camera, clock, target_world=config.target_world)
    range_sensor = SyntheticRangeSensor(config.range_sensor, clock)
    arm = SimulatedArm(config.arm, clock)
    controller = PickPlaceController(
        camera=camera,
        detector=BlobDetector(),
        calibration=camera.calibration,
        range_sensor=range_sensor,
        arm=arm,
        config=config.controller,
        clock=clock,
    )

    if args.camera_faults:
        camera.fail_next(args.camera_faults)
    if args.range_dropouts:
        range_sensor.drop_next(args.range_dropouts)
    if args.obstacle_distance is not None:
        range_sensor.set_distance(args.obstacle_distance)

    return controller.run().as_dict()


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    result = run_simulation(args)
    rendered = json.dumps(result, indent=2)
    print(rendered)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
