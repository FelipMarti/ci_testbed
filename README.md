# ci-testbed

CI testbed with pytest and GitHub Actions of a small sim robotics cell with `pytest` and GitHub
Actions setup. 

The simulation models the parts of a robotics stack that are usually hardest to
test:

- a top-down **camera** with deterministic noise and injected failures
- a **range sensor** used for the safety check
- a **two-link planar arm** with forward/inverse kinematics and joint limits
- a **perception** blob detector and pixel-to-world calibration
- a **pick-and-place controller** state machine with retries and safe aborts

The domain layer has no ROS2 imports. Future ROS2, HTTP, and Docker adapters can
implement the same protocols and reuse the same contract tests.

## Quick start

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"

pytest                         # everything except docker/hardware
pytest -m unit                 # fastest feedback
pytest -m "integration or system"
pytest --cov=ci_testbed --cov-report=html

ruff check .
ruff format --check .
mypy src

python -m ci_testbed.cli --target-x 0.34 --target-y 0.28
```

Example result:

```json
{
  "success": true,
  "state": "done",
  "target_world": {"x": 0.34, "y": 0.28},
  "final_pose": {"x": 0.4975, "y": 0.2475, "theta": 0.0},
  "detection_attempts": 1,
  "abort_reason": null,
  "duration": 1.47
}
```

The CLI uses a deterministic fake clock by default so it returns immediately.
Pass `--real-time` to exercise wall-clock sleeps.

## Architecture

```text
src/ci_testbed/
├── clock.py              Clock protocol, FakeClock, SystemClock
├── config.py             validated dataclass configuration
├── geometry.py           Vec2, poses, pixel/world calibration
├── robot/
│   ├── interface.py      RobotArm Protocol
│   ├── sim_arm.py        two-link FK/IK, joint limits, deterministic motion
│   └── trajectory.py     interpolation and duration helpers
├── sensors/
│   ├── interface.py      Camera and RangeSensor protocols, Frame
│   ├── synthetic_camera.py
│   └── synthetic_range.py
├── perception/
│   └── blob_detector.py  threshold + intensity-weighted centroid
├── control/
│   └── pick_place.py     state machine, retries, safety abort
└── cli.py                runnable end-to-end simulation
```

The controller depends only on protocols and immutable configuration, which is
what makes it straightforward to test with simulators, etc.

## Test strategy

The suite follows a small test pyramid:

| Marker | Purpose | Examples |
| --- | --- | --- |
| `unit` | no I/O, fast | geometry, config validation, FK/IK, detector, noise |
| `integration` | real components in-process | camera → detector → world coordinate |
| `system` | full cell | detect → move → grasp → retreat, CLI artifact |
| `contract` | behaviour every adapter must provide | camera frames, arm motion |
| `slow` | nightly-only scenarios | long stress runs |
| `docker` | future mock containers | camera/robot/lidar services |
| `hardware` | future self-hosted ROS2 runner | hardware-in-the-loop |

Notable pytest practices used here:

- fixtures for the fake clock, camera, arm, sensor, detector, and controller
- `parametrize` for image targets, joint-limit validation, noise, and errors
- fault injection instead of mocking every collaborator
- deterministic seeds for reproducible "flaky-looking" scenarios
- JUnit XML and coverage output for CI
- `pytest-xdist` for parallel execution
- `pytest-rerunfailures` only for integration/system jobs, where transient
  infrastructure failures are plausible

Failure scenarios covered:

- transient camera failure → retried and then succeeds
- persistent camera failure → reaches retry limit, stops safely
- obstacle inside the safety clearance → abort before any motion
- safety sensor dropout → abort before any motion
- robot motion fault → stop, report failure, leave the arm consistent
- unreachable Cartesian target → reject before motion

## GitHub Actions

CI is split by event: feature-branch pushes stay fast, and `main` only receives
pull requests that passed the full suite.

| Trigger | Workflow | Jobs |
| --- | --- | --- |
| Push to feature branch | `branch-ci.yml` | lint, typecheck, unit matrix, branch summary |
| PR to `main` | `pr-ci.yml` | lint, typecheck, unit matrix, integration + contract, system, coverage, future Docker E2E, PR summary |
| Merge to `main` | `main.yml` | post-merge full suite, package build, future container image publish/smoke |
| Nightly / manual | `nightly.yml` | full suite with retries, future Docker suite |
| Tag `v*` | `release.yml` | wheel/sdist and GitHub release |

While a PR is open, a branch push triggers both `Branch CI` for fast feedback and
`PR CI` for the full gate. The `PR CI summary` job is the required check.

### Pipeline shape

```text
feature branch push
lint ──────────────┐
typecheck ─────────┼─> unit matrix ─> branch summary

pull request to main
lint ──────────────┐
typecheck ─────────┼─> integration + contract ─┐
                   ├─> system ─────────────────┤
                   └─> docker-e2e (future) ────┤
unit matrix ────────────────────────────────────┼─> coverage ─> PR summary

merge to main
post-merge tests ─┐
build package ────┼─> container images/smoke (future) ─> main summary
```


### What the pipeline covers

- **Runners:** GitHub-hosted `ubuntu-latest` for branch, PR, and post-merge
  pipelines. A self-hosted runner with labels such as
  `[self-hosted, linux, ros2]` is the intended target for future ROS2 and
  hardware-in-the-loop jobs.
- **Jobs and DAG:** lint, typecheck, unit matrix, integration/contract, system,
  coverage gate, and an aggregate summary that acts as the required check.
- **Parallel execution:** Python-version matrix plus `pytest-xdist`; integration,
  system, and future Docker jobs start after lint/typecheck instead of waiting
  for the unit matrix. Matrix jobs use `fail-fast: false`.
- **Caching:** pip via `actions/setup-python`, plus `.ruff_cache` and
  `.mypy_cache` with `restore-keys`. Future Docker builds use BuildKit GHA cache.
- **Artifacts:** per-job JUnit XML, pytest logs, coverage XML/HTML, and a
  generated system-result JSON. Main uploads `main-dist`, release uploads
  `release-dist`. Uploads use `if: always()` so failures still leave diagnostics.
- **Failure recovery:** per-job timeouts, pytest timeouts, pip install retries,
  reruns only for integration/system tests, stale-run cancellation via
  `concurrency`, and summary jobs that fail if any required upstream job failed
  or was cancelled.
- **Coverage gate:** `fail_under = 85` with branch coverage configured in
  `pyproject.toml`.
- **Post-merge delivery:** `main.yml` re-runs the non-container suite, builds
  wheel/sdist artifacts, and reserves a gated job for publishing container
  images to GHCR with a compose smoke test.
- **Nightly:** `nightly.yml` runs the full non-hardware suite with retries and
  longer artifact retention. A gated `docker_mocks` job is reserved for the
  future container phase.
- **Release:** `release.yml` builds wheel/sdist artifacts and publishes a GitHub
  release for `v*` tags.


## Future work: Docker mock services

The next phase will add containerised mocks without changing the domain logic:

- `camera-mock`: HTTP or MJPEG endpoint returning synthetic frames
- `robot-mock`: REST endpoint for joint state and move commands
- `lidar-mock`: distance endpoint with injectable faults

Planned design:

- each mock implements the same `Camera`/`RangeSensor`/`RobotArm` protocols
- the `contract` suite runs unchanged against simulators and HTTP adapters
- `docker compose` provides healthchecks and deterministic seeds
- `tests/docker/` uses `testcontainers-python` or a compose fixture
- later, real ROS2 adapters (`rclpy`) reuse the same contract tests on a
  self-hosted runner

Where Docker tests run once the mocks exist:

| Stage | Docker tests | Images |
| --- | --- | --- |
| PR to `main` | mock E2E plus container logs | build from PR, do not push |
| Merge to `main` | smoke test published images | build and push to GHCR |
| Nightly | full Docker suite | use `latest` |
| Release | smoke test released images | tag images |
| ROS2 / hardware | not used | trusted self-hosted runners only |

The `docker_e2e` job in `pr-ci.yml`, `container_images` in `main.yml`, and
`docker_mocks` in `nightly.yml` are gated by the repository variable
`ENABLE_DOCKER_JOBS`; leave it unset until these files exist. See
[`docker/README.md`](docker/README.md) for the proposed layout.

