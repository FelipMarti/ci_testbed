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

The domain layer has no ROS2 imports. HTTP adapters and containerised mock
services implement the same protocols and reuse the same contract tests.

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

Run the containerised mocks locally:

```bash
docker compose -f docker/docker-compose.yml up -d --wait
pytest -m docker
docker compose -f docker/docker-compose.yml down -v
```

The compose fixture builds the image by default. Set
`CI_TESTBED_MOCKS_IMAGE=ci-testbed-mocks:local CI_TESTBED_SKIP_BUILD=1` to reuse
an image that was already built or pulled.

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
| `docker` | containerised mock services | camera/robot/lidar HTTP adapters, compose E2E |
| `smoke` | fast published-image check | main-branch pick-and-place E2E |
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
| Push to feature branch | `branch-ci.yml` | lint, typecheck, unit matrix, compose config, branch summary |
| PR to `main` | `pr-ci.yml` | lint, typecheck, unit matrix, integration + contract, system, coverage, full Docker suite, PR summary |
| Merge to `main` | `main.yml` | post-merge full suite, package build, GHCR image publish, published-image smoke |
| Nightly / manual | `nightly.yml` | full suite with retries, full Docker suite |
| Tag `v*` | `release.yml` | wheel/sdist, GitHub release, tested image retag |

While a PR is open, a branch push triggers both `Branch CI` for fast feedback and
`PR CI` for the full gate. The `PR CI summary` job is the required check.

### Pipeline shape

```text
feature branch push
lint ──────────────┐
typecheck ─────────┼─> unit matrix ─┐
docker config ─────┘                └─> branch summary

pull request to main
lint ──────────────┐
typecheck ─────────┼─> integration + contract ─┐
                   ├─> system ─────────────────┤
                   └─> docker mock suite ──────┤
unit matrix ────────────────────────────────────┼─> coverage ─> PR summary

merge to main
post-merge tests ─┐
build package ────┼─> publish image ─> smoke published image ─> main summary
```


### What the pipeline covers

- **Runners:** GitHub-hosted `ubuntu-latest` for branch, PR, and post-merge
  pipelines. A self-hosted runner with labels such as
  `[self-hosted, linux, ros2]` is the intended target for future ROS2 and
  hardware-in-the-loop jobs.
- **Jobs and DAG:** lint, typecheck, unit matrix, integration/contract, system,
  coverage gate, and an aggregate summary that acts as the required check.
- **Parallel execution:** Python-version matrix plus `pytest-xdist`; integration,
  system, and Docker jobs start after lint/typecheck instead of waiting for the
  unit matrix. Matrix jobs use `fail-fast: false`.
- **Caching:** pip via `actions/setup-python`, plus `.ruff_cache` and
  `.mypy_cache` with `restore-keys`. Docker builds use BuildKit GHA cache.
- **Artifacts:** per-job JUnit XML, pytest logs, coverage XML/HTML, compose
  logs, and a generated system-result JSON. Main uploads `main-dist`, release
  uploads `release-dist`. Uploads use `if: always()` so failures still leave
  diagnostics.
- **Failure recovery:** per-job timeouts, pytest timeouts, pip install retries,
  reruns only for integration/system/nightly Docker tests, stale-run
  cancellation via `concurrency`, and summary jobs that fail if any required
  upstream job failed or was cancelled.
- **Coverage gate:** `fail_under = 85` with branch coverage configured in
  `pyproject.toml`.
- **Post-merge delivery:** `main.yml` re-runs the non-container suite, builds
  wheel/sdist artifacts, publishes `latest` and `sha-<full>` images to GHCR, then
  runs the `smoke` tests against the published image.
- **Nightly:** `nightly.yml` runs the full non-hardware suite and the full Docker
  suite with retries and longer artifact retention.
- **Release:** `release.yml` builds wheel/sdist artifacts, publishes a GitHub
  release for `v*` tags, and retags the tested `sha-<commit>` image as the
  release tag.

## Docker mock services

One shared image (`ci-testbed-mocks`) runs three FastAPI services. The package
runtime stays `numpy`-only; FastAPI and uvicorn live in
`docker/requirements.txt`.

| Service | Port | Endpoints |
| --- | --- | --- |
| `camera-mock` | 8001 | `GET /frame`, `POST /target`, `GET /config`, `POST /faults` |
| `robot-mock` | 8002 | `GET /state`, `POST /move_joints`, `POST /move_to_pose`, `POST /home`, `POST /stop`, `POST /gripper`, `POST /faults` |
| `lidar-mock` | 8003 | `GET /distance`, `POST /distance`, `POST /faults` |

All services expose `GET /health` and `POST /reset`. Fault endpoints accept
`{"count": n}`; camera faults, lidar dropouts, and robot motion faults map back
to the existing domain exceptions in the HTTP adapters.

The HTTP adapters live in `src/ci_testbed/adapters/` and implement the same
protocols as the simulators. The `contract` suite runs against both
implementations, and `tests/docker/` contains compose-backed service and E2E
tests. Docker fixtures are session-scoped and write `reports/docker-compose.log`
on teardown.

| Stage | Docker tests | Images |
| --- | --- | --- |
| Branch push | `docker compose config --quiet` | none |
| PR to `main` | full `pytest -m docker` | build and load locally, do not push |
| Merge to `main` | `pytest -m "docker and smoke"` | publish `latest` and `sha-<full>` to GHCR |
| Nightly | full Docker suite with retries | build locally |
| Release | none | pull `sha-<commit>` and push `v*` tag |
| ROS2 / hardware | not used | trusted self-hosted runners only |

The `docker_e2e` job in `pr-ci.yml`, `container_images` in `main.yml`,
`docker_mocks` in `nightly.yml`, and `docker_image` in `release.yml` are gated by
the repository variable `ENABLE_DOCKER_JOBS`; the branch compose check is not
gated. See [`docker/README.md`](docker/README.md) for build and endpoint details.

