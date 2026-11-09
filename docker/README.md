# Docker mock services

The Docker suite runs one shared image with three FastAPI roles. The Python
package stays `numpy`-only; FastAPI and uvicorn are installed only in the image
from `docker/requirements.txt`.

## Layout

```text
docker/
├── Dockerfile
├── docker-compose.yml
├── mock_service.py
├── requirements.txt
└── README.md
```

## Services

| Service | Port | Endpoints |
| --- | --- | --- |
| `camera-mock` | 8001 | `GET /frame`, `POST /target`, `GET /config`, `POST /faults` |
| `robot-mock` | 8002 | `GET /state`, `POST /move_joints`, `POST /move_to_pose`, `POST /home`, `POST /stop`, `POST /gripper`, `POST /faults` |
| `lidar-mock` | 8003 | `GET /distance`, `POST /distance`, `POST /faults` |

Every service also exposes `GET /health` and `POST /reset`.

`POST /faults` accepts `{"count": n}`. The next `n` camera captures, lidar reads,
or robot motion commands fail with the matching domain error.

## Build and run

```bash
docker build -f docker/Dockerfile -t ci-testbed-mocks:local .
docker compose -f docker/docker-compose.yml up -d --wait
pytest -m docker
docker compose -f docker/docker-compose.yml down -v
```

`CI_TESTBED_MOCKS_IMAGE` overrides the image used by compose. CI builds and loads
the image once, then sets `CI_TESTBED_SKIP_BUILD=1` so the pytest fixture does
not rebuild it.

## HTTP adapters

The adapters in `src/ci_testbed/adapters/` implement the existing `Camera`,
`RangeSensor`, and `RobotArm` protocols:

```python
from ci_testbed.adapters import HttpCamera, HttpRangeSensor, HttpRobotArm

camera = HttpCamera("http://127.0.0.1:8001")
lidar = HttpRangeSensor("http://127.0.0.1:8003")
arm = HttpRobotArm("http://127.0.0.1:8002")
```

Service errors map to `CameraFault`, `SensorDropout`, `JointLimitError`,
`UnreachableTargetError`, and `RobotFault`. Transport failures map to the
matching sensor or robot fault so the controller can keep using the same
failure-recovery paths.

## CI integration

- Branch pushes validate the compose file with `docker compose config --quiet`.
- PRs build/load the image and run the full `pytest -m docker` suite.
- `main` publishes `latest` and `sha-<full>` to GHCR, pulls the published image,
  and runs `pytest -m "docker and smoke"`.
- Nightly runs the full Docker suite with retries.
- Releases retag the tested `sha-<commit>` image as `v*`.

Container-starting jobs are gated by the repository variable
`ENABLE_DOCKER_JOBS`. The branch compose check is always enabled.
