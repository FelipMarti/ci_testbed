# Docker-based tests

Tests in this directory are marked `docker` and start the compose stack through
the session-scoped `docker_services` fixture in `tests/conftest.py`. The default
pytest run excludes them; use `pytest -m docker`.

The fixture:

- builds the shared image unless `CI_TESTBED_SKIP_BUILD=1`
- runs `docker compose up -d --wait`
- exposes camera, robot, and lidar base URLs
- resets all services before and after each Docker test
- writes `reports/docker-compose.log`
- tears down with `docker compose down -v`

The suite covers health/reset, camera frames and faults, lidar distance and
dropouts, robot motion and faults, and a full pick-and-place E2E marked `smoke`.
The same `tests/contract` assertions also run against the HTTP adapters when the
`docker` marker is selected.
