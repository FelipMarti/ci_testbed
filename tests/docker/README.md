# Docker-based tests (future work)

Tests in this directory will be marked `docker` and will only run in the nightly
or dedicated Docker workflow.

Planned fixtures:

- a session-scoped compose fixture that starts the mock services
- healthcheck polling with retries and a strict timeout
- HTTP adapter fixtures injected into the existing controller
- contract parametrisation so the same assertions run against simulators and
  containers

Example:

```python
@pytest.fixture(scope="session")
def compose_services():
    subprocess.run(["docker", "compose", "up", "-d", "--wait"], check=True)
    yield
    subprocess.run(["docker", "compose", "down", "-v"], check=True)
```

The exact fixtures will be added together with `docker/docker-compose.yml`.
