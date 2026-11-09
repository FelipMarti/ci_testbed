from __future__ import annotations

import pytest


@pytest.fixture(autouse=True)
def reset_docker_mocks(reset_mocks: None) -> None:
    return None
