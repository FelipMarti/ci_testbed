FROM python:3.12-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1

WORKDIR /app

COPY pyproject.toml README.md ./
COPY src ./src
COPY docker/requirements.txt ./docker/requirements.txt
COPY docker/mock_service.py ./docker/mock_service.py

RUN pip install --no-cache-dir -r docker/requirements.txt \
    && pip install --no-cache-dir .

RUN groupadd --gid 10001 appuser \
    && useradd --uid 10001 --gid appuser --create-home appuser

USER appuser

EXPOSE 8001 8002 8003

ENTRYPOINT ["python", "/app/docker/mock_service.py"]
