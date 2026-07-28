#!/bin/bash
# Build the container image (TensorRT 10.7 base; builds the GPU inference path).
# Self-contained: clones unitree_sdk2 (pinned) + bakes the repo in — no host
# vendoring or bind mount needed.

set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker build \
    -t kist-navigation-planner \
    -f "${REPO_DIR}/docker/Dockerfile" \
    "${REPO_DIR}"
