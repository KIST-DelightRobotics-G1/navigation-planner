#!/bin/bash
# Build the container image (plain Ubuntu base; builds the ROS-free nav
# foundation — no GPU/CV). Self-contained: clones unitree_sdk2 (pinned), builds
# CycloneDDS idlc, and bakes the repo in — no host vendoring or bind mount needed.

set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker build \
    -t kist-navigation-planner \
    -f "${REPO_DIR}/docker/Dockerfile" \
    "${REPO_DIR}"
