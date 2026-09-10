#!/bin/bash
# Build the all-in-one image (ros:humble base): the ROS-free planner PLUS the LIO
# engine (deepglint FAST-LIO + Livox driver, ROS2) it consumes over DDS. Self-
# contained: clones unitree_sdk2 + Livox-SDK2 + deepglint (pinned) and builds them
# with the repo baked in — see docs/LIO_ENGINE.md. Heavy (~10 min, ~640 MB clone).

set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker build \
    -t kist-navigation-planner \
    -f "${REPO_DIR}/docker/Dockerfile" \
    "${REPO_DIR}"
