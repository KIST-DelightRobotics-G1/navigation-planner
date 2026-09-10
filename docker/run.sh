#!/bin/bash
# Launch (or re-attach to) a persistent named container. Reuse across sessions
# so builds/caches survive until you `docker rm kist-navigation-planner`.
#
# Slimmed with the build: this image is the ROS-free nav foundation (no GPU/CV),
# so it only needs the robot's DDS streams:
#   --network host    DDS discovery/subscribe (LiDAR, IMU, odom, lowstate, UWB)
#
# (Dropped with the parked CV/camera stacks: --gpus all, X11/DISPLAY for OpenCV
#  viewers, and the ext-sensor-io sibling mount.)
#
# The whole repo dir is bind-mounted, so the container and the host repo share
# one directory — source, build/, and any built artifacts are live on both sides.
# Edit on the host, build/run in the container.
#
# NOTE: mounts are fixed at create time — if a container already exists without
# this mount, recreate it:  docker rm -f kist-navigation-planner

set -e

CONTAINER=kist-navigation-planner
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# The image is self-contained (deps + engine + source baked and built); NO source
# mount — run.sh drops you into a container with ready binaries under build/, so you
# never build inside. env.sh is auto-sourced (ROS + lio_up/lio_down ready).
#
# Iterative dev: add  -v "${REPO_DIR}":/workspace/kist-navigation-planner  below to
# shadow the baked source with your working copy — then rebuild inside (the bind mount
# hides the baked build/).
if [ "$(docker ps -q -f name=^${CONTAINER}$)" ]; then
    docker exec -it "${CONTAINER}" bash   # .bashrc sources env.sh (ROS + overlay + helpers)
elif [ "$(docker ps -aq -f name=^${CONTAINER}$)" ]; then
    docker start -ai "${CONTAINER}"
else
    # X11 for rviz2 (LIO viz); --network host for the robot/engine DDS streams.
    docker run -it \
        --name "${CONTAINER}" \
        --network host \
        -e DISPLAY="${DISPLAY:-}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -w /workspace/kist-navigation-planner \
        kist-navigation-planner
fi
