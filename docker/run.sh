#!/bin/bash
# Launch (or re-attach to) a persistent named container. Reuse across sessions
# so builds/caches survive until you `docker rm kist-navigation-planner`.
#
# Wired for the workstation consumer role (this repo captures no devices — it
# subscribes to the robot's DDS streams and runs GPU inference):
#   --gpus all                 TensorRT inference on the RTX 4090
#   --network host             DDS discovery/subscribe (LiDAR, odom, UWB, camera)
#   DISPLAY + /tmp/.X11-unix   OpenCV viewers (pose / segmentation)
#
# Assumes the 4090 workstation (--gpus all). On a Jetson (L4T) swap in
# --runtime nvidia. Needs a working NVIDIA driver — if the host driver is in
# version mismatch (unattended-upgrade without reboot), reboot first or --gpus
# all fails at container init.
#
# The whole repo dir is bind-mounted, so the container and the host repo share
# one directory — models/, source, build/, and any built artifacts (e.g. the
# .trt engine) are all live on both sides. Convenient for experimenting: edit
# on the host, build/run in the container.
#
# NOTE: mounts are fixed at create time — if a container already exists without
# this mount, recreate it:  docker rm -f kist-navigation-planner
# A host build/ configured against a different TRT will clash — clear it once
# in the container before building:  rm -rf build && cmake -B build ...

set -e

CONTAINER=kist-navigation-planner
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# The live-camera seg stage embeds ext-sensor-io's RealsenseReceiver. When the
# sibling repo is checked out next to this one, mount it as the container-side
# sibling so the CMake ext block finds it at ../kist-ext-sensor-io.
EXT_DIR="$(cd "${REPO_DIR}/.." && pwd)/kist-ext-sensor-io"
EXT_MOUNT=()
if [ -d "${EXT_DIR}" ]; then
    EXT_MOUNT=(-v "${EXT_DIR}:/workspace/kist-ext-sensor-io")
fi

if [ "$(docker ps -q -f name=^${CONTAINER}$)" ]; then
    docker exec -it "${CONTAINER}" /bin/bash
elif [ "$(docker ps -aq -f name=^${CONTAINER}$)" ]; then
    docker start -ai "${CONTAINER}"
else
    xhost +local:root >/dev/null 2>&1 || true
    docker run -it \
        --name "${CONTAINER}" \
        --gpus all \
        --network host \
        -e DISPLAY="${DISPLAY}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "${REPO_DIR}":/workspace/kist-navigation-planner \
        "${EXT_MOUNT[@]}" \
        -w /workspace/kist-navigation-planner \
        kist-navigation-planner
fi
