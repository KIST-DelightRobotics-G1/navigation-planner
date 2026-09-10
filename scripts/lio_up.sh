#!/usr/bin/env bash
# Bring up the LIO engine inside the single all-in-one container (planner + engine).
# Build the image first:  docker build -t kist-nav -f docker/Dockerfile .
#
#   scripts/lio_up.sh            # start container (if needed) + engine
#   docker exec kist tail -f /tmp/lio_engine.log
#   scripts/lio_down.sh          # stop
set -euo pipefail

IMAGE=${KIST_IMAGE:-kist-navigation-planner}
CONTAINER=${KIST_CONTAINER:-kist-navigation-planner}
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAUNCH=/workspace/kist-navigation-planner/lio_engine/lio_bringup.launch.py

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "image '$IMAGE' not found — build it:  ./docker/build.sh" >&2
  exit 1
fi

xhost +local:root >/dev/null 2>&1 || true
if docker ps -aq -f "name=^${CONTAINER}$" | grep -q .; then
  docker start "$CONTAINER" >/dev/null 2>&1 || true       # reuse (incl. one from run.sh)
else
  docker run -d --name "$CONTAINER" --network host \
    -e DISPLAY="${DISPLAY:-}" -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$REPO":/workspace/kist-navigation-planner \
    "$IMAGE" sleep infinity >/dev/null
fi

if ! docker exec "$CONTAINER" bash -c 'ping -c1 -W1 192.168.123.120 >/dev/null 2>&1'; then
  echo "warning: lidar 192.168.123.120 not reachable from the container (host on the robot LAN?)." >&2
fi

# Engine (driver + FAST-LIO) via our bringup, backgrounded; entrypoint sources ROS+overlay.
docker exec -d "$CONTAINER" bash -c "/entrypoint.sh ros2 launch $LAUNCH > /tmp/lio_engine.log 2>&1"

echo "LIO engine up in container '$CONTAINER' (image '$IMAGE')."
echo "  logs:    docker exec $CONTAINER tail -f /tmp/lio_engine.log"
echo "  planner: docker exec -it $CONTAINER /entrypoint.sh ./build/test_lio_odometry_reader config/config.yaml"
