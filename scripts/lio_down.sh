#!/usr/bin/env bash
# Stop the all-in-one container (kills the engine + frees the robot LAN ports).
set -euo pipefail
CONTAINER=${KIST_CONTAINER:-kist-navigation-planner}
if docker rm -f "$CONTAINER" >/dev/null 2>&1; then
  echo "stopped ('$CONTAINER' removed)."
else
  echo "not running."
fi
