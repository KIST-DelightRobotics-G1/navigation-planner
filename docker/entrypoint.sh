#!/usr/bin/env bash
# Source ROS2 + the LIO engine overlay so both the engine (ros2 launch) and the
# ROS-free planner binaries work in the same container, then run the command.
set -e
source /opt/ros/humble/setup.bash
[ -f /opt/lio_ws/install/setup.bash ] && source /opt/lio_ws/install/setup.bash
exec "$@"
