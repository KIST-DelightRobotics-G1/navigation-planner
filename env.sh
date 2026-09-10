# Environment + helpers for working inside the kist-navigation-planner container.
# Auto-sourced from ~/.bashrc (see docker/Dockerfile); or `source env.sh` by hand.
# Same shape as kist-gearsonic-inference's env.sh (source once, call the functions).

# ROS2 + the LIO engine overlay, so `ros2 launch` and the engine nodes are on PATH.
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f /opt/lio_ws/install/setup.bash ] && source /opt/lio_ws/install/setup.bash

# Start the LIO engine (Livox driver + FAST-LIO) detached via our bringup launch.
# The planner then consumes /Odometry_loc + /cloud_registered_1 over DDS.
#   lio_up            # start
#   tail -f /tmp/lio_engine.log
#   lio_down          # stop
lio_up() {
    if pgrep -f lio_bringup.launch.py >/dev/null 2>&1; then
        echo "LIO engine already running (tail -f /tmp/lio_engine.log)."
        return 0
    fi
    if command -v ping >/dev/null 2>&1 && ! ping -c1 -W1 192.168.123.120 >/dev/null 2>&1; then
        echo "warning: lidar 192.168.123.120 not reachable (host on the robot LAN?)." >&2
    fi
    setsid ros2 launch /workspace/kist-navigation-planner/lio_engine/lio_bringup.launch.py \
        >> /tmp/lio_engine.log 2>&1 < /dev/null &
    echo "LIO engine started (log: /tmp/lio_engine.log)."
}

# Stop the LIO engine (kills the driver + FAST-LIO started by lio_up).
lio_down() {
    local stopped=0
    for pat in lio_bringup.launch.py fastlio_mapping livox_ros_driver2_node; do
        pkill -f "$pat" 2>/dev/null && stopped=1
    done
    [ "$stopped" = 1 ] && echo "LIO engine stopped." || echo "LIO engine not running."
}
