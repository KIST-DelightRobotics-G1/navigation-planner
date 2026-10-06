#pragma once

// LioProcessManager — starts/stops the external LIO engine (Livox driver + FAST-LIO) as a child
// process, so navigation can run it episodically: LIO is UP only while driving to a goal and DOWN
// while idle (when VLA manipulation happens). A manipulation that moves large geometry through the
// LiDAR FOV corrupts FAST-LIO's scan matching and makes its odom diverge; keeping LIO off during
// that window avoids the failure at the system level, and each goal gets a fresh estimator.
//
// This is the C++ equivalent of env.sh's run_lio_daemon / stop_lio_daemon: it launches the same
// `ros2 launch lio_engine/lio_bringup.launch.py` in a NEW session (setsid) so the whole group
// (driver + FAST-LIO) can be killed together, with the ROS2 overlay sourced first (nav itself is
// ROS-free). The engine's output still goes to the same log file.
//
// Disabled by default (manage=false) -> nav assumes LIO is run externally (current workflow).

#include <atomic>
#include <string>

#include <sys/types.h>

namespace kist {

struct LioProcessConfig {
    bool        manage   = false;  // false = LIO run externally (run_lio_daemon); true = nav manages it
    std::string ros_setup   = "/opt/ros/humble/setup.bash";
    std::string ws_setup    = "/opt/lio_ws/install/setup.bash";
    std::string launch_file = "/workspace/kist-navigation-planner/lio_engine/lio_bringup.launch.py";
    std::string log_file    = "/tmp/lio_engine.log";
    double      ready_timeout_s = 20.0;  // give up relocalizing a fresh LIO after this (caller uses it)
};

class LioProcessManager {
public:
    LioProcessManager() = default;
    ~LioProcessManager() { stop(); }
    LioProcessManager(const LioProcessManager&) = delete;
    LioProcessManager& operator=(const LioProcessManager&) = delete;

    void configure(const LioProcessConfig& c) { cfg_ = c; }
    const LioProcessConfig& config() const { return cfg_; }

    // Start the engine (no-op if already started by us, or if manage=false). Returns true if the
    // engine is (being) run by us. fork()s a session leader that execs the sourced ros2 launch.
    bool start();
    // Stop the engine we started: SIGINT the whole process group, then SIGKILL, then reap. Also
    // sweeps the known node names as a backup (mirrors stop_lio_daemon). No-op if not ours.
    void stop();

    bool running() const;   // our child alive?

private:
    LioProcessConfig  cfg_;
    pid_t             pgid_ = -1;   // child session/group leader pid (== pgid via setsid); -1 = none
};

} // namespace kist
