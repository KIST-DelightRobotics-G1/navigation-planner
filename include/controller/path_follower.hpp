#pragma once

// PathFollower — pure-pursuit path tracking. Given the planned Path (odom waypoints) + the
// robot's odom pose, it produces a base-frame velocity command (vx, vy, vyaw): steer toward
// a lookahead point on the path, taper speed near the goal, and on arrival rotate in place
// to the finish heading (if given). Stateless: compute() per control tick. Producing the
// command does NOT move the robot — publishing it (Twist -> gearsonic) does, and that lives
// behind the controller worker's safety gate.

#include "controller/nav_command.hpp"
#include "route_planner/astar_planner/path.hpp"
#include "route_planner/costmap_builder/costmap.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace kist {

struct FollowConfig {
    double v_max         = 0.35;   // m/s cruise speed
    double min_speed_m   = 0.20;   // controller floor: a nonzero translate command is bumped up
                                   // to this (the loco stack ignores speeds below ~0.2 m/s)
    double vyaw_max      = 0.8;    // rad/s max turn rate
    double lookahead_m   = 0.4;    // pure-pursuit lookahead distance
    double arrival_tol_m = 0.20;   // within this of the goal -> stop translating, align yaw
    double yaw_tol_rad   = 0.087;  // ~5 deg; within this of the goal yaw -> arrived (stop)
    double k_yaw         = 1.5;    // steering / rotate proportional gain
    double slow_radius_m = 0.5;    // taper forward speed within this of the goal
    bool   allow_strafe  = true;   // holonomic (vx+vy toward lookahead); false = car-like (vx only)
    float  react_ahead_m = 1.0;    // reactive-stop lookahead: stop if the path is blocked within this
    // The terminal dock behavior (yaw align + object approach/standoff) is NOT here — it is
    // per-destination and travels with the Goal as a DockConfig (see controller/nav_command.hpp).
};

class PathFollower {
public:
    explicit PathFollower(FollowConfig cfg = {}) : cfg_(cfg) {}

    // One control step. `dock` is the destination's terminal behavior (align/approach on/off +
    // standoff); an all-default DockConfig means "just stop at arrival_tol" (ad-hoc goals). `cm`
    // (nullable) is needed by the approach phase to measure the front distance. `goal_yaw` is the
    // finish heading (rad, odom); NaN (or dock.align=false) skips the rotate phase. `phase` (out,
    // optional) reports state. Stateful: once the robot reaches the goal it LATCHES into the
    // terminal (align -> approach -> arrived) mode and won't fall back to path-following (which
    // would oscillate when the approach creeps past the goal). The latch resets when the goal moves.
    NavCommand compute(const Path& path, const Pose2D& pose, const DockConfig& dock,
                       const Costmap* cm = nullptr,
                       float goal_yaw = std::numeric_limits<float>::quiet_NaN(),
                       FollowPhase* phase = nullptr);

    const FollowConfig& config() const { return cfg_; }
    void set_config(const FollowConfig& c) { cfg_ = c; }

private:
    FollowConfig cfg_;
    bool  at_goal_   = false;   // latched terminal mode (align/approach/arrived)
    bool  have_last_ = false;
    float last_gx_ = 0.f, last_gy_ = 0.f;   // last goal, to detect a new one
};

// Reactive safety: is the path within `ahead_m` of the robot blocked? Walks the path from
// the robot forward by ahead_m and returns true if any sampled cell has cost >= lethal (a
// sudden obstacle on the immediate route). Runs at the control rate — far faster than the
// global replan — so the robot stops before the planner reroutes. Off-costmap = blocked.
bool path_ahead_blocked(const Costmap& cm, const Path& path, const Pose2D& pose,
                        float ahead_m, uint8_t lethal = 254);

// Distance (m) from the robot centre to the nearest lethal cell straight ahead (along the
// robot heading), searched up to max_m. Returns max_m if nothing is within range. Used by
// the approach phase to hold a standoff from the object in front.
float front_distance(const Costmap& cm, const Pose2D& pose, float max_m, uint8_t lethal = 254);

} // namespace kist
