#include "controller/local_controller.hpp"

#include <cmath>
#include <limits>

namespace kist {

namespace {
double quat_yaw(const Eigen::Quaterniond& q) {
    return std::atan2(2 * (q.w()*q.z() + q.x()*q.y()), 1 - 2 * (q.y()*q.y() + q.z()*q.z()));
}
}  // namespace

NavCommand LocalController::step(const Path* path, const RobotTransforms* tf, const Costmap* cm,
                                 const Goal* goal, FollowPhase* phase) {
    auto set_phase = [&](FollowPhase p) { if (phase) *phase = p; };
    NavCommand cmd;   // zeros = stop (default/safe)

    if (!path || path->waypoints.empty() || !tf) { set_phase(FollowPhase::Arrived); return cmd; }

    const Pose2D pose{ float(tf->T_odom_pelvis.translation.x()),
                       float(tf->T_odom_pelvis.translation.y()),
                       float(quat_yaw(tf->T_odom_pelvis.rotation)) };

    float      goal_yaw = std::numeric_limits<float>::quiet_NaN();
    DockConfig dock;                                 // default = off (ad-hoc / no goal)
    if (goal && goal->valid) {
        dock = goal->dock;
        if (goal->has_yaw) goal_yaw = goal->yaw;     // NaN otherwise -> align skipped
    }

    FollowPhase ph = FollowPhase::Arrived;
    cmd = follower_.compute(*path, pose, dock, cm, goal_yaw, &ph);

    // Reactive sudden-obstacle stop: ONLY while actively following the path. The terminal
    // approach deliberately creeps toward the goal object (held off by its own front_distance
    // standoff), so path_ahead_blocked — which sees that same object on the path ahead — must
    // not veto it, or the robot would stop before reaching its standoff.
    if (ph == FollowPhase::Driving && cm &&
        path_ahead_blocked(*cm, *path, pose, follower_.config().react_ahead_m, lethal_)) {
        cmd = NavCommand{};                          // sudden obstacle -> stop
        ph  = FollowPhase::Blocked;
    }
    set_phase(ph);
    return cmd;
}

}  // namespace kist
