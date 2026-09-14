#include "controller/path_follower.hpp"

#include <algorithm>
#include <cstddef>

namespace kist {

namespace {
double wrap_pi(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
}  // namespace

NavCommand PathFollower::compute(const Path& path, const Pose2D& pose, const DockConfig& dock,
                                 const Costmap* cm, float goal_yaw, FollowPhase* phase) {
    NavCommand cmd;   // zeros = stop
    auto set_phase = [&](FollowPhase p) { if (phase) *phase = p; };

    if (path.waypoints.empty()) { set_phase(FollowPhase::Arrived); return cmd; }

    const double rx = pose.x, ry = pose.y, ryaw = pose.yaw;
    const auto&  goal = path.waypoints.back();
    const double dist_goal = std::hypot(goal.first - rx, goal.second - ry);

    // Terminal-mode latch: reset when the goal moves (new click); set once the robot reaches
    // the goal. While latched we stay in align/approach/arrived even if the approach creep
    // pushes dist_goal back past arrival_tol (else it flips to Driving and oscillates).
    if (!have_last_ || std::hypot(goal.first - last_gx_, goal.second - last_gy_) > 0.3) {
        at_goal_ = false; have_last_ = true; last_gx_ = goal.first; last_gy_ = goal.second;
    }
    if (dist_goal <= cfg_.arrival_tol_m) at_goal_ = true;

    // ── terminal: rotate to the finish heading, then approach the object to a standoff ──
    // Both steps are per-destination (dock): each is skipped unless the destination enables it.
    // An all-default dock (rviz / ad-hoc goals) does neither -> just stop at arrival_tol.
    if (at_goal_) {
        // 1) align to the finish yaw (face the object)
        if (dock.align && !std::isnan(goal_yaw)) {
            const double yaw_err = wrap_pi(double(goal_yaw) - ryaw);
            if (std::abs(yaw_err) > cfg_.yaw_tol_rad) {
                set_phase(FollowPhase::Aligning);
                cmd.vyaw = std::clamp(cfg_.k_yaw * yaw_err, -cfg_.vyaw_max, cfg_.vyaw_max);
                return cmd;
            }
        }
        // 2) object approach: creep forward until the front object is standoff_m away
        if (dock.approach && cm && dock.standoff_m > 0.0f) {
            const float d = front_distance(*cm, pose, dock.trigger_m);
            if (d < dock.trigger_m && d > dock.standoff_m) {
                set_phase(FollowPhase::Approaching);
                cmd.vx = dock.speed;            // straight forward (already facing the object)
                return cmd;
            }
        }
        set_phase(FollowPhase::Arrived);        // aligned + at standoff (or open-space goal)
        return cmd;
    }

    // ── translate: steer toward a lookahead point on the path ──
    std::size_t ni = 0;                         // nearest waypoint to the robot
    double best = 1e18;
    for (std::size_t i = 0; i < path.waypoints.size(); ++i) {
        const double dx = path.waypoints[i].first - rx, dy = path.waypoints[i].second - ry;
        const double d2 = dx*dx + dy*dy;
        if (d2 < best) { best = d2; ni = i; }
    }
    std::size_t li = ni;                         // walk ahead by the lookahead distance
    double acc = 0.0;
    for (std::size_t i = ni + 1; i < path.waypoints.size(); ++i) {
        acc += std::hypot(path.waypoints[i].first  - path.waypoints[i-1].first,
                          path.waypoints[i].second - path.waypoints[i-1].second);
        li = i;
        if (acc >= cfg_.lookahead_m) break;
    }
    const auto&  look = path.waypoints[li];
    const double dx = look.first - rx, dy = look.second - ry;

    // odom -> robot base frame
    const double c = std::cos(ryaw), s = std::sin(ryaw);
    const double fwd  =  c*dx + s*dy;
    const double left = -s*dx + c*dy;
    const double head_err = std::atan2(left, fwd);

    cmd.vyaw = std::clamp(cfg_.k_yaw * head_err, -cfg_.vyaw_max, cfg_.vyaw_max);
    double speed = cfg_.v_max * std::min(1.0, dist_goal / std::max(1e-3, cfg_.slow_radius_m));
    speed = std::max(speed, cfg_.min_speed_m);   // loco floor: keep it moving until arrival
    const double mag = std::hypot(fwd, left);
    if (mag > 1e-6) {
        if (cfg_.allow_strafe) {                 // holonomic: both axes toward the lookahead
            cmd.vx = speed * fwd  / mag;
            cmd.vy = speed * left / mag;
        } else {                                 // car-like: forward only
            cmd.vx = speed * std::max(0.0, fwd / mag);
        }
    }
    set_phase(FollowPhase::Driving);
    return cmd;
}

bool path_ahead_blocked(const Costmap& cm, const Path& path, const Pose2D& pose,
                        float ahead_m, uint8_t lethal) {
    if (cm.empty() || path.waypoints.empty()) return false;

    // nearest waypoint to the robot
    std::size_t ni = 0; double best = 1e18;
    for (std::size_t i = 0; i < path.waypoints.size(); ++i) {
        const double dx = path.waypoints[i].first - pose.x, dy = path.waypoints[i].second - pose.y;
        const double d2 = dx*dx + dy*dy;
        if (d2 < best) { best = d2; ni = i; }
    }
    // walk forward up to ahead_m, sampling each segment at ~half a cell
    const float step = std::max(0.02f, 0.5f * cm.resolution);
    double travelled = 0.0;
    for (std::size_t i = ni; i + 1 < path.waypoints.size() && travelled < ahead_m; ++i) {
        const float ax = path.waypoints[i].first,  ay = path.waypoints[i].second;
        const float bx = path.waypoints[i+1].first, by = path.waypoints[i+1].second;
        const float seg = std::hypot(bx-ax, by-ay);
        const int   K   = std::max(1, int(seg / step));
        for (int k = 0; k <= K; ++k) {
            const float t = float(k) / K;
            int ix, iy;
            if (!cm.world_to_cell(ax + t*(bx-ax), ay + t*(by-ay), ix, iy)) return true;  // off-map
            if (cm.at(ix, iy) >= lethal) return true;                                     // blocked
        }
        travelled += seg;
    }
    return false;
}

float front_distance(const Costmap& cm, const Pose2D& pose, float max_m, uint8_t lethal) {
    if (cm.empty()) return 0.0f;
    const float cx = std::cos(pose.yaw), sy = std::sin(pose.yaw);   // heading direction
    const float step = std::max(0.02f, 0.5f * cm.resolution);
    for (float d = 0.0f; d <= max_m; d += step) {
        int ix, iy;
        if (!cm.world_to_cell(pose.x + d*cx, pose.y + d*sy, ix, iy)) return d;   // off-map = wall
        if (cm.at(ix, iy) >= lethal) return d;                                    // hit the object
    }
    return max_m;   // nothing within range
}

} // namespace kist

