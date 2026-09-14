#pragma once

// LocalController — the per-tick control DECISION, pulled out of NavSystem so the system layer
// only orchestrates (buffers + threads) while the control logic lives here. Given the latest
// path, robot transforms, costmap, and active goal, it produces one base-frame velocity command:
// extract the 2D pose, decode the goal's dock (align/approach) + finish yaw, run the pure-pursuit
// follower, and apply the reactive-stop safety gate. Any missing/stale input (null) -> safe stop.

#include "controller/nav_command.hpp"
#include "controller/path_follower.hpp"
#include "goal_generation/goal.hpp"
#include "lio/robot_transforms.hpp"
#include "route_planner/planner/astar_planner/path.hpp"
#include "route_planner/perception/costmap_builder/costmap.hpp"

#include <cstdint>

namespace kist {

class LocalController {
public:
    explicit LocalController(FollowConfig fc = {}) : follower_(fc) {}

    // One control step. Inputs are nullable — a null/empty path or null tf is a safe stop.
    // `cm` (nullable) enables reactive-stop + the object-approach standoff; `goal` (nullable)
    // supplies the per-destination dock + finish yaw (used only when valid). Reports the phase
    // (out, optional). Stateful via the follower's terminal latch.
    NavCommand step(const Path* path, const RobotTransforms* tf, const Costmap* cm,
                    const Goal* goal, FollowPhase* phase = nullptr);

    const FollowConfig& config() const { return follower_.config(); }
    void set_config(const FollowConfig& c) { follower_.set_config(c); }

private:
    PathFollower follower_;
    uint8_t      lethal_ = 254;   // reactive-stop cost threshold (= costmap lethal_cost)
};

}  // namespace kist
