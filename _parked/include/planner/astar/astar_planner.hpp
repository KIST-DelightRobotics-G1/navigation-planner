#pragma once

// 8-connected weighted A* over the costmap. Cells at or above obs_cost are
// impassable; the per-cell cost is added to the step so the path prefers the
// low-cost (well-clear) route. Heuristic is weighted Chebyshev distance.
// start/goal are in the world/map frame (the costmap's frame). A stateless
// Module: plan() per (costmap, start, goal).

#include "planner/astar/path.hpp"
#include "planner/costmap/costmap.hpp"

#include <cstdint>
#include <utility>

namespace kist {

struct AStarConfig {
    float base_cost        = 1.0f;    // base traversal cost per cell (> 0)
    int   obs_cost         = 254;     // cells with cost >= this are impassable
    float heuristic_weight = 1.2f;    // f = g + w*h  (>= 1.0; higher = faster, less optimal)
    int   max_iterations   = 400000;  // cap; empty path if exceeded

    // Distance trust: the obstacle (soft) cost is applied in full near the robot
    // and discounted with range, so the near path is shaped carefully while the
    // far part stays provisional (the robot replans as it advances). Impassable
    // (>= obs_cost) cells stay blocked at ANY range — only the soft gradient fades.
    float trust_falloff_m  = 3.0f;    // soft-cost weight decays over this range (m); 0 = off (uniform)
    float trust_min        = 0.2f;    // floor: far soft cost keeps at least this fraction of its weight

    // Path-length cap: reject any route whose accumulated travel distance (m,
    // following the actual winding path, not straight-line) exceeds this. Stops
    // the planner committing to a long way around an obstacle — a goal only
    // reachable by a longer detour yields no path (drive closer, replan). 0 = off.
    float max_path_len_m   = 0.0f;

    // Goal-approach: within this radius (m) of the goal, ignore the costmap cost
    // AND the impassable check — so a destination deliberately placed next to an
    // obstacle (a desk, a fridge front, where the robot actually stood) stays
    // reachable through its inflation zone. Keep it small (the goal footprint).
    // 0 = off (normal cost + lethal block everywhere).
    float goal_ignore_radius_m = 0.0f;
};

class AStarPlanner {
public:
    explicit AStarPlanner(AStarConfig cfg = {}) : cfg_(cfg) {}

    // Plan start -> goal (world coords). Empty path if unreachable / blocked /
    // out of bounds / start==goal.
    Path plan(const Costmap& cm,
              std::pair<float, float> start,
              std::pair<float, float> goal) const;

private:
    AStarConfig cfg_;
};

} // namespace kist
