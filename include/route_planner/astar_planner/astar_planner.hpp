#pragma once

// AStarPlanner — the ROUTE stage. 8-connected weighted A* over the costmap, biased toward
// the corridor centre via the clearance field (penalise cells below ref_clr_m; barrier
// philosophy, no reward past it). Returns the RAW grid route only — the geometry (straighten
// + arcs) is RouteSmoother's job. start/goal are in the odom/map (costmap) frame; the
// clearance field is passed in (computed once, shared with the smoother). Stateless.

#include "route_planner/astar_planner/path.hpp"
#include "route_planner/costmap_builder/costmap.hpp"

#include <cstdint>
#include <utility>
#include <vector>

namespace kist {

struct AStarConfig {
    float base_cost        = 1.0f;    // base traversal cost per cell (> 0) — the length term
    int   obs_cost         = 254;     // cells with cost >= this are impassable (= lethal)
    float heuristic_weight = 1.2f;    // f = g + w*h  (>= 1.0; higher = faster, less optimal)
    int   max_iterations   = 400000;  // cap; empty path if exceeded

    // Within this radius (m) of the goal, ignore the cost + impassable check, so a goal next
    // to a wall (in its inflation zone) stays reachable. 0 = off.
    float goal_ignore_radius_m = 0.30f;

    // Centre preference: penalise cells with clearance below ref_clr_m; 0 beyond (barrier).
    float w_center   = 15.0f;         // weight of the clearance penalty (0 = pure shortest path)
    float ref_clr_m  = 1.00f;         // clearance (m) at/above which a cell is "central enough"
};

class AStarPlanner {
public:
    explicit AStarPlanner(AStarConfig cfg = {}) : cfg_(cfg) {}

    // Plan start -> goal (odom coords). `clr` is the per-cell clearance (m, distance to the
    // nearest lethal cell). Empty path if unreachable / blocked / out of bounds. waypoints
    // and raw_waypoints are both the raw grid route (no smoothing here).
    Path plan(const Costmap& cm, const std::vector<float>& clr,
              std::pair<float, float> start, std::pair<float, float> goal) const;

    const AStarConfig& config() const { return cfg_; }

private:
    AStarConfig cfg_;
};

} // namespace kist
