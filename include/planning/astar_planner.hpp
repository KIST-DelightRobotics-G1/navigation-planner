#pragma once

// 8-connected weighted A* for the ROUTE (topology), then a geometric smoother for the
// SHAPE: line-of-sight straightening (longest clear straight runs, EDT-checked) + the
// largest feasible circular ARC at each corner (grow R until the arc + clearance would hit
// an obstacle; max R = minimum curvature = the gentlest safe turn). So the path is long
// straights joined by wide smooth arcs. start/goal are in the odom/map (costmap) frame.
// Stateless: plan() per call. (Curvature jumps straight->arc; a clothoid pass is future.)

#include "planning/path.hpp"
#include "mapping/costmap.hpp"

#include <cstdint>
#include <utility>

namespace kist {

struct AStarConfig {
    float base_cost        = 1.0f;    // base traversal cost per cell (> 0) — the length term
    int   obs_cost         = 254;     // cells with cost >= this are impassable (= lethal)
    float heuristic_weight = 1.2f;    // f = g + w*h  (>= 1.0; higher = faster, less optimal)
    int   max_iterations   = 400000;  // cap; empty path if exceeded

    // Centre preference (A* is the ROUTE only; the smoother owns the shape). Penalise cells
    // with clearance BELOW ref_clr_m, so the route runs through the middle of a corridor;
    // beyond ref_clr_m the penalty is 0 (barrier philosophy — no reward for hugging the exact
    // ridge, which would only add detours; the LOS smoother centres the wide bits).
    float w_center   = 15.0f;         // weight of the clearance penalty (0 = pure shortest path)
    float ref_clr_m  = 1.00f;         // clearance (m) at/above which a cell is "central enough"

    // Within this radius (m) of the goal, ignore the costmap cost AND the impassable check,
    // so a goal placed next to a wall (in its inflation zone) stays reachable. 0 = off.
    float goal_ignore_radius_m = 0.30f;

    // ── smoothing: LOS straighten + max-radius circular-arc corners ──
    bool  smooth = true;
    float d_safe_m = 0.10f;               // required clearance (m) BEYOND the costmap's lethal
                                          // inflation — a straight/arc must keep the EDT >= this.
                                          // (Don't re-add the robot radius: the costmap already
                                          // inflated by it; this is the extra safety margin.)
    float corner_angle_min_deg = 10.0f;   // |turn| below this -> keep it straight (no arc)
    float r_min_m  = 0.20f;               // arc-radius search range: from r_max down to r_min,
    float r_max_m  = 2.00f;               // the first (largest) feasible R wins = min curvature.
    float r_step_m = 0.05f;
};

class AStarPlanner {
public:
    explicit AStarPlanner(AStarConfig cfg = {}) : cfg_(cfg) {}

    // Plan start -> goal (odom coords). Empty path if unreachable / blocked / out of
    // bounds / start == goal. The returned waypoints are already smoothed.
    Path plan(const Costmap& cm,
              std::pair<float, float> start,
              std::pair<float, float> goal) const;

    const AStarConfig& config() const { return cfg_; }

private:
    AStarConfig cfg_;
};

} // namespace kist
