#pragma once

// RoutePlanner — the PLANNING half: given a Costmap snapshot (produced by ObstacleMapper) plus
// start/goal, produce a Path. Stateless per call: clearance field -> A* route (centre
// preference) -> bubble smoother (LOS straighten + max-radius arc). Folder separation is code
// organization, NOT threading — the planner thread (system/) drives plan() at the replan rate,
// reading the latest Costmap the perception half published.

#include "route_planner/perception/costmap_builder/costmap.hpp"   // Costmap = handoff from perception
#include "route_planner/planner/astar_planner/astar_planner.hpp"
#include "route_planner/planner/astar_planner/path.hpp"
#include "route_planner/planner/route_smoothing/route_smoother.hpp"

#include <utility>

namespace kist {

class RoutePlanner {
public:
    RoutePlanner() = default;

    // ── stage configs (tweak before/while running; read on the planner thread) ──
    AStarConfig       acfg;      // route (centre preference)
    RouteSmoothConfig scfg;      // shape (LOS + arc)

    // Stateless route planning on a costmap snapshot (carries the robot pose as `start` here).
    Path plan(const Costmap& cm, std::pair<float, float> start, std::pair<float, float> goal) const;
};

} // namespace kist
