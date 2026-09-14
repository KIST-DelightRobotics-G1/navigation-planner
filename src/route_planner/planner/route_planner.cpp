#include "route_planner/planner/route_planner.hpp"

#include "route_planner/perception/costmap_builder/clearance.hpp"

namespace kist {

Path RoutePlanner::plan(const Costmap& cm,
                        std::pair<float, float> start, std::pair<float, float> goal) const {
    if (cm.empty()) return Path{};

    const std::vector<float> clr = clearance_field(cm);   // shared by A* cost + smoother
    Path p = AStarPlanner(acfg).plan(cm, clr, start, goal);
    if (!p.empty())
        RouteSmoother(scfg).smooth(cm, clr, p.raw_waypoints, p.waypoints, p.turn_circles);
    return p;
}

} // namespace kist
