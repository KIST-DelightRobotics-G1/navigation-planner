#pragma once

// RouteSmoother — turns the raw A* grid route into the driven SHAPE: line-of-sight
// straightening (longest clear straight runs, EDT-checked) + the largest feasible circular
// ARC at each corner (grow R until the arc + clearance would hit an obstacle; max R = min
// curvature = gentlest safe turn). Straight/failed corners stay sharp. This is the geometry
// stage — the A* stage owns only the route (which corridors). (Curvature jumps straight->
// arc; a clothoid pass is future.)

#include "route_planner/perception/costmap_builder/costmap.hpp"

#include <array>
#include <utility>
#include <vector>

namespace kist {

struct RouteSmoothConfig {
    float d_safe_m = 0.25f;               // required clearance (m) beyond the costmap's lethal
                                          // inflation — a straight/arc keeps EDT >= this.
                                          // 0.25 tuned on the real G1 (turns stay off walls).
    float corner_angle_min_deg = 10.0f;   // |turn| below this -> keep straight (no arc)
    float r_min_m  = 0.20f;               // arc-radius search: r_max down to r_min, first
    float r_max_m  = 2.00f;               // (largest) feasible R wins = minimum curvature.
    float r_step_m = 0.05f;
};

class RouteSmoother {
public:
    explicit RouteSmoother(RouteSmoothConfig cfg = {}) : cfg_(cfg) {}

    // Smooth `raw` (odom waypoints) against the costmap + clearance field. Writes the
    // straight+arc waypoints to out_waypoints and each corner arc's (cx,cy,R) to out_circles.
    void smooth(const Costmap& cm, const std::vector<float>& clr,
                const std::vector<std::pair<float, float>>& raw,
                std::vector<std::pair<float, float>>& out_waypoints,
                std::vector<std::array<float, 3>>& out_circles) const;

    const RouteSmoothConfig& config() const { return cfg_; }

private:
    RouteSmoothConfig cfg_;
};

} // namespace kist
