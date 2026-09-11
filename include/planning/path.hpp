#pragma once

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace kist {

// A planned path: waypoints in the odom/map frame (same frame as the costmap),
// start -> goal. After smoothing these are the straight segments joined by circular
// arcs; turn_circles holds each corner arc's (centre_x, centre_y, radius) for debug.
struct Path {
    int64_t stamp_ns = 0;
    std::vector<std::pair<float, float>> waypoints;      // (x, y) odom — SMOOTHED (LOS + arcs)
    std::vector<std::pair<float, float>> raw_waypoints;  // the raw A* grid path (pre-smoothing)
    std::vector<std::array<float, 3>>    turn_circles;   // (cx, cy, R) per fitted corner arc

    bool   empty() const { return waypoints.empty(); }
    std::size_t size() const { return waypoints.size(); }
};

} // namespace kist
