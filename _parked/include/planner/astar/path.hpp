#pragma once

#include <cstdint>
#include <utility>
#include <vector>

namespace kist {

// A planned path: waypoints in the world/map frame (same frame as the costmap).
struct Path {
    int64_t stamp_ns = 0;
    std::vector<std::pair<float, float>> waypoints;   // (x, y) world, start -> goal

    bool   empty() const { return waypoints.empty(); }
    size_t size()  const { return waypoints.size(); }
};

} // namespace kist
