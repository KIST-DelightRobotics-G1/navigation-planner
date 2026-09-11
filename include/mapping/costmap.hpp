#pragma once

// Costmap — the ObstacleGrid turned into a navigation cost field: obstacles inflated by
// the robot radius (lethal) plus a decaying soft zone around them, so the planner keeps
// clearance. Built from the EDT (distance to nearest obstacle) via a distance->cost LUT.
// Same top-down odom geometry as the ObstacleGrid. LiDAR-only (no semantic / people layer
// yet — that arrives with the camera).

#include <cstdint>
#include <vector>

namespace kist {

struct CostmapConfig {
    float   lethal_radius_m    = 0.30f;   // robot footprint + safety -> impassable
    float   influence_radius_m = 1.00f;   // soft-zone extent (must be > lethal)
    uint8_t lethal_cost        = 254;     // cost of the lethal zone (blocked)
    uint8_t max_soft_cost      = 200;     // cost at the lethal boundary, decays to 0
    float   decay              = 3.0f;    // soft = max_soft * exp(-decay*(d - lethal))

    // Noise removal: an occupied cell needs at least this many occupied 8-neighbours to
    // count as an obstacle. Sparse isolated returns otherwise inflate into lethal disks
    // that wall off free space; dense walls survive. 0 = off.
    int     obstacle_min_neighbors = 2;
};

// 0 = free .. max_soft_cost .. lethal_cost = impassable.
struct Costmap {
    int     n = 0;                          // n x n cells
    float   resolution = 0.05f;             // m/cell
    float   origin_x = 0.f, origin_y = 0.f; // odom coord of cell (0,0) corner
    float   robot_x = 0.f, robot_y = 0.f, robot_yaw = 0.f;
    int64_t stamp_ns = 0;
    std::vector<uint8_t> cells;             // n*n, row-major (iy*n + ix)

    int  index(int ix, int iy) const { return iy * n + ix; }
    bool empty() const { return cells.empty(); }
    bool world_to_cell(float wx, float wy, int& ix, int& iy) const {
        ix = int((wx - origin_x) / resolution);
        iy = int((wy - origin_y) / resolution);
        return ix >= 0 && ix < n && iy >= 0 && iy < n;
    }
    uint8_t at(int ix, int iy) const { return cells[index(ix, iy)]; }
};

} // namespace kist
