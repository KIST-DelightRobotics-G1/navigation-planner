#pragma once

// Probabilistic semantic occupancy grid — a world-anchored top-down grid that
// fuses the LiDAR cloud (geometry, wide/far) and the labeled camera cloud
// (geometry + class, dense/near) into two per-cell beliefs:
//   * occupancy  P(occ)   — via log-odds; both sensors add evidence
//                           L += l_hit;  P = 1/(1+e^-L);  clamped + decayed
//   * class      argmax    — dominant class from the camera's votes
// The clouds arrive in the ROBOT frame; each point is lifted to the WORLD frame
// with the robot pose (x,y,yaw) before it marks a cell, so the map is stable as
// the robot moves and temporal accumulation is valid. Grid-level (not point-
// level) fusion normalizes the sensors' different densities and fields of view.
// This is the header data + the update math (occupancy_grid.cpp);
// OccupancyGridBuilder runs it on a worker thread.

#include "labeled_cloud/labeled_cloud.hpp"        // LabeledCloud, kNoClass
#include "unitree/unitree_pointcloud.hpp"         // UnitreePointCloud

#include <cmath>
#include <cstdint>
#include <vector>

namespace kist {

// Robot world pose used to lift a robot-frame point into the world frame.
struct Pose2D { float x = 0.f, y = 0.f, yaw = 0.f; };

struct GridConfig {
    float half_extent_m = 5.0f;    // grid is 2*half wide/tall, centered on the anchor
    float resolution_m  = 0.05f;   // metres per cell (0.05 -> 200x200 at 5m)
    float min_z = 0.2f, max_z = 1.8f;   // obstacle height band (robot frame)

    // Inverse sensor model — log-odds added per hit. LiDAR is trusted more;
    // the camera's trust falls off with range (far depth is noisy).
    float l_hit_lidar    = 0.85f;
    float l_hit_cam_near = 1.10f;
    float l_hit_cam_far  = 0.20f;
    float cam_d_max      = 4.0f;   // range (m) at which camera trust hits _far
    float l_min = -4.0f, l_max = 4.0f;  // log-odds clamp -> P in [0.018, 0.982]

    // Temporal decay toward P=0.5, per update. A cell decays at the DYNAMIC rate
    // when its dominant class is dynamic (people move -> forget fast), else at the
    // STATIC rate (walls persist). is_dynamic[c] flags a class id as dynamic;
    // unlabeled (LiDAR-only) cells default to static.
    float decay_static  = 0.98f;   // slow -> long memory
    float decay_dynamic = 0.85f;   // fast -> short memory
    bool  is_dynamic[256] = {};    // per class id; filled from config

    float occ_threshold = 0.65f;   // P(occ) above which a cell reads "occupied"
};

// World-anchored grid: cell (ix,iy) maps to world (origin + i*res). +X, +Y are
// the world axes; the robot's own world position is carried for rendering.
struct OccupancyGrid {
    int     n = 0;                 // n x n cells
    float   resolution = 0.05f;    // m/cell
    float   origin_x = 0.f, origin_y = 0.f;   // world coord of cell (0,0) corner
    float   robot_x = 0.f, robot_y = 0.f;     // robot world position at build time
    float   robot_yaw = 0.f;                  // robot heading (rad) at build time
    int64_t stamp_ns = 0;

    std::vector<float>   log_odds;    // n*n — occupancy belief
    std::vector<uint8_t> label;       // n*n — dominant class (kNoClass = none)
    std::vector<float>   label_vote;  // n*n — majority-vote strength

    int  index(int ix, int iy) const { return iy * n + ix; }
    bool world_to_cell(float wx, float wy, int& ix, int& iy) const {
        ix = int(std::floor((wx - origin_x) / resolution));
        iy = int(std::floor((wy - origin_y) / resolution));
        return ix >= 0 && ix < n && iy >= 0 && iy < n;
    }
    float prob(int idx) const { return 1.0f / (1.0f + std::exp(-log_odds[idx])); }
    bool  empty() const { return log_odds.empty(); }
};

// Allocate/clear the grid to the configured size, centered on (center_x,center_y)
// in world coords (log_odds 0 = P 0.5, no label).
void grid_reset(OccupancyGrid& g, const GridConfig& cfg, float center_x, float center_y);

// One temporal step: pull every cell toward the prior (log_odds *= decay,
// label_vote *= decay). Call once per integration cycle before adding new hits.
void grid_decay(OccupancyGrid& g, const GridConfig& cfg);

// Add LiDAR evidence: each robot-frame point is lifted to world via `pose`, and
// (if in the height band + inside the grid) raises its cell's occupancy.
void grid_integrate_lidar(OccupancyGrid& g, const UnitreePointCloud& cloud,
                          const Pose2D& pose, const GridConfig& cfg);

// Add camera evidence: like the LiDAR path, plus a class vote per point
// (Boyer-Moore majority for the cell's dominant class); range-weighted trust.
void grid_integrate_labeled(OccupancyGrid& g, const LabeledCloud& cloud,
                            const Pose2D& pose, const GridConfig& cfg);

} // namespace kist
