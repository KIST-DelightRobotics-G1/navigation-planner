#pragma once

// ObstacleGrid — the 2D top-down occupancy grid the planner/costmap consumes. It is the
// PROJECTION (column-max) of the 3D ObstacleVoxelGrid: a cell is occupied if any voxel
// in its vertical column is occupied. This 2D form is the output/wire type (publisher,
// costmap); the accumulation lives in obstacle_voxel_grid.hpp.
//
// Why voxel-then-project (not flat 2D): flat 2D ray-clearing collapses height, so a beam
// flying OVER a low obstacle (a desk) to the wall behind carves the desk's footprint and
// it flickers/erases, leaving a phantom gap the planner routes through. In 3D the beam
// clears only the voxels it truly passes (its real height), preserving the desk voxel;
// the column projection then keeps the footprint solid. (Same idea as ROS VoxelLayer.)
//
// Contract with the LIO pipeline (unchanged): per-scan registered cloud in odom; ray
// origin = T_odom_lidar (3D, at the scan's stamp); height band measured above the floor
// (floor_z from the stable T_odom_pelvis). LiDAR-only (camera last).

#include <cmath>
#include <cstdint>
#include <vector>

namespace kist {

struct ObstacleGridConfig {
    float half_extent_m = 5.0f;    // grid spans 2*half; 5 m -> 10x10 m window
    float resolution_m  = 0.05f;   // xy metres per cell (0.05 -> 200x200 at 5 m)
    float resolution_z_m = 0.10f;  // z metres per voxel bin (vertical resolution)

    // Obstacle height band. The BOTTOM is a fixed margin above the floor (clears the floor
    // + its noise). The TOP follows the LiDAR (= the robot's head height): points above
    // (lidar_z - upper_margin_below_lidar_m) are ignored, so overhead / ceiling clutter
    // above the robot is dropped and the cap tracks the sensor as the robot bobs. The
    // effective band is [floor + min_height_m, lidar - upper_margin_below_lidar_m].
    float min_height_m = 0.16f;               // above the floor (at zero range)
    // Range-dependent floor cut: the effective bottom is min_height_m + this * range.
    // Far floor returns scatter UP (grazing incidence) and cross a flat cut, painting a
    // black annulus at the floor's coverage radius; lifting the cut with range removes
    // them while keeping the low cut for NEAR obstacles (small range -> ~min_height_m).
    float floor_cut_slope_m_per_m = 0.04f;
    float upper_margin_below_lidar_m = 0.30f; // below the LiDAR (dynamic top) -> band top
                                              // = lidar_z - 0.30 (~1.06 m above floor); clears
                                              // the head-mounted camera clutter near the sensor
    // Voxel-grid vertical ALLOCATION ceiling (above floor). Must sit above the sensor so
    // the sensor voxel + the dynamic cap fit; the real marking top is the dynamic one above.
    float max_height_m = 1.80f;

    // Standing pelvis height above the floor (m). floor_z_odom defaults to
    // T_odom_pelvis.z - this; tracks the robot even as odom z drifts. CALIBRATED on the
    // real G1 via the floor probe (p05 -> 0 with the dense registered cloud).
    float pelvis_stand_height_m = 0.89f;

    // Inverse sensor model (log-odds), per voxel.
    float l_hit  = 0.85f;          // endpoint (obstacle) evidence
    float l_miss = 0.40f;          // free-space carve per traversed voxel; 0 = off. Safe
                                   // to keep strong now — 3D clearing no longer erases
                                   // low obstacles a beam flies over.
    float l_min  = -4.0f;
    float l_max  =  4.0f;          // clamp -> P(occ) in [0.018, 0.982]

    // Temporal decay toward P=0.5 per integration cycle (slow -> long memory). 0.96 ~= a
    // 5 s memory at 10 Hz: an occupied cell that is NEVER re-observed fades in ~5 s. This
    // is only the fallback GC — a moving person is removed far faster by ray-clearing when
    // the vacated space is re-observed (a scan or two), not by decay.
    float decay = 0.96f;

    float occ_threshold = 0.65f;   // P(occ) above which a projected cell reads "occupied"
    float max_range_m   = 30.0f;   // ignore points beyond this from the ray origin
    // Self-footprint: drop points within this horizontal radius of the sensor — they are
    // the robot's own body (torso/shoulders under the head-mounted lidar), not obstacles.
    float self_radius_m = 0.35f;
};

// Odom-anchored 2D grid (projection output). Cell (ix,iy) covers odom
// [origin + i*res, origin + (i+1)*res).
struct ObstacleGrid {
    int     n = 0;                             // n x n cells
    float   resolution = 0.10f;                // m/cell
    float   origin_x = 0.f, origin_y = 0.f;    // odom coord of cell (0,0) corner
    float   robot_x = 0.f, robot_y = 0.f;      // robot (pelvis) odom position — render
    float   robot_yaw = 0.f;                   // robot heading (rad) — render
    int64_t stamp_ns = 0;
    std::vector<float> log_odds;               // n*n, row-major (iy*n + ix) — column-max

    int  index(int ix, int iy) const { return iy * n + ix; }
    bool world_to_cell(float wx, float wy, int& ix, int& iy) const {
        ix = int(std::floor((wx - origin_x) / resolution));
        iy = int(std::floor((wy - origin_y) / resolution));
        return ix >= 0 && ix < n && iy >= 0 && iy < n;
    }
    float prob(int idx) const { return 1.0f / (1.0f + std::exp(-log_odds[idx])); }
    bool  empty() const { return log_odds.empty(); }
};

} // namespace kist
