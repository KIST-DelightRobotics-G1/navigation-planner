#pragma once

// 2D correlative scan matching: refine a prior pose (UWB/odom) by aligning the
// current LiDAR scan to the accumulated occupancy grid, over a SMALL window. The
// window is sized to the walking head-bob, so the match only removes that bob /
// short-term error while staying tied to the drift-free UWB pose globally.
//
// Method: build a likelihood field from the grid's occupied cells (distance
// transform -> Gaussian), then brute-force the (dx, dy, dyaw) that maximises the
// sum of likelihoods at the transformed scan points. Cheap, derivative-free,
// robust to the coarse initial guess. Runs BEFORE grid integration so the scan
// is anchored with the corrected pose.

#include "occupancy_grid/occupancy_grid.hpp"   // Pose2D, OccupancyGrid, GridConfig

#include <vector>

namespace kist {

struct ScanMatchConfig {
    bool  enabled          = true;
    float search_xy_m      = 0.20f;   // translation half-window (>= expected bob)
    float search_yaw_deg   = 6.0f;    // rotation half-window
    float step_xy_m        = 0.025f;  // translation search step
    int   yaw_steps        = 13;      // yaw candidates across the window (odd -> includes 0)
    float sigma_m          = 0.10f;   // likelihood falloff vs distance-to-obstacle
    int   min_scan_points  = 120;     // fewer -> skip (return prior)
    int   min_map_cells    = 300;     // sparser map -> skip (nothing to match to yet)
};

// Refine `prior` by matching the robot-frame 2D scan (packed [x0,y0,x1,y1,...])
// to the grid. Returns `prior` unchanged when disabled or data is insufficient.
Pose2D scan_match(const OccupancyGrid& g, const GridConfig& gcfg,
                  const std::vector<float>& scan_xy,
                  const Pose2D& prior, const ScanMatchConfig& cfg);

} // namespace kist
