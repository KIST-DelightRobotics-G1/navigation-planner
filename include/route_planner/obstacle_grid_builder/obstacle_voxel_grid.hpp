#pragma once

// ObstacleVoxelGrid — the 3D log-odds accumulator behind ObstacleGrid. Columns are the
// same n x n xy lattice; each column is a stack of k height bins over the obstacle band
// (above the floor). A registered scan marks the voxel each point lands in and 3D-ray-
// clears the voxels a beam actually passes through (at its true height), so a beam that
// flies OVER a low obstacle no longer carves it. voxel_project_to_2d() collapses each
// column to its max belief -> the 2D ObstacleGrid the planner/costmap consume.
//
// Anchoring: the z axis is HEIGHT ABOVE THE FLOOR (bin 0 bottom = min_height_m), which is
// stable even as odom z drifts; the caller passes floor_z_odom to convert a point's odom
// z into a height. The xy lattice is odom-anchored like the 2D grid.

#include "lio/lio_cloud.hpp"          // LioCloud — registered scan (odom frame)
#include "route_planner/obstacle_grid_builder/obstacle_grid.hpp"  // ObstacleGridConfig, ObstacleGrid (projection out)

#include <cmath>
#include <cstdint>
#include <vector>

namespace kist {

struct ObstacleVoxelGrid {
    int     n = 0;                             // n x n columns
    int     k = 0;                             // height bins per column
    float   resolution   = 0.10f;              // xy m/cell
    float   resolution_z = 0.10f;              // z  m/bin
    float   origin_x = 0.f, origin_y = 0.f;    // odom coord of column (0,0) corner
    float   min_height = 0.12f;                // height-above-floor at bin 0 bottom
    int64_t stamp_ns = 0;
    std::vector<float> log_odds;               // n*n*k; index = (iy*n + ix)*k + iz

    int  index(int ix, int iy, int iz) const { return (iy * n + ix) * k + iz; }
    bool col_to_cell(float wx, float wy, int& ix, int& iy) const {
        ix = int(std::floor((wx - origin_x) / resolution));
        iy = int(std::floor((wy - origin_y) / resolution));
        return ix >= 0 && ix < n && iy >= 0 && iy < n;
    }
    // Height (above floor) -> bin index; may be outside [0,k) (caller checks).
    int  height_to_bin(float h) const { return int(std::floor((h - min_height) / resolution_z)); }
    bool empty() const { return log_odds.empty(); }
};

// Allocate/clear to the configured size, centred on (center_x,center_y) in odom.
void voxel_reset(ObstacleVoxelGrid& v, const ObstacleGridConfig& cfg,
                 float center_x, float center_y);

// Rolling window: slide the grid so it stays centred on (center_x,center_y) as the robot
// moves. The origin only moves in whole cells, so existing voxels keep their odom meaning
// (accumulation stays valid); columns shifted in from outside are cleared to unknown, and
// columns that fall off the far edge are dropped. Sub-cell moves are a no-op. Call before
// decay/integrate each frame.
void voxel_recenter(ObstacleVoxelGrid& v, float center_x, float center_y);

// One temporal step: pull every voxel toward the prior (log_odds *= decay).
void voxel_decay(ObstacleVoxelGrid& v, const ObstacleGridConfig& cfg);

// Integrate one registered scan (odom). ray origin = the lidar centre in odom (3D, from
// T_odom_lidar); floor_z_odom = the floor plane's z in odom. Each in-band point marks its
// voxel; a 3D beam from the sensor voxel to that voxel clears the voxels it passes.
void voxel_integrate(ObstacleVoxelGrid& v, const LioCloud& scan,
                     float ray_ox, float ray_oy, float ray_oz, float floor_z_odom,
                     const ObstacleGridConfig& cfg);

// Column-max projection into the 2D ObstacleGrid (allocated/filled here). A cell's belief
// is the strongest voxel in its column, so a low obstacle keeps a solid footprint.
void voxel_project_to_2d(const ObstacleVoxelGrid& v, ObstacleGrid& out);

} // namespace kist
