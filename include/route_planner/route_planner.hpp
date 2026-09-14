#pragma once

// RoutePlanner — the route-planning pipeline, wired in one place. It owns the four stages
// (obstacle_grid_builder -> costmap_builder -> astar_planner -> route_smoothing) and the
// data that flows between them, and exposes two operations:
//
//   update_map(scan, T_odom_lidar, T_odom_pelvis)  [stateful]  -> the rolling voxel grid is
//     integrated and projected to a 2D ObstacleGrid + Costmap (call per LIO scan).
//   plan(costmap, start, goal)                     [stateless] -> clearance field -> A* route
//     -> bubble smoother -> Path (raw + smoothed + turn circles).
//
// Folder separation is code organization, NOT threading: this is a single computation unit.
// The threads / DDS live outside (system/), and drive these methods.

#include "lio/lio_cloud.hpp"
#include "lio/robot_transforms.hpp"
#include "route_planner/obstacle_grid_builder/obstacle_grid.hpp"
#include "route_planner/obstacle_grid_builder/obstacle_voxel_grid.hpp"
#include "route_planner/costmap_builder/costmap.hpp"
#include "route_planner/costmap_builder/costmap_builder.hpp"
#include "route_planner/astar_planner/astar_planner.hpp"
#include "route_planner/astar_planner/path.hpp"
#include "route_planner/route_smoothing/route_smoother.hpp"

#include <utility>

namespace kist {

class RoutePlanner {
public:
    RoutePlanner() = default;

    // ── stage configs (tweak before/while running; read on the perception/planner thread) ──
    ObstacleGridConfig gcfg;      // grid + height band
    CostmapConfig      ccfg;      // inflation radii / cost shape
    AStarConfig        acfg;      // route (centre preference)
    RouteSmoothConfig  scfg;      // shape (LOS + arc)
    float              floor_ema = 0.05f;   // floor_z low-pass (gait bob)

    // Stateful map update — integrate one registered scan at its stamp-matched pose
    // (RobotTransforms carries T_odom_lidar = ray origin, T_odom_pelvis = robot base).
    void update_map(const LioCloud& scan, const RobotTransforms& tf);
    const ObstacleGrid& grid()    const { return grid_; }
    const Costmap&      costmap() const { return costmap_; }

    // Stateless route planning on a costmap snapshot (carries the robot pose as `start` here).
    Path plan(const Costmap& cm, std::pair<float, float> start, std::pair<float, float> goal) const;

private:
    ObstacleVoxelGrid vgrid_;      // rolling accumulator (stateful)
    CostmapBuilder    cb_;
    ObstacleGrid      grid_;       // latest projection
    Costmap           costmap_;    // latest cost field

    bool  grid_ready_  = false;
    bool  floor_init_  = false;
    float floor_z_ema_ = 0.0f;
};

} // namespace kist
