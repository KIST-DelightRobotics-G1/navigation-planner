#pragma once

// ObstacleMapper — the PERCEPTION half of the nav pipeline: turn registered LIO scans into a
// live rolling occupancy grid + costmap (odom frame). Stateful: a 3D voxel grid is integrated
// over time (log-odds + decay), projected to a 2D ObstacleGrid, then inflated into a Costmap.
// This is the "what is around me right now" model the planner routes over (NOT the saved prior
// map). Folder separation is code organization, NOT threading — the perception thread (system/)
// drives update_map(); the Costmap it produces is the handoff to the planner half.

#include "lio/lio_cloud.hpp"
#include "lio/robot_transforms.hpp"
#include "route_planner/perception/obstacle_grid_builder/obstacle_grid.hpp"
#include "route_planner/perception/obstacle_grid_builder/obstacle_voxel_grid.hpp"
#include "route_planner/perception/costmap_builder/costmap.hpp"
#include "route_planner/perception/costmap_builder/costmap_builder.hpp"

namespace kist {

class ObstacleMapper {
public:
    ObstacleMapper() = default;

    // ── stage configs (tweak before/while running; read on the perception thread) ──
    ObstacleGridConfig gcfg;                 // grid + height band
    CostmapConfig      ccfg;                 // inflation radii / cost shape
    float              floor_ema = 0.05f;    // floor_z low-pass (gait bob)

    // Stateful map update — integrate one registered scan at its stamp-matched pose
    // (RobotTransforms carries T_odom_lidar = ray origin, T_odom_pelvis = robot base).
    void update_map(const LioCloud& scan, const RobotTransforms& tf);
    const ObstacleGrid& grid()    const { return grid_; }
    const Costmap&      costmap() const { return costmap_; }

private:
    ObstacleVoxelGrid vgrid_;      // rolling accumulator (stateful)
    CostmapBuilder    cb_;
    ObstacleGrid      grid_;       // latest projection
    Costmap           costmap_;    // latest cost field (handoff to the planner)

    bool  grid_ready_  = false;
    bool  floor_init_  = false;
    float floor_z_ema_ = 0.0f;
};

} // namespace kist
