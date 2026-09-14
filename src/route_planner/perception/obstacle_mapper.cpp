#include "route_planner/perception/obstacle_mapper.hpp"

#include <cmath>

namespace kist {

namespace {
double quat_yaw(const Eigen::Quaterniond& q) {
    return std::atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y() + q.z()*q.z()));
}
}  // namespace

void ObstacleMapper::update_map(const LioCloud& scan, const RobotTransforms& tf) {
    const Transform& T_odom_lidar  = tf.T_odom_lidar;    // ray origin (3D carve) + band top
    const Transform& T_odom_pelvis = tf.T_odom_pelvis;   // robot base: grid centre, floor, pose
    const float px = float(T_odom_pelvis.translation.x());
    const float py = float(T_odom_pelvis.translation.y());
    const float lx = float(T_odom_lidar.translation.x());
    const float ly = float(T_odom_lidar.translation.y());
    const float lz = float(T_odom_lidar.translation.z());
    const float floor_raw = float(T_odom_pelvis.translation.z()) - gcfg.pelvis_stand_height_m;
    if (!floor_init_) { floor_z_ema_ = floor_raw; floor_init_ = true; }
    else floor_z_ema_ += floor_ema * (floor_raw - floor_z_ema_);   // low-pass the gait bob

    if (!grid_ready_) { voxel_reset(vgrid_, gcfg, px, py); grid_ready_ = true; }
    voxel_recenter(vgrid_, px, py);
    voxel_decay(vgrid_, gcfg);
    voxel_integrate(vgrid_, scan, lx, ly, lz, floor_z_ema_, gcfg);

    voxel_project_to_2d(vgrid_, grid_);
    grid_.robot_x = px; grid_.robot_y = py;
    grid_.robot_yaw = float(quat_yaw(T_odom_pelvis.rotation));

    costmap_ = cb_.build(grid_, gcfg, ccfg);
}

} // namespace kist
