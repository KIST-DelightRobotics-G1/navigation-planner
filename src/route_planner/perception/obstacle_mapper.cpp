#include "route_planner/perception/obstacle_mapper.hpp"

#include <cmath>

namespace kist {

namespace {
double quat_yaw(const Eigen::Quaterniond& q) {
    return std::atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y() + q.z()*q.z()));
}
}  // namespace

void ObstacleMapper::update_map(const LioCloud& scan, const RobotTransforms& tf,
                                const Transform& T_leveled_odom) {
    // Everything into the gravity-leveled frame: robot base + ray origin (poses), and the scan.
    // T_leveled_odom is identity until the ground leveler locks, so this reduces to the old odom
    // behavior during startup. In the leveled frame the floor is the XY plane -> floor_z ~ 0.
    const Eigen::Vector3d pel = T_leveled_odom.transformPoint(tf.T_odom_pelvis.translation);
    const Eigen::Vector3d lid = T_leveled_odom.transformPoint(tf.T_odom_lidar.translation);
    const Eigen::Quaterniond pel_rot = T_leveled_odom.rotation * tf.T_odom_pelvis.rotation;
    const float px = float(pel.x()), py = float(pel.y());
    const float lx = float(lid.x()), ly = float(lid.y()), lz = float(lid.z());
    const float floor_raw = float(pel.z()) - gcfg.pelvis_stand_height_m;   // ~0 once leveled
    if (!floor_init_) { floor_z_ema_ = floor_raw; floor_init_ = true; }
    else floor_z_ema_ += floor_ema * (floor_raw - floor_z_ema_);   // low-pass the gait bob

    // Leveled copy of the scan (xy stays consistent with the leveled robot/goal — no tilt smear).
    LioCloud lev = scan;
    const std::size_t n = scan.point_count();
    for (std::size_t i = 0; i < n; ++i) {
        const Eigen::Vector3d q = T_leveled_odom.transformPoint(
            Eigen::Vector3d(scan.xyz[3*i], scan.xyz[3*i+1], scan.xyz[3*i+2]));
        lev.xyz[3*i] = float(q.x()); lev.xyz[3*i+1] = float(q.y()); lev.xyz[3*i+2] = float(q.z());
    }

    if (!grid_ready_) { voxel_reset(vgrid_, gcfg, px, py); grid_ready_ = true; }
    voxel_recenter(vgrid_, px, py);
    voxel_decay(vgrid_, gcfg);
    voxel_integrate(vgrid_, lev, lx, ly, lz, floor_z_ema_, gcfg);

    voxel_project_to_2d(vgrid_, grid_);
    grid_.robot_x = px; grid_.robot_y = py;
    grid_.robot_yaw = float(quat_yaw(pel_rot));

    costmap_ = cb_.build(grid_, gcfg, ccfg);
}

} // namespace kist
