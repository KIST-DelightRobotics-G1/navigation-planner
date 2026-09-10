#pragma once

// LioOdometry — the sensor pose the LIO engine estimates (from /Odometry_loc): the
// Mid-360 IMU body pose in the stable odom frame = T_odom_lidar_imu. `pose` is a
// StampedTransform (parent=Odom, child=LidarImu) carrying the timestamp; velocity is the
// odom-frame linear velocity (0 if the engine omits twist).
//
// This is the engine's honest output (the IMU body). To get the LiDAR scan centre,
// compose the static lidar_imu->lidar edge (Mid-360 datasheet ext); the transform tree
// does that when you lookup(Odom, Lidar) — see lio_tree_producer.hpp.

#include "transforms/stamped_transform.hpp"   // StampedTransform (+ FrameId, Transform)

#include <Eigen/Core>

namespace kist {

struct LioOdometry {
    StampedTransform pose;                                       // T_odom_lidar (+ stamp)
    Eigen::Vector3d  linear_velocity = Eigen::Vector3d::Zero();  // m/s, odom frame
};

} // namespace kist
