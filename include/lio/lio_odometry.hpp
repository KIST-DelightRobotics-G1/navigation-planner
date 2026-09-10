#pragma once

// LioOdometry — the sensor pose the LIO engine estimates (from /Odometry_loc). This is
// the T_odom_lidar the whole LIO stack exists to produce: the Mid-360 body pose in the
// stable odom frame. `pose` is a StampedTransform (parent=Odom, child=Lidar) carrying
// the timestamp; velocity is the odom-frame linear velocity (0 if the engine omits twist).
//
// (Strictly the engine reports T_odom_body (IMU body); we label child=Lidar and ignore
// the ~4 cm Mid-360 imu<->lidar offset — the registered cloud is already exactly in odom.)

#include "transforms/stamped_transform.hpp"   // StampedTransform (+ FrameId, Transform)

#include <Eigen/Core>

namespace kist {

struct LioOdometry {
    StampedTransform pose;                                       // T_odom_lidar (+ stamp)
    Eigen::Vector3d  linear_velocity = Eigen::Vector3d::Zero();  // m/s, odom frame
};

} // namespace kist
