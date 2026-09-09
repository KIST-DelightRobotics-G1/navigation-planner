#pragma once

// LIO output types — the pose (T_odom_lidar) and the registered cloud. Kept in their
// own header so any consumer (the transform-tree producer, the occupancy grid) can
// depend on the LIO output without pulling in the LioWorker class or anything
// FAST-LIO / PCL. Only Eigen + std here.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace kist {

struct LioPose {                     // T_odom_lidar(t)
    int64_t            stamp_ns        = 0;
    Eigen::Vector3d    position        = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation     = Eigen::Quaterniond::Identity();
    Eigen::Vector3d    linear_velocity = Eigen::Vector3d::Zero();   // odom frame
};

// Motion-undistorted points placed in the odom frame via the estimated pose — the
// STABLE view (raw sensor-frame points smear obstacles as the head swings). Two
// stages upstream: raw -> IMU-undistorted (reference frame) -> registered in odom.
// Flattened xyz + intensity, no PCL type (keeps the PIMPL boundary clean).
struct LioCloud {
    std::vector<float> xyz;          // x0 y0 z0 x1 y1 z1 ...  (odom frame)
    std::vector<float> intensity;    // per point
    std::size_t point_count() const { return xyz.size() / 3; }
};

// One LIO step's output: the pose AND the registered cloud, bundled.
struct LioResult {
    int64_t  stamp_ns = 0;
    LioPose  pose;                   // T_odom_lidar (+ velocity)
    LioCloud cloud;                  // registered points, frame = odom
    bool     valid = false;          // false while the filter is still initializing
};

} // namespace kist
