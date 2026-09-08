#pragma once

// One sample from the Mid-360's built-in IMU — the output of
// UnitreePointCloudImuReader. Lives next to its producer, the same pairing as
// unitree_pointcloud.hpp / unitree_pointcloud_reader.hpp.
//
// Frame-neutral (no DDS type), so estimators / LIO consume it without depending on
// the robot's message layer. accel is in m/s^2 (the reader converts from the
// Livox IMU's native g); gyro in rad/s. Both are in the livox_frame.

#include <Eigen/Core>

#include <cstdint>

namespace kist {

struct ImuSample {
    int64_t         stamp_ns = 0;
    Eigen::Vector3d gyro     = Eigen::Vector3d::Zero();   // rad/s, livox_frame
    Eigen::Vector3d accel    = Eigen::Vector3d::Zero();   // m/s^2, includes gravity
};

} // namespace kist
