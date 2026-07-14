#pragma once

#include <cstdint>
#include <string>

namespace kist {

// One odometry sample — the output of UnitreeOdometryReader. Lives next
// to its reader, same pairing as unitree_pointcloud.hpp.
//
// Raw SDK values, forwarded as-is: odom-frame pose drifts over time and
// has no global reference. Global (map-frame) localisation is the EKF
// fusion stage's job (odom predict + UWB update), not this reader's.
struct UnitreeOdometry {
    // Sensor timestamp from the Odometry header.
    int64_t stamp_ns = 0;

    // Coordinate frame; arrives as "odom" from the locomotion
    // controller (verified live — the onboard relay's rewrite to the
    // same name is a no-op).
    std::string frame_id;

    // pose (position + orientation quaternion)
    float px = 0.0f, py = 0.0f, pz = 0.0f;
    float qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 1.0f;

    // twist (body-frame linear / angular velocity)
    float vx = 0.0f, vy = 0.0f, vz = 0.0f;
    float wx = 0.0f, wy = 0.0f, wz = 0.0f;
};

} // namespace kist
