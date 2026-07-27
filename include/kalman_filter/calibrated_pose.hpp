#pragma once

#include <cstdint>
#include <string>

namespace kist {

// Fused global pose — the output of PoseFilter (calibrated_pose_buf).
//
// The type name is the contract: this is only ever produced once the EKF is
// initialized AND its yaw bias has converged, so every CalibratedPose carries
// a trustworthy heading. Until then the buffer stays empty (move the robot to
// calibrate) — "has data" already means "calibrated", so no flag is needed.
struct CalibratedPose {
    int64_t     stamp_ns = 0;   // odom sample timestamp that produced it
    float       x = 0.0f;       // UWB/map frame position (m)
    float       y = 0.0f;
    float       yaw = 0.0f;     // global heading (rad) = odom_yaw + b_theta
    std::string frame_id;       // output frame (config, default "map")
};

} // namespace kist
